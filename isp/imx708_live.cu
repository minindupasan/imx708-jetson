// SPDX-License-Identifier: BSD-2-Clause
// Copyright (C) 2019-2025, Raspberry Pi Ltd (control algorithms, tuning data and DNG layout
//   ported from libcamera src/ipa/rpi and rpicam-apps)
// Copyright (C) 2026, imx708-jetson contributors (Jetson port, CUDA ISP)
//
// imx708-live: live video from the Raspberry Pi Camera Module 3 (IMX708) on Jetson,
// bypassing the Argus ISP (whose NITO tuning leaves the black level in).
//
//   V4L2 RAW10 capture -> CUDA ISP (black level, lens shading, WB, demosaic, CCM, gamma)
//   -> GStreamer appsrc (zero-copy) -> user-supplied sink pipeline
//
// The control algorithms follow Raspberry Pi's libcamera IPA (rpi.agc, rpi.awb, rpi.alsc,
// rpi.ccm, rpi.contrast, rpi.af) using the official IMX708 tuning (rpi_imx708_tuning.h).
// Per-frame rates in the tuning assume ~30 fps; they are rescaled to the real frame rate.
//
// Video: frames go to a GStreamer sink pipeline. Stills (--still): runs AE/AWB/AF for --timeout
// ms, then saves one frame as jpeg/png/bmp/yuv420/rgb and optionally a DNG raw (--raw). A sink
// pipeline given with --still gets the video frames until then.
//
// stdin commands, after libcamera's AF controls:
//   f                   auto mode (AfModeAuto) and start a scan (AfTrigger)
//   c                   continuous mode (AfModeContinuous)
//   w X Y W H [X Y ...] AF windows (AfWindows), fractions of the frame, up to 10; w alone for
//                       the default window
//   m CODE              manual mode, lens at CODE (445 = infinity .. 925 = closest)
//   q                   quit

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <mutex>
#include <vector>

#include <fcntl.h>
#include <glob.h>
#include <linux/i2c-dev.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <cuda_runtime.h>
#include <tiffio.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include "rpi_imx708_tuning.h"

constexpr float BLACK = 64.f, WHITE = 1023.f;  // 10-bit (RPi: 4096 in 16-bit)
constexpr double TUNING_FPS = 30.0;             // frame rate the RPi per-frame constants assume

// Sensor modes, in the order of the driver's mode table / device-tree mode nodes
// (the index is the V4L2 sensor_mode value).
struct SensorMode {
	const char* name;
	int w, h, fps;  // fps = maximum
	bool superpixel;       // full-res mode: one output pixel per Bayer quad (half size)
	double line_us;        // sensor line time (line_length_pix / pixel rate)
	int crop_x, crop_y;    // readout origin on the 4608x2592 array
	int bin;               // sensor pixels per raw pixel
};
static const SensorMode MODES[] = {
	{"full", 4608, 2592, 14, true, 15648 / 595.2, 0, 0, 1},
	{"binned", 2304, 1296, 56, false, 7824 / 585.6, 0, 0, 2},
	{"crop", 1536, 864, 120, false, 5216 / 566.4, 768, 432, 2},
	{"hdr", 2304, 1296, 30, false, 5216 / 777.6, 0, 0, 2},  // sensor HDR (long/medium/short merged on-chip)
};
constexpr int FULL_W = 4608, FULL_H = 2592;

constexpr int ZX = 16, ZY = 12, NZ = ZX * ZY;  // AGC/AWB/ALSC zone grid (as on the Pi)
constexpr int HIST_BINS = 128;
constexpr int GRID_X = 8;  // column segments of the per-row profile (flicker detector)
constexpr int FX = 64, FY = 48, NF = FX * FY;  // focus statistics grid
constexpr int CELL = 6;                         // floats per focus cell (see focus_kernel)

// V4L2 controls exposed by the tegracam imx708 driver
constexpr uint32_t CID_SENSOR_MODE = 0x009a2008;
constexpr uint32_t CID_GAIN = 0x009a2009;        // 16 = 1x
constexpr uint32_t CID_EXPOSURE = 0x009a200a;    // microseconds
constexpr uint32_t CID_FRAME_RATE = 0x009a200b;  // fps * 1e6
constexpr uint32_t CID_BYPASS = 0x009a2064;

#define CK(x)                                                                                      \
	do {                                                                                           \
		cudaError_t e_ = (x);                                                                      \
		if (e_ != cudaSuccess) {                                                                   \
			fprintf(stderr, "CUDA error %s:%d: %s\n", __FILE__, __LINE__, cudaGetErrorString(e_)); \
			exit(1);                                                                               \
		}                                                                                          \
	} while (0)

static volatile sig_atomic_t g_quit = 0;
static void on_signal(int) { g_quit = 1; }

static double now_s() {
	timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

// Piecewise-linear function over (x, y) pairs, clamped at the ends.
static double pwl(const double* xy, int n, double x) {
	if (x <= xy[0]) return xy[1];
	for (int i = 1; i < n; i++)
		if (x <= xy[2 * i]) {
			const double x0 = xy[2 * i - 2], x1 = xy[2 * i];
			const double t = x1 > x0 ? (x - x0) / (x1 - x0) : 0;
			return xy[2 * i - 1] + t * (xy[2 * i + 1] - xy[2 * i - 1]);
		}
	return xy[2 * n - 1];
}

// Per-frame IIR speed tuned at 30 fps, converted to the same time constant at `fps`.
static double rescale_speed(double speed, int fps) { return 1.0 - pow(1.0 - speed, TUNING_FPS / fps); }
static int rescale_frames(int frames, int fps) { return std::max(1, (int)lround(frames * fps / TUNING_FPS)); }

// ---------------------------------------------------------------- GPU

// Stats layout (floats)
constexpr int ST_ZONE = 0;                           // NZ x {R, G, B, n} over all pixels (AGC)
constexpr int ST_ZONE_UNSAT = ST_ZONE + NZ * 4;      // NZ x {R, G, B, n} unsaturated only (AWB)
constexpr int ST_HIST = ST_ZONE_UNSAT + NZ * 4;      // Y histogram
constexpr int ST_COUNT = ST_HIST + HIST_BINS;

struct IspParams {
	float gain[3];      // WB gain * digital gain per channel (R, G, B)
	float ccm[9];
	float sat;
	int crop_x, crop_y, bin;  // raw -> full-array coordinates for the lens-shading tables
};

struct StatParams {
	int qw, qh;
	int box;  // focus measure on 2x2 quad boxes (full-resolution mode)
	int crop_x, crop_y, bin;
};

__constant__ float c_lsc[3][NZ];       // lens-shading gains R, G, B on the 16x12 grid
__constant__ float c_gamma[1025];      // gamma LUT, linear [0,1] -> [0,1]

__device__ __forceinline__ float raw_px(const uint16_t* raw, int stride, int y, int x) {
	// Tegra VI stores RAW10 in the top bits of a 16-bit word.
	float v = float(raw[y * stride + x] >> 6) - BLACK;
	return fmaxf(v, 0.f) * (1.f / (WHITE - BLACK));
}

__device__ __forceinline__ float quad_green(const uint16_t* raw, int stride, int qy, int qx) {
	return 0.5f * (raw_px(raw, stride, 2 * qy, 2 * qx + 1) + raw_px(raw, stride, 2 * qy + 1, 2 * qx));
}

__device__ __forceinline__ float focus_green(const uint16_t* raw, int stride, int qy, int qx, int box) {
	if (!box) return quad_green(raw, stride, qy, qx);
	return 0.25f * (quad_green(raw, stride, qy, qx) + quad_green(raw, stride, qy, qx + 1) +
	                quad_green(raw, stride, qy + 1, qx) + quad_green(raw, stride, qy + 1, qx + 1));
}

// Bilinear lookup of the 16x12 lens-shading grid at a position on the full pixel array.
__device__ __forceinline__ void lsc_at(float sx, float sy, float g[3]) {
	const float fx = fminf(fmaxf(sx * (ZX / (float)FULL_W) - 0.5f, 0.f), ZX - 1.001f);
	const float fy = fminf(fmaxf(sy * (ZY / (float)FULL_H) - 0.5f, 0.f), ZY - 1.001f);
	const int x0 = (int)fx, y0 = (int)fy;
	const float ax = fx - x0, ay = fy - y0;
	for (int c = 0; c < 3; c++) {
		const float* t = c_lsc[c];
		const float top = t[y0 * ZX + x0] * (1 - ax) + t[y0 * ZX + x0 + 1] * ax;
		const float bot = t[(y0 + 1) * ZX + x0] * (1 - ax) + t[(y0 + 1) * ZX + x0 + 1] * ax;
		g[c] = top * (1 - ay) + bot * ay;
	}
}

__device__ __forceinline__ float gamma_lut(float v) {
	v = fminf(fmaxf(v, 0.f), 1.f) * 1024.f;
	const int i = min((int)v, 1023);
	const float a = v - i;
	return c_gamma[i] * (1.f - a) + c_gamma[i + 1] * a;
}

// Linear camera RGB (after LSC/WB) -> CCM -> gamma -> 8-bit, with highlight desaturation.
__device__ __forceinline__ uchar4 finish(float r, float g, float b, float raw_peak, const IspParams& p) {
	float R = p.ccm[0] * r + p.ccm[1] * g + p.ccm[2] * b;
	float G = p.ccm[3] * r + p.ccm[4] * g + p.ccm[5] * b;
	float B = p.ccm[6] * r + p.ccm[7] * g + p.ccm[8] * b;
	// Near sensor saturation the colour gains would tint clipped highlights (typically pink):
	// fade towards neutral as the brightest raw channel approaches clipping.
	const float clip = fminf(fmaxf((raw_peak - 0.85f) * (1.f / 0.12f), 0.f), 1.f);
	const float m = (R + G + B) * (1.f / 3.f);
	const float sat = p.sat * (1.f - clip);
	R = m + (R - m) * sat;
	G = m + (G - m) * sat;
	B = m + (B - m) * sat;
	const float top = fmaxf(R, fmaxf(G, B));
	R += (top - R) * clip;
	G += (top - G) * clip;
	B += (top - B) * clip;
	return make_uchar4((unsigned char)(gamma_lut(R) * 255.f + 0.5f), (unsigned char)(gamma_lut(G) * 255.f + 0.5f),
	                   (unsigned char)(gamma_lut(B) * 255.f + 0.5f), 255);
}

// Statistics on the Bayer quad grid, after lens shading, before WB:
// zone means for AGC (all pixels) and AWB (unsaturated) and a Y histogram.
__global__ void stats_kernel(const uint16_t* __restrict__ raw, int stride, StatParams sp, float* __restrict__ stats) {
	__shared__ float zones[2][NZ][4];
	__shared__ unsigned int hist[HIST_BINS];
	const int tid = threadIdx.y * blockDim.x + threadIdx.x, nthreads = blockDim.x * blockDim.y;
	for (int i = tid; i < 2 * NZ * 4; i += nthreads) (&zones[0][0][0])[i] = 0.f;
	for (int i = tid; i < HIST_BINS; i += nthreads) hist[i] = 0;
	__syncthreads();

	const int qx = blockIdx.x * blockDim.x + threadIdx.x;
	const int qy = blockIdx.y * blockDim.y + threadIdx.y;
	if (qx < sp.qw && qy < sp.qh) {
		float r = raw_px(raw, stride, 2 * qy, 2 * qx);
		float g = quad_green(raw, stride, qy, qx);
		float b = raw_px(raw, stride, 2 * qy + 1, 2 * qx + 1);
		const bool saturated = fmaxf(r, fmaxf(g, b)) > 0.97f;
		float lsc[3];
		lsc_at(sp.crop_x + (2 * qx + 1) * sp.bin, sp.crop_y + (2 * qy + 1) * sp.bin, lsc);
		r *= lsc[0], g *= lsc[1], b *= lsc[2];

		const int z = (qy * ZY / sp.qh) * ZX + qx * ZX / sp.qw;
		atomicAdd(&zones[0][z][0], r);
		atomicAdd(&zones[0][z][1], g);
		atomicAdd(&zones[0][z][2], b);
		atomicAdd(&zones[0][z][3], 1.f);
		if (!saturated) {
			atomicAdd(&zones[1][z][0], r);
			atomicAdd(&zones[1][z][1], g);
			atomicAdd(&zones[1][z][2], b);
			atomicAdd(&zones[1][z][3], 1.f);
		}
		const float y = 0.299f * r + 0.587f * g + 0.114f * b;
		atomicAdd(&hist[min((int)(y * HIST_BINS), HIST_BINS - 1)], 1u);

	}
	__syncthreads();
	for (int i = tid; i < 2 * NZ; i += nthreads) {
		const float* zz = &zones[0][0][0] + i * 4;
		if (zz[3] > 0) {
			const int base = (i < NZ ? ST_ZONE : ST_ZONE_UNSAT) + (i % NZ) * 4;
			for (int k = 0; k < 4; k++) atomicAdd(&stats[base + k], zz[k]);
		}
	}
	for (int i = tid; i < HIST_BINS; i += nthreads)
		if (hist[i]) atomicAdd(&stats[ST_HIST + i], (float)hist[i]);
}

// Focus statistics for the AF (like the Pi ISP's focus and AWB regions, on a finer grid): one
// block per cell of the FX x FY grid, writing {sum of squared Laplacian of green, samples,
// sum R, sum G, sum B, quads}. In full-resolution mode the Laplacian is taken on 2x2 quad boxes,
// so it sees the same detail as in the binned modes.
__global__ void focus_kernel(const uint16_t* __restrict__ raw, int stride, StatParams sp, float* __restrict__ cells) {
	const int x0 = blockIdx.x * sp.qw / FX, x1 = (blockIdx.x + 1) * sp.qw / FX;
	const int y0 = blockIdx.y * sp.qh / FY, y1 = (blockIdx.y + 1) * sp.qh / FY;
	const int cw = x1 - x0, n = cw * (y1 - y0), d = sp.box ? 2 : 1, edge = 3;
	float acc[CELL] = {0};
	for (int i = threadIdx.x; i < n; i += blockDim.x) {
		const int qx = x0 + i % cw, qy = y0 + i / cw;
		acc[2] += raw_px(raw, stride, 2 * qy, 2 * qx);
		acc[3] += quad_green(raw, stride, qy, qx);
		acc[4] += raw_px(raw, stride, 2 * qy + 1, 2 * qx + 1);
		acc[5] += 1.f;
		if (qx % d || qy % d || qx < edge || qy < edge || qx >= sp.qw - edge || qy >= sp.qh - edge) continue;
		const float lap = 4.f * focus_green(raw, stride, qy, qx, sp.box) - focus_green(raw, stride, qy - d, qx, sp.box) -
		                  focus_green(raw, stride, qy + d, qx, sp.box) - focus_green(raw, stride, qy, qx - d, sp.box) -
		                  focus_green(raw, stride, qy, qx + d, sp.box);
		acc[0] += lap * lap;
		acc[1] += 1.f;
	}
	__shared__ float part[32][CELL];
	const int lane = threadIdx.x % 32, warp = threadIdx.x / 32, warps = blockDim.x / 32;
	for (int k = 0; k < CELL; k++) {
		float v = acc[k];
		for (int o = 16; o > 0; o /= 2) v += __shfl_down_sync(0xffffffff, v, o);
		if (!lane) part[warp][k] = v;
	}
	__syncthreads();
	if (threadIdx.x < CELL) {
		float v = 0.f;
		for (int w = 0; w < warps; w++) v += part[w][threadIdx.x];
		cells[(blockIdx.y * FX + blockIdx.x) * CELL + threadIdx.x] = v;
	}
}

// Mean green per quad row, split into GRID_X column segments (flicker detector input).
__global__ void rows_kernel(const uint16_t* __restrict__ raw, int stride, int qw, float* __restrict__ rowseg) {
	__shared__ float seg[GRID_X];
	const int qy = blockIdx.x;
	if (threadIdx.x < GRID_X) seg[threadIdx.x] = 0.f;
	__syncthreads();
	float local[GRID_X] = {0};
	for (int qx = threadIdx.x; qx < qw; qx += blockDim.x) local[qx * GRID_X / qw] += quad_green(raw, stride, qy, qx);
	for (int i = 0; i < GRID_X; i++)
		if (local[i] != 0.f) atomicAdd(&seg[i], local[i]);
	__syncthreads();
	if (threadIdx.x < GRID_X) rowseg[qy * GRID_X + threadIdx.x] = seg[threadIdx.x] / (qw / GRID_X);
}

// Full-resolution mode: one RGB pixel per RGGB quad.
__global__ void render_superpixel(const uint16_t* __restrict__ raw, int stride, int qw, int qh, uchar4* __restrict__ out,
                                  IspParams p) {
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= qw || y >= qh) return;
	const float r = raw_px(raw, stride, 2 * y, 2 * x), g = quad_green(raw, stride, y, x),
	            b = raw_px(raw, stride, 2 * y + 1, 2 * x + 1);
	float lsc[3];
	lsc_at(p.crop_x + (2 * x + 1) * p.bin, p.crop_y + (2 * y + 1) * p.bin, lsc);
	out[y * qw + x] = finish(r * lsc[0] * p.gain[0], g * lsc[1] * p.gain[1], b * lsc[2] * p.gain[2],
	                         fmaxf(r, fmaxf(g, b)), p);
}

// Binned modes: Malvar-He-Cutler 5x5 demosaic at full output resolution (RGGB), on
// white-balanced, shading-corrected samples.
__global__ void render_mhc(const uint16_t* __restrict__ raw, int stride, int w, int h, uchar4* __restrict__ out,
                           IspParams p) {
	const int x = blockIdx.x * blockDim.x + threadIdx.x;
	const int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (x >= w || y >= h) return;
	float lsc[3];
	lsc_at(p.crop_x + (x + 0.5f) * p.bin, p.crop_y + (y + 0.5f) * p.bin, lsc);
	const float cg[3] = {lsc[0] * p.gain[0], lsc[1] * p.gain[1], lsc[2] * p.gain[2]};
	float peak = 0.f;
	// Mirror at the borders, preserving the Bayer phase (step by 2).
	auto v = [&](int dy, int dx) {
		int yy = y + dy, xx = x + dx;
		if (yy < 0) yy += 2 * ((-yy + 1) / 2);
		if (yy >= h) yy -= 2 * ((yy - h) / 2 + 1);
		if (xx < 0) xx += 2 * ((-xx + 1) / 2);
		if (xx >= w) xx -= 2 * ((xx - w) / 2 + 1);
		const float s = raw_px(raw, stride, yy, xx);
		const int ch = (yy & 1) + (xx & 1);  // 0 = R, 1 = G, 2 = B
		return s * cg[ch];
	};
	const float c = v(0, 0);
	const float n4 = v(-1, 0) + v(1, 0) + v(0, -1) + v(0, 1);
	const float d4 = v(-1, -1) + v(-1, 1) + v(1, -1) + v(1, 1);
	const float h2 = v(0, -2) + v(0, 2), v2 = v(-2, 0) + v(2, 0);
	const float hn = v(0, -1) + v(0, 1), vn = v(-1, 0) + v(1, 0);

	const float g_rb = (4.f * c + 2.f * n4 - h2 - v2) * 0.125f;
	const float along_row = (5.f * c + 4.f * hn - d4 - h2 + 0.5f * v2) * 0.125f;
	const float along_col = (5.f * c + 4.f * vn - d4 - v2 + 0.5f * h2) * 0.125f;
	const float diag = (6.f * c + 2.f * d4 - 1.5f * (h2 + v2)) * 0.125f;

	float r, g, b;
	const bool even_row = !(y & 1), even_col = !(x & 1);
	if (even_row && even_col) {
		r = c, g = g_rb, b = diag;
	} else if (even_row) {
		r = along_row, g = c, b = along_col;
	} else if (even_col) {
		r = along_col, g = c, b = along_row;
	} else {
		r = diag, g = g_rb, b = c;
	}
	peak = fmaxf(r / cg[0], fmaxf(g / cg[1], b / cg[2]));
	out[y * w + x] = finish(fmaxf(r, 0.f), fmaxf(g, 0.f), fmaxf(b, 0.f), peak, p);
}

// ---------------------------------------------------------------- V4L2

struct Camera {
	int fd = -1;
	int w = 0, h = 0;
	int stride = 0;  // in uint16 elements
	size_t frame_bytes = 0;
	std::vector<void*> bufs;
	std::vector<size_t> lens;

	void set_ctrl64(uint32_t id, int64_t v) {
		v4l2_ext_control c{};
		c.id = id;
		c.value64 = v;
		v4l2_ext_controls cs{};
		cs.which = V4L2_CTRL_WHICH_CUR_VAL;
		cs.count = 1;
		cs.controls = &c;
		if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &cs) < 0) fprintf(stderr, "set ctrl 0x%x: %s\n", id, strerror(errno));
	}

	bool open_dev(const char* dev, int mode_idx, int fps) {
		const SensorMode& m = MODES[mode_idx];
		fd = open(dev, O_RDWR | O_NONBLOCK);
		if (fd < 0) return perror(dev), false;

		v4l2_control bc{CID_BYPASS, 0};  // stream straight to user space
		if (ioctl(fd, VIDIOC_S_CTRL, &bc) < 0) perror("bypass_mode");
		set_ctrl64(CID_SENSOR_MODE, mode_idx);  // DT uses use_sensor_mode_id

		v4l2_format f{};
		f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		f.fmt.pix.width = m.w;
		f.fmt.pix.height = m.h;
		f.fmt.pix.pixelformat = V4L2_PIX_FMT_SRGGB10;
		f.fmt.pix.field = V4L2_FIELD_NONE;
		if (ioctl(fd, VIDIOC_S_FMT, &f) < 0) return perror("VIDIOC_S_FMT"), false;
		if ((int)f.fmt.pix.width != m.w || (int)f.fmt.pix.height != m.h) {
			fprintf(stderr, "driver gave %ux%u instead of %dx%d (driver/overlay without this mode?)\n",
			        f.fmt.pix.width, f.fmt.pix.height, m.w, m.h);
			return false;
		}
		w = m.w;
		h = m.h;
		stride = f.fmt.pix.bytesperline / 2;
		frame_bytes = (size_t)f.fmt.pix.bytesperline * h;
		set_ctrl64(CID_FRAME_RATE, (int64_t)fps * 1000000);

		v4l2_requestbuffers rb{};
		rb.count = 3;  // few buffers = low latency
		rb.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		rb.memory = V4L2_MEMORY_MMAP;
		if (ioctl(fd, VIDIOC_REQBUFS, &rb) < 0) return perror("VIDIOC_REQBUFS"), false;
		for (unsigned i = 0; i < rb.count; i++) {
			v4l2_buffer b{};
			b.type = rb.type;
			b.memory = rb.memory;
			b.index = i;
			if (ioctl(fd, VIDIOC_QUERYBUF, &b) < 0) return perror("VIDIOC_QUERYBUF"), false;
			void* p = mmap(nullptr, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
			if (p == MAP_FAILED) return perror("mmap"), false;
			bufs.push_back(p);
			lens.push_back(b.length);
			if (ioctl(fd, VIDIOC_QBUF, &b) < 0) return perror("VIDIOC_QBUF"), false;
		}
		int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) return perror("VIDIOC_STREAMON"), false;
		return true;
	}

	int dequeue(double* ts) {
		v4l2_buffer b{};
		b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		b.memory = V4L2_MEMORY_MMAP;
		if (ioctl(fd, VIDIOC_DQBUF, &b) < 0) return -1;
		*ts = b.timestamp.tv_sec + b.timestamp.tv_usec * 1e-6;
		return b.index;
	}

	void requeue(int i) {
		v4l2_buffer b{};
		b.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		b.memory = V4L2_MEMORY_MMAP;
		b.index = i;
		if (ioctl(fd, VIDIOC_QBUF, &b) < 0) perror("VIDIOC_QBUF");
	}

	~Camera() {
		if (fd < 0) return;
		int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		ioctl(fd, VIDIOC_STREAMOFF, &type);
		for (size_t i = 0; i < bufs.size(); i++) munmap(bufs[i], lens[i]);
		close(fd);
	}
};

// ---------------------------------------------------------------- focus motor

// DW9817 VCM on the sensor's I2C bus. Like the Raspberry Pi dw9807-vcm driver, moves are
// ramped in steps of 16 codes 1 ms apart, which keeps the lens quiet and free of ringing.
struct Vcm {
	static constexpr int RAMP_STEP = 16;
	int fd = -1;
	int pos = -1;
	bool powered = false;

	bool open_bus() {
		glob_t g;
		if (glob("/sys/bus/i2c/drivers/imx708/*-001a", 0, nullptr, &g) != 0) return false;
		const char* name = strrchr(g.gl_pathv[0], '/') + 1;
		char dev[64];
		snprintf(dev, sizeof dev, "/dev/i2c-%d", atoi(name));
		globfree(&g);
		fd = open(dev, O_RDWR);
		if (fd < 0) return perror(dev), false;
		if (ioctl(fd, I2C_SLAVE_FORCE, 0x0c) < 0) return perror("I2C_SLAVE_FORCE"), false;
		return true;
	}

	bool write_pos(int p) {
		uint8_t cmd[3] = {0x03, (uint8_t)(p >> 8), (uint8_t)(p & 0xff)};
		return write(fd, cmd, 3) == 3;
	}

	// The VCM is only powered while the sensor streams, so (re)power it lazily.
	bool set(int p) {
		if (fd < 0) return false;
		p = std::clamp(p, 0, 1023);
		if (p == pos) return true;
		if (!powered) {
			uint8_t on[2] = {0x02, 0x00};
			if (write(fd, on, 2) != 2) return false;
			powered = true;
			pos = -1;
		}
		if (pos >= 0) {
			while (abs(p - pos) > RAMP_STEP) {
				pos += p > pos ? RAMP_STEP : -RAMP_STEP;
				if (!write_pos(pos)) return powered = false, false;
				usleep(1000);
			}
		}
		if (!write_pos(p)) return powered = false, false;
		pos = p;
		return true;
	}
};

// Lens position <-> dioptres, from the tuning's "map" (linear).
static double code_to_dioptres(double c) {
	return rpi::AF_MAP_D0 + (c - rpi::AF_MAP_C0) * (rpi::AF_MAP_D1 - rpi::AF_MAP_D0) / (rpi::AF_MAP_C1 - rpi::AF_MAP_C0);
}
static int dioptres_to_code(double d) {
	return (int)lround(rpi::AF_MAP_C0 +
	                   (d - rpi::AF_MAP_D0) * (rpi::AF_MAP_C1 - rpi::AF_MAP_C0) / (rpi::AF_MAP_D1 - rpi::AF_MAP_D0));
}

// ---------------------------------------------------------------- autofocus (rpi.af)

// Port of libcamera's rpi.af. On the Pi it follows the phase-detect (PDAF) data the IMX708 sends
// in its embedded lines, and falls back to contrast scans when those aren't good enough. The
// Jetson VI doesn't pass embedded lines to V4L2, so this is rpi.af's contrast path on its own,
// the way it runs on the Pi with a sensor that has no PDAF:
//
// - The focus measure and the R, G, B means are taken over the AF windows (up to 10, merged by
//   area) on the focus statistics grid, or by default over the middle half of the width and
//   middle third of the height.
// - Auto mode scans when triggered. Continuous mode scans when it is entered, then again each
//   time the contrast or colour in the windows changes by more than the retrigger ratio and
//   then stays steady for retrigger_delay frames.
// - A scan steps the lens coarsely until the contrast drops off (in continuous mode starting
//   from where the lens is, and turning round if it set off the wrong way), finely back across
//   the peak, then moves to the top of a parabola through the best three points.
//
// One addition: new windows restart a scan in progress, as rpi.af does on a sensor mode switch,
// because the contrast recorded so far was measured somewhere else.
struct AfWindow {
	double x, y, w, h;  // fractions of the frame
};

struct AutoFocus {
	enum Mode { MANUAL, AUTO, CONTINUOUS } mode = MANUAL;
	enum class State { Idle, Scanning, Focused, Failed };  // libcamera's AfState
	enum class Scan { Idle, Trigger, Coarse1, Coarse2, Fine, Settle } scan = Scan::Idle;
	static constexpr size_t MAX_WINDOWS = 10;

	// tuning, rescaled to the frame rate
	double max_slew = rpi::AF_MAX_SLEW;
	int step_frames = rpi::AF_STEP_FRAMES, retrigger_delay = rpi::AF_RETRIGGER_DELAY, skip_frames = rpi::AF_SKIP_FRAMES;

	std::vector<AfWindow> windows;
	std::vector<double> weights;  // per focus cell; empty until computed for the current windows

	bool initted = false;
	double ftarget = -1, fsmooth = -1;
	double prev_contrast = 0, old_scene_contrast = 0;
	double prev_average[3] = {0, 0, 0}, old_scene_average[3] = {0, 0, 0};
	int skip_count = 0, step_count = 0, scene_change_count = 0;
	size_t scan_max_index = 0;
	double scan_max_contrast = 0, scan_min_contrast = 1e9, scan_step = 0;
	struct ScanRecord {
		double focus, contrast;
	};
	std::vector<ScanRecord> scan_data;
	State report_state = State::Idle;

	void configure(int fps) {
		max_slew = rpi::AF_MAX_SLEW * TUNING_FPS / fps;
		step_frames = rescale_frames(rpi::AF_STEP_FRAMES, fps);
		retrigger_delay = rescale_frames(rpi::AF_RETRIGGER_DELAY, fps);
		skip_frames = rescale_frames(rpi::AF_SKIP_FRAMES, fps);
	}

	// ---- controls (libcamera's AfMode, AfTrigger, AfMetering + AfWindows, LensPosition)

	void set_mode(Mode m) {
		if (mode == m) return;
		mode = m;
		if (m == CONTINUOUS)
			scan = Scan::Trigger;
		else if (m != AUTO || scan < Scan::Coarse1)
			go_idle();
	}

	void trigger_scan() {
		if (mode == AUTO && scan == Scan::Idle) scan = Scan::Trigger;
	}

	// No windows: the default one.
	void set_windows(const std::vector<AfWindow>& w) {
		windows.assign(w.begin(), w.begin() + std::min(w.size(), MAX_WINDOWS));
		weights.clear();
		if (scan >= Scan::Coarse1 && scan < Scan::Settle) start_programmed_scan();
	}

	// Manual mode, or forced (the starting position). Limited by the lens map, not the scan range.
	void set_lens_position(double dioptres, bool force = false) {
		if (mode != MANUAL && !force) return;
		ftarget = std::clamp(dioptres, rpi::AF_MAP_D0, rpi::AF_MAP_D1);
		update_lens_position();
	}

	// ---- per frame

	// Focus measure and colour means in the windows, from the focus statistics (NF cells of
	// {sharpness, samples, R, G, B, quads}). Scaled to the magnitudes of the Pi's statistics; the
	// contrast is divided by the mean green squared, so it doesn't change with exposure.
	void process(const float* cells) {
		if (weights.empty()) compute_weights();
		double sharp = 0, samples = 0, sum[3] = {0, 0, 0}, quads = 0;
		for (int i = 0; i < NF; i++) {
			const double w = weights[i];
			if (w == 0) continue;
			const float* c = &cells[i * CELL];
			sharp += w * c[0];
			samples += w * c[1];
			for (int k = 0; k < 3; k++) sum[k] += w * c[2 + k];
			quads += w * c[5];
		}
		quads = std::max(quads, 1e-9);
		const double g = sum[1] / quads;
		prev_contrast = sharp / std::max(samples, 1e-9) / std::max(g * g, 1e-6) * 1000.0;
		for (int k = 0; k < 3; k++) prev_average[k] = sum[k] / quads * 1000.0;
	}

	// Runs the algorithm on the statistics from process() and sets the lens for the next frame.
	void prepare() {
		if (scan == Scan::Trigger) start_af();
		if (initted) {
			do_af(prev_contrast);
			update_lens_position();
		}
	}

	State state() const {
		if (mode == AUTO && scan != Scan::Idle) return State::Scanning;
		if (mode == MANUAL) return State::Idle;
		return report_state;
	}

	const char* state_name() const {
		if (mode == MANUAL) return "manual";
		switch (state()) {
		case State::Scanning:
			return "scanning";
		case State::Focused:
			return "focused";
		case State::Failed:
			return "failed";
		default:
			return "idle";
		}
	}

	// ---- rpi.af

	// Weight of each focus cell: how much of it the windows cover, or 1 in the default window.
	void compute_weights() {
		weights.assign(NF, 0.0);
		double sum = 0;
		for (const AfWindow& w : windows) {
			for (int r = 0; r < FY; r++) {
				const double h = std::min((r + 1.0) / FY, w.y + w.h) - std::max((double)r / FY, w.y);
				if (h <= 0) continue;
				for (int c = 0; c < FX; c++) {
					const double a = h * (std::min((c + 1.0) / FX, w.x + w.w) - std::max((double)c / FX, w.x));
					if (a <= 0) continue;
					weights[r * FX + c] += a;
					sum += a;
				}
			}
		}
		if (sum == 0) {
			for (int r = FY / 3; r < FY - FY / 3; r++)
				for (int c = FX / 4; c < FX - FX / 4; c++) weights[r * FX + c] = 1;
		}
	}

	// Lens position with the most contrast: a parabola through the best sample and its neighbours
	// (or the two on one side, at the end of a scan).
	double find_peak(size_t i) const {
		double f = scan_data[i].focus;
		if (scan_data.size() >= 3) {
			if (i == 0)
				i++;
			else if (i + 1 >= scan_data.size())
				i--;
			const double abx = scan_data[i - 1].focus - scan_data[i].focus;
			const double aby = scan_data[i - 1].contrast - scan_data[i].contrast;
			const double cbx = scan_data[i + 1].focus - scan_data[i].focus;
			const double cby = scan_data[i + 1].contrast - scan_data[i].contrast;
			const double denom = 2.0 * (aby * cbx - cby * abx);
			if (fabs(denom) >= 1.0 / 64.0 && denom * abx > 0.0) {
				f = (aby * cbx * cbx - cby * abx * abx) / denom;
				f = std::clamp(f, std::min(abx, cbx), std::max(abx, cbx));
				f += scan_data[i].focus;
			}
		}
		return f;
	}

	void do_scan(double contrast) {
		// Record lens position and contrast for the current scan
		if (scan_data.empty() || contrast > scan_max_contrast) {
			scan_max_contrast = contrast;
			scan_max_index = scan_data.size();
			if (scan != Scan::Fine) std::copy(prev_average, prev_average + 3, old_scene_average);
		}
		scan_min_contrast = std::min(scan_min_contrast, contrast);
		scan_data.push_back({ftarget, contrast});

		const double lo = rpi::AF_FOCUS_MIN, hi = rpi::AF_FOCUS_MAX, fine = rpi::AF_STEP_FINE;
		if ((scan_step >= 0.0 && ftarget >= hi) || (scan_step <= 0.0 && ftarget <= lo) ||
		    (scan == Scan::Fine && scan_data.size() >= 3) || contrast < rpi::AF_CONTRAST_RATIO * scan_max_contrast) {
			// Finished a scan, at a limit or because the contrast dropped off. If this was the first
			// coarse scan and the peak wasn't bracketed, reverse. After a fine scan, we're done.
			// Otherwise start a fine scan in the opposite direction.
			const double pk = find_peak(scan_max_index);
			if (scan == Scan::Coarse1 && scan_data[0].contrast >= rpi::AF_CONTRAST_RATIO * scan_max_contrast) {
				scan_step = -scan_step;
				scan = Scan::Coarse2;
			} else if (scan == Scan::Fine || fine <= 0.0) {
				ftarget = pk;
				scan = Scan::Settle;
			} else if (scan_step >= 0.0) {
				ftarget = std::min(pk + fine, hi);
				scan_step = -fine;
				scan = Scan::Fine;
			} else {
				ftarget = std::max(pk - fine, lo);
				scan_step = fine;
				scan = Scan::Fine;
			}
			scan_data.clear();
		} else {
			ftarget += scan_step;
		}
		step_count = (ftarget == fsmooth) ? 0 : step_frames;
	}

	void do_af(double contrast) {
		// Skip frames at startup
		if (skip_count > 0) {
			skip_count--;
			return;
		}
		if (mode == MANUAL) return;

		if (scan < Scan::Coarse1 && mode == CONTINUOUS) {
			// Not scanning: wait for a scene change, followed by stability.
			const double r = rpi::AF_RETRIGGER_RATIO;
			bool changed = contrast + 1.0 < r * old_scene_contrast || old_scene_contrast + 1.0 < r * contrast;
			for (int c = 0; c < 3; c++)
				changed |= prev_average[c] + 1.0 < r * old_scene_average[c] ||
				           old_scene_average[c] + 1.0 < r * prev_average[c];
			if (changed) {
				old_scene_contrast = contrast;
				std::copy(prev_average, prev_average + 3, old_scene_average);
				scene_change_count = 1;
			} else if (scene_change_count) {
				scene_change_count++;
			}
			if (scene_change_count >= retrigger_delay) start_programmed_scan();
		} else if (scan >= Scan::Coarse1 && fsmooth == ftarget) {
			// Scanning. Wait step_frames for the statistics to catch up with the lens between steps,
			// and to settle at the end.
			if (step_count > 0) {
				step_count--;
			} else if (scan == Scan::Settle) {
				report_state = prev_contrast >= rpi::AF_CONTRAST_RATIO * scan_max_contrast &&
				                       scan_min_contrast <= rpi::AF_CONTRAST_RATIO * scan_max_contrast
				                   ? State::Focused
				                   : State::Failed;
				scan = Scan::Idle;
				scene_change_count = 0;
				old_scene_contrast = std::max(scan_max_contrast, prev_contrast);
				scan_data.clear();
			} else {
				do_scan(contrast);
			}
		}
	}

	void update_lens_position() {
		if (scan >= Scan::Coarse1) ftarget = std::clamp(ftarget, rpi::AF_FOCUS_MIN, rpi::AF_FOCUS_MAX);
		if (initted) {
			// from a known lens position: apply the slew rate limit
			fsmooth = std::clamp(ftarget, fsmooth - max_slew, fsmooth + max_slew);
		} else {
			// from an unknown position: go straight to the target, but skip frames
			fsmooth = ftarget;
			initted = true;
			skip_count = skip_frames;
		}
	}

	void start_af() {
		start_programmed_scan();
		update_lens_position();
	}

	void start_programmed_scan() {
		const double lo = rpi::AF_FOCUS_MIN, hi = rpi::AF_FOCUS_MAX, coarse = rpi::AF_STEP_COARSE;
		if (!initted || mode != CONTINUOUS || fsmooth <= lo + 2.0 * coarse) {
			ftarget = lo;
			scan_step = coarse;
			scan = Scan::Coarse2;
		} else if (fsmooth >= hi - 2.0 * coarse) {
			ftarget = hi;
			scan_step = -coarse;
			scan = Scan::Coarse2;
		} else {
			scan_step = -coarse;
			scan = Scan::Coarse1;
		}
		scan_max_contrast = 0.0;
		scan_min_contrast = 1e9;
		scan_max_index = 0;
		scan_data.clear();
		step_count = step_frames;
		report_state = State::Scanning;
	}

	void go_idle() {
		scan = Scan::Idle;
		report_state = State::Idle;
		scan_data.clear();
	}
};

// ---------------------------------------------------------------- flicker

// Detects mains-powered light flicker (100 Hz on 50 Hz mains, 120 Hz on 60 Hz) from
// rolling-shutter banding. Each sensor row is exposed at a known instant, so the row
// profile's deviation from its running average is correlated against 100/120 Hz over
// ~1.5 s of absolute time: flicker adds up coherently, scene content and noise do not.
struct FlickerDetector {
	double period_us = 0;  // exposure quantum; 0 = no flicker detected
	bool fixed = false;    // frequency forced from the command line
	std::vector<double> avg;
	double c[2][2] = {{0, 0}, {0, 0}};
	long n = 0;
	double t_start = -1;

	void force(int mains_hz) {
		fixed = true;
		period_us = mains_hz > 0 ? 1e6 / (2.0 * mains_hz) : 0;
	}

	void update(const float* rowseg, int rows, double ts, double row_dt) {
		if (fixed) return;
		std::vector<double> p(rows);
		double m = 0;
		for (int r = 0; r < rows; r++) {
			double v = 0;
			for (int i = 0; i < GRID_X; i++) v += rowseg[r * GRID_X + i];
			p[r] = v;
			m += v;
		}
		m /= rows;
		if (m < 1e-5) return;
		if (avg.empty()) {
			for (auto& v : p) v /= m;
			avg = p;
			t_start = ts;
			return;
		}
		static const double w[2] = {2 * M_PI * 100, 2 * M_PI * 120};
		for (int r = 0; r < rows; r++) {
			const double v = p[r] / m, res = v - avg[r];
			avg[r] += 0.05 * (v - avg[r]);
			const double t = ts + r * row_dt;
			for (int k = 0; k < 2; k++) {
				c[k][0] += res * cos(w[k] * t);
				c[k][1] -= res * sin(w[k] * t);
			}
			n++;
		}
		if (ts - t_start < 1.5) return;
		const double a100 = 2 * hypot(c[0][0], c[0][1]) / n, a120 = 2 * hypot(c[1][0], c[1][1]) / n;
		const double hi = fmax(a100, a120), lo = fmin(a100, a120);
		if (hi > 0.002 && hi > 2.5 * lo) {  // keep the last detection when banding disappears
			const double p_us = a100 > a120 ? 1e6 / 100 : 1e6 / 120;
			if (p_us != period_us)
				fprintf(stderr, "\nflicker detected: %d Hz lighting (amplitude %.1f%%)\n", a100 > a120 ? 100 : 120,
				        hi * 100);
			period_us = p_us;
		}
		c[0][0] = c[0][1] = c[1][0] = c[1][1] = 0;
		n = 0;
		t_start = ts;
	}
};

// ---------------------------------------------------------------- statistics (host view)

struct ZoneStats {
	double r[NZ], g[NZ], b[NZ], n[NZ];  // sums
};

// Centre-weighted metering: the Pi's weights [3,3,3,2,2,2,2,1,1,1,1,0,0,0,0] are for its
// 15 concentric AGC regions; here they are applied by elliptical distance on the 16x12 grid.
static void centre_weights(double w[NZ]) {
	for (int y = 0; y < ZY; y++)
		for (int x = 0; x < ZX; x++) {
			const double dx = (x + 0.5 - ZX / 2.0) / (ZX / 2.0), dy = (y + 0.5 - ZY / 2.0) / (ZY / 2.0);
			const double d = sqrt(dx * dx + dy * dy);
			w[y * ZX + x] = d < 0.35 ? 3 : d < 0.65 ? 2 : d < 1.0 ? 1 : 0;
		}
}

// ---------------------------------------------------------------- AGC (rpi.agc)

struct Agc {
	double speed = rpi::AGC_SPEED;
	int startup_frames = rpi::AGC_STARTUP_FRAMES;
	double max_exposure_us = 66666;
	double weights[NZ];
	int frame = 0;
	double filtered_total = 0;  // exposure_us * analogue gain * digital gain
	double exposure_us = 10000, again = 1.0, dgain = 1.0;
	double flicker_us = 0;
	double lux = 400, target_y = 0.16, measured_y = 0;

	void configure(int fps) {
		speed = rescale_speed(rpi::AGC_SPEED, fps);
		startup_frames = rescale_frames(rpi::AGC_STARTUP_FRAMES, fps);
		// exposure must fit in the frame (driver clamps to frame_length - 48 lines)
		max_exposure_us = std::min(rpi::AGC_SHUTTER_US[4], 1e6 / fps * 0.97);
		centre_weights(weights);
	}

	static double limit_exposure(double e, double max_e) { return std::clamp(e, 100.0, max_e); }

	// Mean Y of the metered zones if the image were brightened by `gain` (with per-zone clipping).
	double initial_y(const ZoneStats& z, const double wb[3], double gain) const {
		double sr = 0, sg = 0, sb = 0, np = 0;
		for (int i = 0; i < NZ; i++) {
			if (z.n[i] <= 0 || weights[i] <= 0) continue;
			sr += weights[i] * std::min(z.r[i] * gain, z.n[i]);
			sg += weights[i] * std::min(z.g[i] * gain, z.n[i]);
			sb += weights[i] * std::min(z.b[i] * gain, z.n[i]);
			np += weights[i] * z.n[i];
		}
		if (np <= 0) return 0;
		const double min_wb = std::max(std::min({wb[0], wb[1], wb[2], 1.0}), 1.0);
		sr *= wb[0] / min_wb, sg *= wb[1] / min_wb, sb *= wb[2] / min_wb;
		return (0.299 * sr + 0.587 * sg + 0.114 * sb) / np;
	}

	// Mean of the histogram between two quantiles, in [0,1).
	static double inter_quantile_mean(const float* hist, double qlo, double qhi) {
		double total = 0;
		for (int i = 0; i < HIST_BINS; i++) total += hist[i];
		if (total <= 0) return 1;
		const double lo = qlo * total, hi = qhi * total;
		double cum = 0, sum = 0, cnt = 0;
		for (int i = 0; i < HIST_BINS; i++) {
			const double a = cum, b = cum + hist[i];
			const double take = std::max(0.0, std::min(b, hi) - std::max(a, lo));
			sum += take * (i + 0.5);
			cnt += take;
			cum = b;
		}
		return cnt > 0 ? sum / cnt / HIST_BINS : 1;
	}

	// frame_exposure_us / frame_again: what this frame was actually exposed with.
	void process(const ZoneStats& z, const float* hist, const double wb[3], double full_y, double frame_exposure_us,
	             double frame_again) {
		frame++;
		// rpi.lux
		lux = (rpi::LUX_REF_EXPOSURE_US / frame_exposure_us) * (rpi::LUX_REF_GAIN / frame_again) *
		      (full_y / rpi::LUX_REF_Y) * rpi::LUX_REF_LUX;

		target_y = std::min(0.9, pwl(rpi::AGC_Y_TARGET, 3, lux));
		measured_y = initial_y(z, wb, 1.0);
		double gain = 1.0;
		for (int i = 0; i < 8; i++) {
			const double y = i ? initial_y(z, wb, gain) : measured_y;
			const double extra = std::min(10.0, target_y / (y + 0.001));
			gain *= extra;
			if (extra < 1.01) break;
		}
		// "normal" constraint: the brightest 2% must reach at least y_target 0.2 (lower bound)
		const double cy = std::min(0.9, pwl(rpi::AGC_CONSTRAINT_Y, 2, lux));
		const double cgain = cy / inter_quantile_mean(hist, rpi::AGC_CONSTRAINT_QLO, rpi::AGC_CONSTRAINT_QHI);
		if (cgain > gain) gain = cgain;

		// target, limited by what the exposure mode allows
		double target = frame_exposure_us * frame_again * gain;
		target = std::min(target, limit_exposure(rpi::AGC_SHUTTER_US[4], max_exposure_us) * rpi::AGC_GAIN[4]);

		// filter
		double sp = speed, stable = 0.02;
		if (frame <= startup_frames) sp = 1.0, stable = 0.0;
		if (!filtered_total) {
			filtered_total = target;
		} else if (!(filtered_total * (1 - stable) < target && filtered_total * (1 + stable) > target)) {
			if (filtered_total < 1.2 * target && filtered_total > 0.8 * target) sp = sqrt(sp);
			filtered_total = sp * target + (1 - sp) * filtered_total;
		}
		divide_up();
	}

	// Split the total exposure into exposure time and gain along the "normal" exposure mode.
	void divide_up() {
		const double value = filtered_total;
		double e = limit_exposure(rpi::AGC_SHUTTER_US[0], max_exposure_us);
		double g = rpi::AGC_GAIN[0];
		if (e * g < value) {
			for (int stage = 1; stage < 5; stage++) {
				const double se = limit_exposure(rpi::AGC_SHUTTER_US[stage], max_exposure_us);
				if (se * g >= value) {
					e = value / g;
					break;
				}
				e = se;
				if (rpi::AGC_GAIN[stage] * e >= value) {
					g = value / e;
					break;
				}
				g = rpi::AGC_GAIN[stage];
			}
		}
		if (flicker_us > 0) {  // whole flicker periods only
			const int periods = (int)(e / flicker_us);
			if (periods) {
				const double ne = periods * flicker_us;
				g *= e / ne;
				e = ne;
			}
		}
		again = std::min(g, 16.0);
		exposure_us = e;
		const double no_dg = again * exposure_us;
		dgain = std::clamp(filtered_total / no_dg, 1.0, 4.0);
		filtered_total = no_dg * dgain;
	}
};

// ---------------------------------------------------------------- AWB (rpi.awb, Bayesian)

struct Awb {
	double speed = 0.05;
	int startup_frames = 10;
	int frame = 0;
	double gain_r = 1.6, gain_b = 1.6, ct = 4500;  // filtered output

	void configure(int fps) {
		speed = rescale_speed(0.05, fps);
		startup_frames = rescale_frames(10, fps);
	}

	static int ct_points() { return (int)(sizeof(rpi::AWB_CT_CURVE) / sizeof(double) / 3); }
	static double ct_r(double t) { return ct_eval(t, 1); }
	static double ct_b(double t) { return ct_eval(t, 2); }
	static double ct_eval(double t, int col) {
		const double* c = rpi::AWB_CT_CURVE;
		const int n = ct_points();
		if (t <= c[0]) return c[col];
		for (int i = 1; i < n; i++)
			if (t <= c[3 * i]) {
				const double t0 = c[3 * i - 3], t1 = c[3 * i];
				const double a = t1 > t0 ? (t - t0) / (t1 - t0) : 0;
				return c[3 * i - 3 + col] + a * (c[3 * i + col] - c[3 * i - 3 + col]);
			}
		return c[3 * (n - 1) + col];
	}

	struct Zone {
		double r, b;  // R/G, B/G
	};
	std::vector<Zone> zones;

	double delta2_sum(double gr, double gb) const {
		double s = 0;
		for (auto& z : zones) {
			const double dr = gr * z.r - 1, db = gb * z.b - 1;
			s += std::min(dr * dr + db * db, 0.2);  // delta_limit
		}
		return s;
	}

	// Prior log-likelihood over CT, interpolated for the current lux.
	double prior(double t, double lux) const {
		const double* p[3] = {rpi::AWB_PRIOR0, rpi::AWB_PRIOR1, rpi::AWB_PRIOR2};
		const int n[3] = {rpi::AWB_PRIOR0_N, rpi::AWB_PRIOR1_N, rpi::AWB_PRIOR2_N};
		const double l[3] = {rpi::AWB_PRIOR0_LUX, rpi::AWB_PRIOR1_LUX, rpi::AWB_PRIOR2_LUX};
		t = std::clamp(t, 2000.0, 13000.0);
		if (lux <= l[0]) return pwl(p[0], n[0], t);
		if (lux >= l[2]) return pwl(p[2], n[2], t);
		const int i = lux < l[1] ? 0 : 1;
		const double a = (lux - l[i]) / (l[i + 1] - l[i]);
		return pwl(p[i], n[i], t) * (1 - a) + pwl(p[i + 1], n[i + 1], t) * a;
	}

	static double interp_quadratic(double ax, double ay, double bx, double by, double cx, double cy) {
		const double cax = cx - ax, cay = cy - ay, bax = bx - ax, bay = by - ay;
		const double den = 2 * (bay * cax - cay * bax);
		if (fabs(den) > 1e-3) return std::clamp((bay * cax * cax - cay * bax * bax) / den + ax, ax, cx);
		return ay < cy - 1e-3 ? ax : (cy < ay - 1e-3 ? cx : bx);
	}

	void process(const ZoneStats& z, double lux) {
		frame++;
		zones.clear();
		const double min_g = 32.0 / 65536;  // min_G (16-bit) as a fraction of full scale
		for (int i = 0; i < NZ; i++) {
			if (z.n[i] < 16) continue;
			const double g = z.g[i] / z.n[i];
			if (g < min_g) continue;
			const double eps = 1.0 / 65536;
			zones.push_back({z.r[i] / z.n[i] * rpi::AWB_SENS_R / (g + eps), z.b[i] / z.n[i] * rpi::AWB_SENS_B / (g + eps)});
		}
		if (zones.size() > 10) {
			const double pscale = zones.size() / (double)NZ;
			auto ll = [&](double t, double r, double b) { return delta2_sum(1 / r, 1 / b) - prior(t, lux) * pscale; };

			// coarse search along the CT curve
			std::vector<std::pair<double, double>> pts;
			size_t best = 0;
			for (double t = rpi::AWB_CT_LO;;) {
				pts.push_back({t, ll(t, ct_r(t), ct_b(t))});
				if (pts.back().second < pts[best].second) best = pts.size() - 1;
				if (t >= rpi::AWB_CT_HI) break;
				t = std::min(t + t / 10 * 0.2, rpi::AWB_CT_HI);
			}
			double t = pts[best].first;
			if (pts.size() > 2) {
				const size_t bp = std::max<size_t>(1, std::min(best, pts.size() - 2));
				t = interp_quadratic(pts[bp - 1].first, pts[bp - 1].second, pts[bp].first, pts[bp].second,
				                     pts[bp + 1].first, pts[bp + 1].second);
			}
			double r = ct_r(t), b = ct_b(t);

			// fine search: along the curve and transversely off it
			const double step = t / 10 * 0.2 * 0.1;
			int nsteps = 5;
			const double rd = ct_r(t + nsteps * step) - ct_r(t - nsteps * step);
			const double bd = ct_b(t + nsteps * step) - ct_b(t - nsteps * step);
			double tx = bd, ty = -rd;
			const double tl = sqrt(tx * tx + ty * ty);
			if (tl > 1e-3) {
				tx /= tl, ty /= tl;
				const double range = rpi::AWB_TRANSVERSE_NEG + rpi::AWB_TRANSVERSE_POS;
				const int nd = std::clamp((int)floor(range * 100 + 0.5) + 1, 3, 12);
				nsteps += nd;
				double best_ll = 0, bt = 0, br = 0, bb = 0;
				for (int i = -nsteps; i <= nsteps; i++) {
					const double tt = t + i * step;
					const double pr = prior(tt, lux) * pscale;
					const double rc = ct_r(tt), bc = ct_b(tt);
					double px[12], py[12];
					int bpnt = 0;
					for (int j = 0; j < nd; j++) {
						px[j] = -rpi::AWB_TRANSVERSE_NEG + range * j / (nd - 1);
						py[j] = delta2_sum(1 / (rc + tx * px[j]), 1 / (bc + ty * px[j])) - pr;
						if (py[j] < py[bpnt]) bpnt = j;
					}
					bpnt = std::max(1, std::min(bpnt, nd - 2));
					const double off = interp_quadratic(px[bpnt - 1], py[bpnt - 1], px[bpnt], py[bpnt], px[bpnt + 1],
					                                    py[bpnt + 1]);
					const double rt = rc + tx * off, btt = bc + ty * off;
					const double fl = delta2_sum(1 / rt, 1 / btt) - pr;
					if (bt == 0 || fl < best_ll) best_ll = fl, bt = tt, br = rt, bb = btt;
				}
				t = bt, r = br, b = bb;
			}
			const double new_r = 1 / r * rpi::AWB_SENS_R, new_b = 1 / b * rpi::AWB_SENS_B;
			const double sp = frame < startup_frames ? 1.0 : speed;
			gain_r = sp * new_r + (1 - sp) * gain_r;
			gain_b = sp * new_b + (1 - sp) * gain_b;
			ct = sp * t + (1 - sp) * ct;
		}
	}
};

// ---------------------------------------------------------------- colour (rpi.ccm, rpi.alsc, rpi.contrast)

static void ccm_for_ct(double ct, float out[9]) {
	const int n = rpi::CCM_N;
	int i = 0;
	while (i < n - 2 && ct > rpi::CCM_CT[i + 1]) i++;
	const double a = std::clamp((ct - rpi::CCM_CT[i]) / (rpi::CCM_CT[i + 1] - rpi::CCM_CT[i]), 0.0, 1.0);
	for (int k = 0; k < 9; k++) out[k] = (float)(rpi::CCM[i * 9 + k] * (1 - a) + rpi::CCM[(i + 1) * 9 + k] * a);
}

// Static part of rpi.alsc: calibrated colour shading for this CT plus luminance correction.
// The Pi reads the sensor out rotated 180 degrees (the module is mounted upside down and
// libcamera flips it); this driver does not flip, so the tables are rotated to match.
static void lsc_tables(double ct, float out[3][NZ]) {
	const double a = std::clamp((ct - rpi::ALSC_CAL_CT[0]) / (rpi::ALSC_CAL_CT[1] - rpi::ALSC_CAL_CT[0]), 0.0, 1.0);
	double cr[NZ], cb[NZ], mr = 1e9, mb = 1e9;
	for (int i = 0; i < NZ; i++) {
		cr[i] = rpi::ALSC_CAL_CR[i] * (1 - a) + rpi::ALSC_CAL_CR[NZ + i] * a;
		cb[i] = rpi::ALSC_CAL_CB[i] * (1 - a) + rpi::ALSC_CAL_CB[NZ + i] * a;
		mr = std::min(mr, cr[i]);
		mb = std::min(mb, cb[i]);
	}
	for (int i = 0; i < NZ; i++) {
		const int src = NZ - 1 - i;  // 180-degree rotation
		const double lum = (rpi::ALSC_LUMINANCE_LUT[src] - 1) * rpi::ALSC_LUMINANCE_STRENGTH + 1;
		out[0][i] = (float)(cr[src] / mr * lum);
		out[1][i] = (float)lum;
		out[2][i] = (float)(cb[src] / mb * lum);
	}
}

static void gamma_table(float out[1025]) {
	for (int i = 0; i <= 1024; i++) out[i] = (float)(pwl(rpi::GAMMA_CURVE, rpi::GAMMA_N, i / 1024.0 * 65535) / 65535);
}

// ---------------------------------------------------------------- output buffers

// Zero-copy frames for GStreamer: pinned, GPU-mapped host buffers wrapped into GstBuffers.
// A slot is reused only after GStreamer releases its buffer.
struct OutRing {
	static constexpr int N = 4;
	uchar4* host[N];
	uchar4* dev[N];
	std::atomic<bool> busy[N];
	size_t bytes = 0;

	void init(size_t b) {
		bytes = b;
		for (int i = 0; i < N; i++) {
			CK(cudaHostAlloc(&host[i], bytes, cudaHostAllocMapped));
			CK(cudaHostGetDevicePointer(&dev[i], host[i], 0));
			busy[i] = false;
		}
	}
	int acquire() {
		for (int i = 0; i < N; i++)
			if (!busy[i].load()) return i;
		return -1;
	}
	static void release(gpointer p) { static_cast<std::atomic<bool>*>(p)->store(false); }
	GstBuffer* wrap(int i) {
		busy[i] = true;
		return gst_buffer_new_wrapped_full(GST_MEMORY_FLAG_READONLY, host[i], bytes, 0, bytes, &busy[i], release);
	}
	void free_all() {
		for (int i = 0; i < N; i++) cudaFreeHost(host[i]);
	}
};

// Sensor control values take effect a couple of frames after they are written
// (libcamera's defaults: exposure 2 frames, gain 1 frame, plus our capture queue).
struct DelayedControls {
	static constexpr int EXPOSURE_DELAY = 2, GAIN_DELAY = 2, N = 8;
	double exposure[N], again[N];
	long frame = 0;
	void init(double e, double g) {
		for (int i = 0; i < N; i++) exposure[i] = e, again[i] = g;
	}
	void push(double e, double g) {  // values written after frame `frame`
		frame++;
		exposure[frame % N] = e;
		again[frame % N] = g;
	}
	double frame_exposure() const { return exposure[(frame - EXPOSURE_DELAY + 1 + N * 4) % N]; }
	double frame_again() const { return again[(frame - GAIN_DELAY + 1 + N * 4) % N]; }
};

// ---------------------------------------------------------------- still image output

// jpeg / png through GStreamer encoders.
static bool write_gst_image(const char* path, const uchar4* rgba, int w, int h, const char* encoder) {
	char desc[512];
	snprintf(desc, sizeof desc,
	         "appsrc name=s caps=video/x-raw,format=RGBA,width=%d,height=%d,framerate=0/1 ! videoconvert ! %s ! "
	         "filesink location=\"%s\"",
	         w, h, encoder, path);
	GError* err = nullptr;
	GstElement* pipe = gst_parse_launch(desc, &err);
	if (!pipe) {
		fprintf(stderr, "encoder pipeline: %s\n", err ? err->message : "?");
		return false;
	}
	GstAppSrc* src = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipe), "s"));
	gst_element_set_state(pipe, GST_STATE_PLAYING);
	const size_t bytes = (size_t)w * h * 4;
	GstBuffer* buf = gst_buffer_new_allocate(nullptr, bytes, nullptr);
	gst_buffer_fill(buf, 0, rgba, bytes);
	gst_app_src_push_buffer(src, buf);
	gst_app_src_end_of_stream(src);
	GstBus* bus = gst_element_get_bus(pipe);
	GstMessage* msg =
	    gst_bus_timed_pop_filtered(bus, 30 * GST_SECOND, (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
	const bool ok = msg && GST_MESSAGE_TYPE(msg) == GST_MESSAGE_EOS;
	if (msg) gst_message_unref(msg);
	gst_element_set_state(pipe, GST_STATE_NULL);
	gst_object_unref(bus);
	gst_object_unref(src);
	gst_object_unref(pipe);
	return ok;
}

static bool write_bmp(const char* path, const uchar4* rgba, int w, int h) {
	FILE* f = fopen(path, "wb");
	if (!f) return perror(path), false;
	const int row = (w * 3 + 3) & ~3;
	const uint32_t data = (uint32_t)row * h, file = 54 + data;
	uint8_t hdr[54] = {'B', 'M'};
	auto put32 = [&](int o, uint32_t v) { memcpy(hdr + o, &v, 4); };
	auto put16 = [&](int o, uint16_t v) { memcpy(hdr + o, &v, 2); };
	put32(2, file), put32(10, 54), put32(14, 40), put32(18, w), put32(22, h), put16(26, 1), put16(28, 24);
	put32(34, data), put32(38, 2835), put32(42, 2835);
	fwrite(hdr, 1, 54, f);
	std::vector<uint8_t> line(row, 0);
	for (int y = h - 1; y >= 0; y--) {  // bottom-up, BGR
		for (int x = 0; x < w; x++) {
			const uchar4 p = rgba[(size_t)y * w + x];
			line[3 * x] = p.z, line[3 * x + 1] = p.y, line[3 * x + 2] = p.x;
		}
		fwrite(line.data(), 1, row, f);
	}
	return fclose(f) == 0;
}

// Packed 24-bit R, G, B (rpicam "-e rgb").
static bool write_rgb(const char* path, const uchar4* rgba, int w, int h) {
	FILE* f = fopen(path, "wb");
	if (!f) return perror(path), false;
	std::vector<uint8_t> line((size_t)w * 3);
	for (int y = 0; y < h; y++) {
		for (int x = 0; x < w; x++) {
			const uchar4 p = rgba[(size_t)y * w + x];
			line[3 * x] = p.x, line[3 * x + 1] = p.y, line[3 * x + 2] = p.z;
		}
		fwrite(line.data(), 1, line.size(), f);
	}
	return fclose(f) == 0;
}

// Planar YUV 4:2:0 (I420), full-range BT.601 as used for JPEG (rpicam "-e yuv420").
static bool write_yuv420(const char* path, const uchar4* rgba, int w, int h) {
	FILE* f = fopen(path, "wb");
	if (!f) return perror(path), false;
	std::vector<uint8_t> Y((size_t)w * h), U((size_t)(w / 2) * (h / 2)), V(U.size());
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++) {
			const uchar4 p = rgba[(size_t)y * w + x];
			Y[(size_t)y * w + x] = (uint8_t)std::clamp(lround(0.299 * p.x + 0.587 * p.y + 0.114 * p.z), 0L, 255L);
		}
	for (int y = 0; y < h / 2; y++)
		for (int x = 0; x < w / 2; x++) {
			double r = 0, g = 0, b = 0;
			for (int k = 0; k < 4; k++) {
				const uchar4 p = rgba[(size_t)(2 * y + k / 2) * w + 2 * x + (k & 1)];
				r += p.x, g += p.y, b += p.z;
			}
			r /= 4, g /= 4, b /= 4;
			U[(size_t)y * (w / 2) + x] = (uint8_t)std::clamp(lround(128 - 0.168736 * r - 0.331264 * g + 0.5 * b), 0L, 255L);
			V[(size_t)y * (w / 2) + x] = (uint8_t)std::clamp(lround(128 + 0.5 * r - 0.418688 * g - 0.081312 * b), 0L, 255L);
		}
	fwrite(Y.data(), 1, Y.size(), f);
	fwrite(U.data(), 1, U.size(), f);
	fwrite(V.data(), 1, V.size(), f);
	return fclose(f) == 0;
}

// Adobe DNG of the raw Bayer frame, following rpicam-apps' dng.cpp: 16-bit CFA (RGGB),
// black/white level, ColorMatrix1 = inverse(RGB->XYZ * CCM * WB gains), AsShotNeutral.
static bool write_dng(const char* path, const uint16_t* raw, int stride, int w, int h, const float ccm[9], double gain_r,
                      double gain_b, double exposure_us, double again) {
	auto mul = [](const double* a, const double* b, double* o) {
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++) o[i * 3 + j] = a[i * 3] * b[j] + a[i * 3 + 1] * b[3 + j] + a[i * 3 + 2] * b[6 + j];
	};
	const double rgb2xyz[9] = {0.4124564, 0.3575761, 0.1804375, 0.2126729, 0.7151522,
	                           0.0721750, 0.0193339, 0.1191920, 0.9503041};
	double c[9], wb[9] = {gain_r, 0, 0, 0, 1, 0, 0, 0, gain_b}, t[9], m[9];
	for (int i = 0; i < 9; i++) c[i] = ccm[i];
	mul(rgb2xyz, c, t);
	mul(t, wb, m);
	const double det = m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) +
	                   m[2] * (m[3] * m[7] - m[4] * m[6]);
	float cam_xyz[9] = {
	    (float)((m[4] * m[8] - m[5] * m[7]) / det), (float)((m[2] * m[7] - m[1] * m[8]) / det),
	    (float)((m[1] * m[5] - m[2] * m[4]) / det), (float)((m[5] * m[6] - m[3] * m[8]) / det),
	    (float)((m[0] * m[8] - m[2] * m[6]) / det), (float)((m[2] * m[3] - m[0] * m[5]) / det),
	    (float)((m[3] * m[7] - m[4] * m[6]) / det), (float)((m[1] * m[6] - m[0] * m[7]) / det),
	    (float)((m[0] * m[4] - m[1] * m[3]) / det)};
	float neutral[3] = {(float)(1 / gain_r), 1.f, (float)(1 / gain_b)};

	// libtiff only knows the CFA tags as EXIF tags, so register them for the main IFD.
	static TIFFExtendProc parent_extender;
	static const TIFFFieldInfo cfa_fields[] = {
	    {TIFFTAG_CFAREPEATPATTERNDIM, 2, 2, TIFF_SHORT, FIELD_CUSTOM, 1, 0, (char*)"CFARepeatPatternDim"},
	    {TIFFTAG_CFAPATTERN, -1, -1, TIFF_BYTE, FIELD_CUSTOM, 1, 1, (char*)"CFAPattern"}};
	static std::once_flag once;
	std::call_once(once, [] {
		parent_extender = TIFFSetTagExtender([](TIFF* t) {
			TIFFMergeFieldInfo(t, cfa_fields, 2);
			if (parent_extender) parent_extender(t);
		});
	});
	TIFF* tif = TIFFOpen(path, "w");
	if (!tif) return false;
	const short cfa_dim[] = {2, 2};
	const uint8_t cfa[] = {0, 1, 1, 2};  // RGGB
	const uint16_t bl_dim[] = {1, 1};
	float black = BLACK;
	uint32_t white = 1023;
	TIFFSetField(tif, TIFFTAG_SUBFILETYPE, 0);
	TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, w);
	TIFFSetField(tif, TIFFTAG_IMAGELENGTH, h);
	TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 16);
	TIFFSetField(tif, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
	TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_CFA);
	TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
	TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(tif, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
	TIFFSetField(tif, TIFFTAG_MAKE, "Raspberry Pi");
	TIFFSetField(tif, TIFFTAG_MODEL, "imx708");
	TIFFSetField(tif, TIFFTAG_UNIQUECAMERAMODEL, "Raspberry Pi imx708 (Jetson)");
	TIFFSetField(tif, TIFFTAG_SOFTWARE, "imx708-live");
	TIFFSetField(tif, TIFFTAG_DNGVERSION, "\001\001\000\000");
	TIFFSetField(tif, TIFFTAG_DNGBACKWARDVERSION, "\001\000\000\000");
	TIFFSetField(tif, TIFFTAG_CFAREPEATPATTERNDIM, cfa_dim);
	TIFFSetField(tif, TIFFTAG_CFAPATTERN, 4, cfa);
	TIFFSetField(tif, TIFFTAG_BLACKLEVELREPEATDIM, bl_dim);
	TIFFSetField(tif, TIFFTAG_BLACKLEVEL, 1, &black);
	TIFFSetField(tif, TIFFTAG_WHITELEVEL, 1, &white);
	TIFFSetField(tif, TIFFTAG_COLORMATRIX1, 9, cam_xyz);
	TIFFSetField(tif, TIFFTAG_ASSHOTNEUTRAL, 3, neutral);
	TIFFSetField(tif, TIFFTAG_CALIBRATIONILLUMINANT1, 21);  // D65
	char desc[128];
	snprintf(desc, sizeof desc, "exposure %.0f us, analogue gain %.2f", exposure_us, again);
	TIFFSetField(tif, TIFFTAG_IMAGEDESCRIPTION, desc);
	std::vector<uint16_t> line(w);
	bool ok = true;
	for (int y = 0; y < h && ok; y++) {
		for (int x = 0; x < w; x++) line[x] = raw[(size_t)y * stride + x] >> 6;  // 10-bit values
		ok = TIFFWriteScanline(tif, line.data(), y, 0) == 1;
	}
	TIFFClose(tif);
	return ok;
}

static std::string encoding_for(const std::string& path, const std::string& requested) {
	if (!requested.empty()) return requested;
	const size_t dot = path.rfind('.');
	std::string ext = dot == std::string::npos ? "" : path.substr(dot + 1);
	for (auto& ch : ext) ch = tolower(ch);
	if (ext == "png" || ext == "bmp" || ext == "rgb") return ext;
	if (ext == "yuv" || ext == "yuv420") return "yuv420";
	return "jpeg";
}

// ---------------------------------------------------------------- main

// AF windows: groups of four numbers X Y W H, separated by spaces or commas.
static std::vector<AfWindow> parse_af_windows(const char* s) {
	std::vector<double> v;
	for (char* end; *s; s = end) {
		while (*s == ' ' || *s == ',') s++;
		if (!*s) break;
		const double x = strtod(s, &end);
		if (end == s) return {};
		v.push_back(x);
	}
	std::vector<AfWindow> w;
	if (v.size() % 4) return w;
	for (size_t i = 0; i < v.size(); i += 4) w.push_back({v[i], v[i + 1], v[i + 2], v[i + 3]});
	return w;
}

static void usage(const char* argv0) {
	fprintf(stderr,
	        "usage: %s [--mode full|binned|crop|hdr] [--fps N] [--af continuous|auto|off] [--af-window X,Y,W,H[,...]]\n"
	        "          [--focus CODE] [--sat X] [--flicker auto|50|60|off] [--device /dev/videoN] [SINK PIPELINE]\n"
	        "       %s --still FILE [-e jpeg|png|bmp|yuv420|rgb] [--raw] [--timeout MS] [--quality Q] [...]\n"
	        "  sensor modes (--mode, also 4608/2304/1536):\n"
	        "    full   4608x2592, up to 14 fps (video output 2304x1296; stills full resolution)\n"
	        "    binned 2304x1296 2x2 binned, up to 56 fps (video default)\n"
	        "    crop   1536x864 2x2 binned centre crop, up to 120 fps\n"
	        "    hdr    2304x1296 sensor HDR, 30 fps\n"
	        "  --fps: video frame rate, default 30 (capped at the mode's maximum)\n"
	        "  --af: continuous (default) refocuses when the scene changes; auto focuses once at the start\n"
	        "  --af-window: areas autofocus looks at, as fractions of the frame, up to 10 (default: the\n"
	        "               middle half of the width and middle third of the height)\n"
	        "  --focus: manual focus at lens position CODE (445 = infinity .. 925 = closest)\n"
	        "  SINK PIPELINE receives video/x-raw,format=RGBA (default: fakesink)\n"
	        "  --still: default mode full; encoding from the extension or -e; --raw adds FILE.dng;\n"
	        "           --timeout: ms of auto exposure/white balance/focus before the capture (default 3000);\n"
	        "           a SINK PIPELINE gets the video frames until the capture\n"
	        "  stdin: f = autofocus scan, c = continuous AF, w X Y W H [X Y W H ...] = AF windows\n"
	        "         (w alone: default), m CODE = manual focus, q = quit\n",
	        argv0, argv0);
}

int main(int argc, char** argv) {
	std::string sink = "fakesink", device = "/dev/video0";
	AutoFocus af;
	AutoFocus::Mode af_mode = AutoFocus::CONTINUOUS;
	std::vector<AfWindow> af_windows;
	int manual_focus = -1, mode_idx = -1, fps_req = 30;
	std::string still_path, encoding;
	bool want_raw = false;
	int timeout_ms = 3000, quality = 93;
	float sat = 1.0f;
	FlickerDetector flicker;
	for (int i = 1; i < argc; i++) {
		std::string a = argv[i];
		if (a == "--mode" && i + 1 < argc) {
			const std::string m = argv[++i];
			mode_idx = (m == "full" || m == "4608") ? 0 : (m == "crop" || m == "1536") ? 2 : m == "hdr" ? 3 : 1;
		} else if (a == "--fps" && i + 1 < argc) {
			fps_req = atoi(argv[++i]);
		} else if ((a == "--still" || a == "-o") && i + 1 < argc) {
			still_path = argv[++i];
		} else if ((a == "-e" || a == "--encoding") && i + 1 < argc) {
			encoding = argv[++i];
		} else if (a == "--raw" || a == "-r") {
			want_raw = true;
		} else if ((a == "--timeout" || a == "-t") && i + 1 < argc) {
			timeout_ms = atoi(argv[++i]);
		} else if ((a == "--quality" || a == "-q") && i + 1 < argc) {
			quality = atoi(argv[++i]);
		} else if (a == "--af" && i + 1 < argc) {
			std::string m = argv[++i];
			af_mode = m == "off" ? AutoFocus::MANUAL : (m == "auto" || m == "once") ? AutoFocus::AUTO : AutoFocus::CONTINUOUS;
		} else if (a == "--af-window" && i + 1 < argc) {
			af_windows = parse_af_windows(argv[++i]);
			if (af_windows.empty()) return usage(argv[0]), 1;
		} else if (a == "--focus" && i + 1 < argc) {
			manual_focus = atoi(argv[++i]);
			af_mode = AutoFocus::MANUAL;
		} else if (a == "--sat" && i + 1 < argc) {
			sat = atof(argv[++i]);
		} else if (a == "--flicker" && i + 1 < argc) {
			std::string m = argv[++i];
			if (m == "off")
				flicker.force(0);
			else if (m == "50" || m == "60")
				flicker.force(atoi(m.c_str()));
		} else if (a == "--device" && i + 1 < argc) {
			device = argv[++i];
		} else if (a == "-h" || a == "--help") {
			return usage(argv[0]), 0;
		} else {
			sink = a;
		}
	}
	const bool still = !still_path.empty();
	if (mode_idx < 0) mode_idx = still ? 0 : 1;
	const SensorMode& mode = MODES[mode_idx];
	const int fps = std::clamp(fps_req, 2, mode.fps);
	if (still) {
		encoding = encoding_for(still_path, encoding);
		if (encoding != "jpeg" && encoding != "png" && encoding != "bmp" && encoding != "yuv420" && encoding != "rgb") {
			fprintf(stderr, "unknown encoding %s\n", encoding.c_str());
			return 1;
		}
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	gst_init(&argc, &argv);

	Camera cam;
	if (!cam.open_dev(device.c_str(), mode_idx, fps)) return 1;
	Vcm vcm;
	if (!vcm.open_bus()) fprintf(stderr, "warning: focus motor bus not found, focus disabled\n");

	Agc agc;
	agc.configure(fps);
	agc.flicker_us = flicker.period_us;
	Awb awb;
	awb.configure(fps);
	// As libcamera starts the IPA: lens at the default (or the manual) position, then the controls.
	// In auto mode rpicam-apps triggers a scan as the camera starts.
	af.configure(fps);
	af.set_lens_position(manual_focus >= 0 ? code_to_dioptres(manual_focus) : rpi::AF_FOCUS_DEFAULT, true);
	af.set_windows(af_windows);
	af.set_mode(af_mode);
	af.trigger_scan();
	float* d_focus;
	CK(cudaMalloc(&d_focus, NF * CELL * sizeof(float)));
	std::vector<float> h_focus(NF * CELL);

	auto write_sensor = [&](double e, double g) {
		cam.set_ctrl64(CID_EXPOSURE, (int64_t)lround(e));
		cam.set_ctrl64(CID_GAIN, std::clamp((int64_t)lround(g * 16), (int64_t)16, (int64_t)257));
	};
	DelayedControls dc;
	dc.init(agc.exposure_us, agc.again);
	write_sensor(agc.exposure_us, agc.again);

	const int qw = cam.w / 2, qh = cam.h / 2;
	const int ow = mode.superpixel ? qw : cam.w, oh = mode.superpixel ? qh : cam.h;

	uint16_t* d_raw;
	CK(cudaMalloc(&d_raw, cam.frame_bytes));
	float* d_stats;
	CK(cudaMalloc(&d_stats, ST_COUNT * sizeof(float)));
	std::vector<float> h_stats(ST_COUNT);
	float* d_rowseg;
	CK(cudaMalloc(&d_rowseg, (size_t)qh * GRID_X * sizeof(float)));
	std::vector<float> h_rowseg((size_t)qh * GRID_X);
	OutRing ring;
	ring.init((size_t)ow * oh * 4);

	float gamma[1025], lsc[3][NZ];
	gamma_table(gamma);
	CK(cudaMemcpyToSymbol(c_gamma, gamma, sizeof gamma));
	double lsc_ct = -1;

	char caps[160];
	snprintf(caps, sizeof caps, "video/x-raw,format=RGBA,width=%d,height=%d,framerate=%d/1", ow, oh, fps);
	std::string desc = std::string("appsrc name=src is-live=true format=time do-timestamp=true caps=") + caps +
	                   " ! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 ! " + sink;
	GError* err = nullptr;
	GstElement* pipe = gst_parse_launch(desc.c_str(), &err);
	if (!pipe) {
		fprintf(stderr, "bad sink pipeline: %s\n", err ? err->message : "?");
		return 1;
	}
	GstAppSrc* src = GST_APP_SRC(gst_bin_get_by_name(GST_BIN(pipe), "src"));
	GstBus* bus = gst_element_get_bus(pipe);
	gst_element_set_state(pipe, GST_STATE_PLAYING);

	IspParams prm{};
	prm.sat = sat;
	prm.crop_x = mode.crop_x, prm.crop_y = mode.crop_y, prm.bin = mode.bin;
	// Full-res mode measures focus on 2x2-averaged quads to suppress noise.
	StatParams sp{qw, qh, mode.superpixel ? 1 : 0, mode.crop_x, mode.crop_y, mode.bin};
	const dim3 block(32, 8);
	const dim3 sgrid((qw + 31) / 32, (qh + 7) / 8), ogrid((ow + 31) / 32, (oh + 7) / 8);
	int frames = 0, frames_since = 0, dropped = 0;
	double t_last = now_s(), proc_ms = 0;
	std::string af_reported;
	fprintf(stderr, "sensor %s %dx%d @ %d fps -> %s\n", mode.name, cam.w, cam.h, fps,
	        still ? still_path.c_str() : sink.c_str());
	const double t_start = now_s();

	bool stdin_open = true;
	std::string input;  // stdin not yet split into lines
	while (!g_quit) {
		pollfd pf[2] = {{cam.fd, POLLIN, 0}, {stdin_open ? STDIN_FILENO : -1, POLLIN, 0}};
		int n = poll(pf, 2, 2000);
		if (n < 0 && errno == EINTR) continue;  // Ctrl-C: g_quit is set
		if (n <= 0) {
			fprintf(stderr, "\nno frames from sensor\n");
			break;
		}

		// stdin is read with read() rather than fgets(): stdio would hold back a second command
		// sent in the same write, and poll() wouldn't report it until more input arrived.
		bool quit = false;
		if (pf[1].revents & (POLLIN | POLLHUP)) {
			char buf[256];
			const ssize_t len = read(STDIN_FILENO, buf, sizeof buf);
			if (len > 0) {
				input.append(buf, len);
			} else if (len == 0 || errno != EINTR) {
				stdin_open = false;  // EOF: stop polling it, or poll() returns at once forever
				if (!input.empty()) input += '\n';
			}
			for (size_t nl; !quit && (nl = input.find('\n')) != std::string::npos;) {
				const std::string cmd = input.substr(0, nl);
				input.erase(0, nl + 1);
				const char* line = cmd.c_str();
				if (line[0] == 'q') {
					quit = true;
				} else if (line[0] == 'f') {
					af.set_mode(AutoFocus::AUTO);
					af.trigger_scan();
				} else if (line[0] == 'w') {
					af.set_windows(parse_af_windows(line + 1));
				} else if (line[0] == 'c') {
					af.set_mode(AutoFocus::CONTINUOUS);
				} else if (line[0] == 'm') {
					af.set_mode(AutoFocus::MANUAL);
					af.set_lens_position(code_to_dioptres(atoi(line + 1)));
				}
			}
		}
		if (quit) break;
		if (!(pf[0].revents & POLLIN)) continue;

		double ts = 0;
		const int idx = cam.dequeue(&ts);
		if (idx < 0) continue;
		const double t0 = now_s();
		CK(cudaMemcpy(d_raw, cam.bufs[idx], cam.frame_bytes, cudaMemcpyHostToDevice));
		cam.requeue(idx);

		// colour processing for this frame from the current AWB/AGC state
		if (fabs(awb.ct - lsc_ct) > 50) {
			lsc_tables(awb.ct, lsc);
			CK(cudaMemcpyToSymbol(c_lsc, lsc, sizeof lsc));
			lsc_ct = awb.ct;
		}
		prm.gain[0] = (float)(awb.gain_r * agc.dgain);
		prm.gain[1] = (float)agc.dgain;
		prm.gain[2] = (float)(awb.gain_b * agc.dgain);
		ccm_for_ct(awb.ct, prm.ccm);

		CK(cudaMemset(d_stats, 0, ST_COUNT * sizeof(float)));
		stats_kernel<<<sgrid, block>>>(d_raw, cam.stride, sp, d_stats);
		focus_kernel<<<dim3(FX, FY), 256>>>(d_raw, cam.stride, sp, d_focus);
		rows_kernel<<<qh, 256>>>(d_raw, cam.stride, qw, d_rowseg);
		const int slot = ring.acquire();
		if (slot >= 0) {
			if (mode.superpixel)
				render_superpixel<<<ogrid, block>>>(d_raw, cam.stride, qw, qh, ring.dev[slot], prm);
			else
				render_mhc<<<ogrid, block>>>(d_raw, cam.stride, cam.w, cam.h, ring.dev[slot], prm);
		}
		CK(cudaGetLastError());
		CK(cudaMemcpy(h_stats.data(), d_stats, ST_COUNT * sizeof(float), cudaMemcpyDeviceToHost));
		CK(cudaMemcpy(h_focus.data(), d_focus, NF * CELL * sizeof(float), cudaMemcpyDeviceToHost));
		CK(cudaMemcpy(h_rowseg.data(), d_rowseg, h_rowseg.size() * sizeof(float), cudaMemcpyDeviceToHost));
		CK(cudaDeviceSynchronize());
		proc_ms += (now_s() - t0) * 1e3;

		if (slot >= 0)
			gst_app_src_push_buffer(src, ring.wrap(slot));
		else
			dropped++;  // downstream still holds every output buffer

		if (GstMessage* msg = gst_bus_pop_filtered(bus, (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS))) {
			if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
				GError* e;
				gchar* dbg = nullptr;
				gst_message_parse_error(msg, &e, &dbg);
				fprintf(stderr, "\nGStreamer error: %s\n%s\n", e->message, dbg ? dbg : "");
				g_error_free(e);
				g_free(dbg);
			}
			gst_message_unref(msg);
			break;
		}

		// ---- statistics -> control algorithms
		ZoneStats zall, zunsat;
		double full_y = 0, full_n = 0;
		for (int i = 0; i < NZ; i++) {
			const float* a = &h_stats[ST_ZONE + i * 4];
			const float* u = &h_stats[ST_ZONE_UNSAT + i * 4];
			zall.r[i] = a[0], zall.g[i] = a[1], zall.b[i] = a[2], zall.n[i] = a[3];
			zunsat.r[i] = u[0], zunsat.g[i] = u[1], zunsat.b[i] = u[2], zunsat.n[i] = u[3];
			full_y += 0.299 * a[0] + 0.587 * a[1] + 0.114 * a[2];
			full_n += a[3];
		}
		full_y /= std::max(full_n, 1.0);

		flicker.update(h_rowseg.data(), qh, ts, 2 * mode.line_us * 1e-6);
		agc.flicker_us = flicker.period_us;

		const double wb[3] = {awb.gain_r, 1.0, awb.gain_b};
		agc.process(zall, &h_stats[ST_HIST], wb, full_y, dc.frame_exposure(), dc.frame_again());
		awb.process(zunsat, agc.lux);
		write_sensor(agc.exposure_us, agc.again);
		dc.push(agc.exposure_us, agc.again);
		frames++;
		if (frames == 2) {
			// Controls set before STREAMON are overwritten by the mode table and V4L2 skips
			// writes of unchanged values, so nudge each control to force a real write.
			// Frame rate first: the exposure is clamped to the frame length.
			cam.set_ctrl64(CID_FRAME_RATE, (int64_t)fps * 1000000 - 1);
			cam.set_ctrl64(CID_FRAME_RATE, (int64_t)fps * 1000000);
			cam.set_ctrl64(CID_EXPOSURE, (int64_t)agc.exposure_us + 1);
			cam.set_ctrl64(CID_GAIN, 17);
			write_sensor(agc.exposure_us, agc.again);
		}

		// AF (the first frames can be left over from before the sensor started)
		if (frames > 2) {
			af.process(h_focus.data());
			af.prepare();
			if (af_reported != af.state_name()) {
				af_reported = af.state_name();
				fprintf(stderr, "\nAF %s at %.2f dioptres (lens %d)\n", af_reported.c_str(), af.fsmooth,
				        dioptres_to_code(af.fsmooth));
			}
		}
		vcm.set(dioptres_to_code(af.fsmooth));

		// Still: once the timeout has passed and focus is not moving, capture this frame.
		const bool af_busy = af.state() == AutoFocus::State::Scanning;
		const double elapsed_ms = (now_s() - t_start) * 1e3;
		if (still && elapsed_ms >= timeout_ms && (!af_busy || elapsed_ms >= timeout_ms + 5000)) {
			fprintf(stderr, "\ncapturing: exp %.0f us, ag %.2f, dg %.2f, CT %.0fK, lens %d\n", dc.frame_exposure(),
			        dc.frame_again(), agc.dgain, awb.ct, vcm.pos);
			// full-resolution demosaic of this frame, with this frame's colour processing
			std::vector<uchar4> img((size_t)cam.w * cam.h);
			uchar4* d_img;
			CK(cudaMalloc(&d_img, img.size() * sizeof(uchar4)));
			const dim3 fgrid((cam.w + 31) / 32, (cam.h + 7) / 8);
			render_mhc<<<fgrid, block>>>(d_raw, cam.stride, cam.w, cam.h, d_img, prm);
			CK(cudaGetLastError());
			CK(cudaMemcpy(img.data(), d_img, img.size() * sizeof(uchar4), cudaMemcpyDeviceToHost));
			cudaFree(d_img);
			bool ok;
			if (encoding == "jpeg") {
				char enc[64];
				snprintf(enc, sizeof enc, "jpegenc quality=%d", quality);
				ok = write_gst_image(still_path.c_str(), img.data(), cam.w, cam.h, enc);
			} else if (encoding == "png") {
				ok = write_gst_image(still_path.c_str(), img.data(), cam.w, cam.h, "video/x-raw,format=RGB ! pngenc");
			} else if (encoding == "bmp") {
				ok = write_bmp(still_path.c_str(), img.data(), cam.w, cam.h);
			} else if (encoding == "rgb") {
				ok = write_rgb(still_path.c_str(), img.data(), cam.w, cam.h);
			} else {
				ok = write_yuv420(still_path.c_str(), img.data(), cam.w, cam.h);
			}
			fprintf(stderr, "%s %s (%s, %dx%d)\n", ok ? "wrote" : "FAILED", still_path.c_str(), encoding.c_str(), cam.w,
			        cam.h);
			if (want_raw) {
				const size_t dot = still_path.rfind('.');
				const std::string dng = (dot == std::string::npos ? still_path : still_path.substr(0, dot)) + ".dng";
				std::vector<uint16_t> raw_copy(cam.frame_bytes / 2);
				CK(cudaMemcpy(raw_copy.data(), d_raw, cam.frame_bytes, cudaMemcpyDeviceToHost));
				const bool dok = write_dng(dng.c_str(), raw_copy.data(), cam.stride, cam.w, cam.h, prm.ccm, awb.gain_r,
				                           awb.gain_b, dc.frame_exposure(), dc.frame_again());
				fprintf(stderr, "%s %s (raw %dx%d)\n", dok ? "wrote" : "FAILED", dng.c_str(), cam.w, cam.h);
				ok &= dok;
			}
			g_quit = ok ? 1 : 2;
			break;
		}

		frames_since++;
		const double t = now_s();
		if (t - t_last >= 1.0) {
			fprintf(stderr,
			        "\r%5.1f fps (gpu %.1f ms) | exp %5.0f us ag %4.1f dg %.2f %s| lux %5.0f Y %.3f/%.3f | "
			        "CT %4.0fK | AF %.2f D (lens %d) %s contrast %.0f   ",
			        frames_since / (t - t_last), proc_ms / frames_since, agc.exposure_us, agc.again, agc.dgain,
			        flicker.period_us == 10000 ? "flicker 100Hz " : flicker.period_us > 0 ? "flicker 120Hz " : "",
			        agc.lux, agc.measured_y, agc.target_y, awb.ct, af.fsmooth, vcm.pos, af.state_name(),
			        af.prev_contrast);
			if (dropped) fprintf(stderr, "drop %d ", dropped);
			t_last = t;
			frames_since = 0;
			proc_ms = 0;
		}
	}

	fprintf(stderr, "\nstopping\n");
	gst_app_src_end_of_stream(src);  // lets muxers (e.g. mp4mux) finalise the file
	if (GstMessage* msg = gst_bus_timed_pop_filtered(bus, 5 * GST_SECOND,
	                                                 (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR)))
		gst_message_unref(msg);
	gst_element_set_state(pipe, GST_STATE_NULL);
	gst_object_unref(bus);
	gst_object_unref(src);
	gst_object_unref(pipe);
	ring.free_all();
	cudaFree(d_raw);
	cudaFree(d_stats);
	cudaFree(d_focus);
	cudaFree(d_rowseg);
	return g_quit == 2 ? 1 : 0;
}
