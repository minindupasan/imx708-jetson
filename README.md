# imx708-jetson

Raspberry Pi Camera Module 3 (Sony IMX708) on the Jetson Orin Nano, JetPack 7.2.1 (L4T R39.2.1).

The existing driver from RidgeRun gets frames out of the sensor, but through Argus the picture is
washed out and pink, and gain, exposure and frame rate are off. This repo fixes the driver and
replaces the Argus ISP with a small CUDA pipeline that runs Raspberry Pi's own camera algorithms
and IMX708 tuning, so the image and the auto exposure / white balance / focus behave like they do
on a Pi.

- `kernel/`: patches for the RidgeRun driver (gain, exposure and timing fixes, extra sensor modes,
  CAM1-only overlay)
- `isp/`: `imx708-live`, V4L2 RAW10 capture → CUDA ISP → GStreamer
- `scripts/`: driver build script, a wrapper for common pipelines, manual focus helper

## Hardware

Tested on a Jetson Orin Nano Super developer kit with a Camera Module 3 (standard, with autofocus)
on the CAM1 connector. Other Orin carrier boards will need their own device tree.

## Installing

Install the build dependencies:

```sh
sudo apt install build-essential git curl device-tree-compiler nvidia-l4t-kernel-headers \
    cuda-toolkit libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libtiff-dev \
    gstreamer1.0-plugins-ugly v4l-utils i2c-tools
```

Build and install the driver. The script downloads NVIDIA's R39.2.1 sources (pass the path to
`public_sources.tbz2` if you already have it), applies RidgeRun's JetPack 7.2.1 patch and then the
patches in `kernel/`, and builds the module and overlays. Building NVIDIA's out-of-tree modules
takes a while.

```sh
git clone https://github.com/minindupasan/imx708-jetson.git
cd imx708-jetson
scripts/build-driver.sh --install
```

Select the overlay and reboot:

```sh
sudo /opt/nvidia/jetson-io/config-by-hardware.py -n 2="Camera IMX708-C"
sudo reboot
```

`Camera IMX708-C` is for a single camera on CAM1. `Camera IMX708` describes cameras on both
connectors. After rebooting, `dmesg | grep imx708` should show the sensor and `/dev/video0` should
exist.

Build the ISP:

```sh
make -C isp
```

## Usage

```sh
scripts/imx708-live.sh display                     # preview on the Jetson's screen
scripts/imx708-live.sh udp 192.168.1.10 5000       # H.264 over UDP (ffplay udp://@:5000)
scripts/imx708-live.sh file out.mp4                # H.264 recording, Ctrl-C to stop
scripts/imx708-live.sh --mode crop --fps 120 display
scripts/imx708-live.sh still photo.jpg --raw       # photo.jpg + photo.dng
```

Video defaults to 30 fps in the 2x2 binned mode. The Orin Nano has no hardware video encoder, so
H.264 is done in software by x264 at 1080p, which is good for about 30 fps. Options go before the
output (`scripts/imx708-live.sh --mode binned still photo.png`).

Sensor modes (`--mode`):

| Mode     | Size      | Max fps | Notes                                  |
|----------|-----------|---------|----------------------------------------|
| `full`   | 4608x2592 | 14      | Default for stills                     |
| `binned` | 2304x1296 | 56      | Default for video, best in low light   |
| `crop`   | 1536x864  | 120     | Centre crop, narrower field of view    |
| `hdr`    | 2304x1296 | 30      | On-sensor HDR                          |

Other outputs: `yuv out.yuv` (I420) and `rgb out.rgb` (RGB888) for raw video. For stills the
format follows the extension (`.jpg`, `.png`, `.bmp`, `.yuv`, `.rgb`) or `-e`. `--raw` adds a DNG.

Options:

```
--fps N                      frame rate (default 30)
--af continuous|auto|off     autofocus mode (default continuous; auto focuses once at the start)
--af-window X,Y,W,H[,...]    areas autofocus looks at, as fractions of the frame, up to 10
                             (default: the middle half of the width, middle third of the height)
--focus CODE                 manual lens position, 445 (infinity) to 925 (closest)
--flicker auto|50|60|off     mains flicker avoidance
--sat X                      saturation
--timeout MS                 stills: time to settle AE/AWB/AF before capture (default 3000)
--quality Q                  JPEG quality (default 93)
```

While it runs, these commands (one per line) on stdin work like libcamera's AF controls:

```
f                      auto mode, and scan once (AfModeAuto + AfTrigger)
c                      continuous mode (AfModeContinuous)
w X Y W H [X Y W H…]   autofocus windows, fractions of the frame (AfWindows); w alone: default
m CODE                 manual mode, lens at CODE (AfModeManual + LensPosition)
q                      quit
```

Each change of AF state is printed as a line such as `AF focused at 2.74 dioptres (lens 533)`
(idle, scanning, focused, failed or manual).

Autofocus is `rpi.af`'s contrast path: it scans the lens for the position where the windows are
sharpest and, in continuous mode, scans again when the scene in them changes and then holds
still. On the Pi the sensor's phase-detect pixels usually do this instead, but the Jetson can't
read them out. Like on the Pi, a plain subject in front of a detailed background loses to the
background unless a window is put on it: an application that wants faces in focus detects them
and sets the windows (as `imx708-depth` does). A new window during a scan starts the scan again;
otherwise, as on the Pi, it only changes what is measured.

`imx708-live` can also be used directly with any GStreamer sink pipeline, see
`isp/imx708-live --help`.

## How it works

The sensor is captured as RAW10 through V4L2 with the Argus ISP bypassed. Each frame is processed
on the GPU: black level, lens shading, white balance, demosaic, colour matrix and gamma. Output
buffers are handed to GStreamer without copying.

Statistics from each frame drive ports of these Raspberry Pi libcamera algorithms, using the
`imx708.json` tuning file:

- `rpi.agc`: centre-weighted auto exposure, the "normal" exposure mode and highlight constraint
- `rpi.awb`: Bayesian white balance along the calibrated colour temperature curve
- `rpi.alsc`: lens shading tables (the calibrated tables only, not the adaptive part)
- `rpi.ccm`, `rpi.contrast`: colour matrices by colour temperature and the gamma curve
- `rpi.af`: autofocus, the contrast-detect path (the Jetson can't capture the sensor's PDAF
  data), on a 64x48 grid of focus statistics

Mains flicker is detected from rolling-shutter banding, and exposure is then kept to whole flicker
periods. The focus motor (DW9817) is driven over I2C with the same small ramped steps as the Pi's
driver, which keeps it quiet.

Not ported: denoise and sharpening.

## Driver patches

Applied on top of RidgeRun's `7.2.1_orin_nano_imx708_v0.1.0.patch`:

1. Analogue gain now uses the IMX708 formula (code = 1024 - 1024/gain). Before, a requested 16x
   gain gave about 1.9x. Exposure is limited to frame length - 48 lines, and the full-resolution
   `line_length` is corrected so that mode runs at 14 fps instead of 8.6.
2. Adds the 2x2 binned (56 fps), 1536x864 crop (120 fps) and HDR (30 fps) modes, with register
   tables from the Raspberry Pi kernel driver.
3. Adds an overlay for a single camera on CAM1. The dual overlay fails to probe the empty port.

## Troubleshooting

- **`VIDIOC_S_FMT: Device or resource busy`**: another process has the camera open. Only one
  capture can run at a time.
- **`imx708-focus.sh` says the focus motor isn't responding**: the motor is only powered while the
  sensor is streaming. Start a capture first.
- **Pink or washed-out image**: you're probably using `nvarguscamerasrc`. Use `imx708-live`.

## Status

Tested on the hardware above: video in all four modes, every output (display excepted, the test
board was headless), stills in each format, DNG output, auto exposure, white balance, autofocus,
manual focus and flicker detection.

The HDR mode works but has only been tried in a dim room; its image is noisier than the binned
mode and has a slight magenta cast. The DNG files haven't been checked in a raw converter yet.

## License

The kernel patches, device-tree overlays and `build-driver.sh` are GPL-2.0, following the NVIDIA
and RidgeRun sources they modify. Everything else is BSD-2-Clause, following the Raspberry Pi
libcamera and rpicam-apps code it is ported from. See [LICENSES](LICENSES). Original copyright
notices are kept in each file.

## Credits

- [RidgeRun](https://github.com/RidgeRun/NVIDIA-Jetson-IMX708-RPIV3) wrote the original IMX708
  driver for Jetson that this builds on.
- Raspberry Pi's [libcamera](https://github.com/raspberrypi/libcamera) algorithms and tuning,
  [rpicam-apps](https://github.com/raspberrypi/rpicam-apps) and the
  [imx708 kernel driver](https://github.com/raspberrypi/linux/blob/rpi-6.12.y/drivers/media/i2c/imx708.c).
- NVIDIA's Jetson Linux camera drivers.

This project is not affiliated with Raspberry Pi Ltd, RidgeRun or NVIDIA.
