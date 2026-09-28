#!/bin/bash
# SPDX-License-Identifier: BSD-2-Clause
# Raspberry Pi Camera Module 3 (IMX708) on Jetson: GPU ISP with Raspberry Pi's AE/AWB/AF tuning.
#
# Video (default 30 fps):
#   imx708-live.sh [options] display               # on the Jetson's monitor
#   imx708-live.sh [options] udp <host> [port]     # H.264 MPEG-TS over UDP (default port 5000)
#   imx708-live.sh [options] file <out.mp4>        # H.264 MP4 recording (Ctrl-C to finish)
#   imx708-live.sh [options] yuv <out.yuv>         # raw I420 frames (rpicam-vid --codec yuv420)
#   imx708-live.sh [options] rgb <out.rgb>         # raw packed RGB888 frames
# Stills:
#   imx708-live.sh [options] still <out.jpg|.png|.bmp|.yuv|.rgb> [--raw] [-e ENC] [--timeout MS] [--quality Q]
#                                                  # --raw also writes <out>.dng
#
# Options: --mode full|binned|crop|hdr   4608x2592 (<=14 fps) | 2304x1296 (<=56) | 1536x864 (<=120) | HDR 2304x1296 (30)
#          --fps N (default 30), --af continuous|auto|off, --focus CODE (manual, 0-1023),
#          --flicker auto|50|60|off, --sat X
# While running, type: f = autofocus scan, c = continuous AF, m N = manual focus, q = quit
set -e
dir=$(cd "$(dirname "$0")" && pwd)
bin="$dir/../isp/imx708-live"

opts=()
mode=binned
while [[ $1 == --* ]]; do
	[[ $1 == --mode ]] && mode=$2
	opts+=("$1" "$2")
	shift 2
done

if [[ $1 == still ]]; then
	shift
	out=${1:?usage: $0 still <file> [options]}
	shift
	exec "$bin" "${opts[@]}" --still "$out" "$@"
fi

# The Orin Nano has no hardware video encoder: x264 in software, sized for the CPU budget.
case "$mode" in
crop | 1536) enc_size="width=1536,height=864" ;;
*) enc_size="width=1920,height=1080" ;;
esac
enc="nvvidconv ! video/x-raw,format=I420,$enc_size ! x264enc tune=zerolatency speed-preset=superfast bitrate=12000 key-int-max=30 threads=4"

case "$1" in
display)
	export DISPLAY=${DISPLAY:-:1}
	sink="nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! nv3dsink sync=false"
	;;
udp)
	host=${2:?usage: $0 udp <host> [port]}
	sink="$enc ! mpegtsmux ! udpsink host=$host port=${3:-5000} sync=false"
	;;
file)
	out=${2:?usage: $0 file <out.mp4>}
	sink="$enc ! h264parse ! mp4mux ! filesink location=$out"
	;;
yuv)
	out=${2:?usage: $0 yuv <out.yuv>}
	sink="videoconvert ! video/x-raw,format=I420 ! filesink location=$out"
	;;
rgb)
	out=${2:?usage: $0 rgb <out.rgb>}
	sink="videoconvert ! video/x-raw,format=RGB ! filesink location=$out"
	;;
*)
	sed -n '3,18p' "$0"
	exit 1
	;;
esac

exec "$bin" "${opts[@]}" "$sink"
