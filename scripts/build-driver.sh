#!/bin/bash
# SPDX-License-Identifier: GPL-2.0
#
# Build the IMX708 kernel module and device-tree overlays for JetPack 7.2.1 (L4T R39.2.1),
# natively on the Jetson.
#
#   scripts/build-driver.sh [--install] [path/to/public_sources.tbz2]
#
# Steps: NVIDIA R39.2.1 OOT sources -> RidgeRun's 7.2.1 IMX708 patch -> this repo's patch
#        -> nv_imx708.ko + tegra234-p3768-camera-rpicam3-imx708{,-C}.dtbo in build/out/
# --install copies them to /lib/modules and /boot (needs sudo).
set -e

L4T_SOURCES_URL=https://developer.nvidia.com/downloads/embedded/l4t/r39_release_v2.1/sources/public_sources.tbz2
RIDGERUN_REPO=https://github.com/RidgeRun/NVIDIA-Jetson-IMX708-RPIV3.git
RIDGERUN_COMMIT=292b5c1	# known-good revision of the RidgeRun repo
RIDGERUN_PATCH=patches_orin_nano/patches/7.2.1_orin_nano_imx708_v0.1.0.patch

repo=$(cd "$(dirname "$0")/.." && pwd)
install=0
[[ $1 == --install ]] && { install=1; shift; }
tarball=$1

build=$repo/build
src=$build/src
out=$build/out
kh=/lib/modules/$(uname -r)/build
mkdir -p "$build" "$out"

[[ -d $kh ]] || { echo "kernel headers missing: $kh (install nvidia-l4t-kernel-headers)" >&2; exit 1; }

if [[ ! -d $src ]]; then
	if [[ -z $tarball ]]; then
		tarball=$build/public_sources.tbz2
		[[ -f $tarball ]] || curl -fL -o "$tarball" "$L4T_SOURCES_URL"
	fi
	echo "== extracting NVIDIA OOT sources"
	tar xjf "$tarball" -C "$build" Linux_for_Tegra/source/kernel_oot_modules_src.tbz2
	mkdir -p "$src"
	tar xjf "$build/Linux_for_Tegra/source/kernel_oot_modules_src.tbz2" -C "$src"

	echo "== applying RidgeRun IMX708 patch + this repo's patch"
	[[ -d $build/ridgerun ]] || git clone -q "$RIDGERUN_REPO" "$build/ridgerun"
	git -C "$build/ridgerun" checkout -q "$RIDGERUN_COMMIT"
	git -C "$src" init -q
	# The RidgeRun patch also touches the in-tree kernel (kernel/), which is not needed for OOT modules.
	git -C "$src" apply --include='nvidia-oot/*' --include='hardware/*' "$build/ridgerun/$RIDGERUN_PATCH"
	# Commit the baseline and apply the series with git am, so build/src can be used to work on
	# the patches (edit, commit, git format-patch).
	g=(git -C "$src" -c user.name=imx708-jetson -c user.email=imx708-jetson@localhost)
	"${g[@]}" add -A
	"${g[@]}" commit -q -m "NVIDIA R39.2.1 + RidgeRun IMX708 7.2.1"
	"${g[@]}" am -q "$repo"/kernel/*.patch
fi

echo "== building modules (takes a while)"
conftest=$src/out/nvidia-conftest
mkdir -p "$conftest/nvidia"
cp -a "$src"/nvidia-oot/scripts/conftest/* "$conftest/nvidia/"
make -j"$(nproc)" ARCH=arm64 src="$conftest/nvidia" obj="$conftest/nvidia" CC=gcc LD=ld \
	NV_KERNEL_SOURCES="$kh" NV_KERNEL_OUTPUT="$kh" -f "$conftest/nvidia/Makefile"
make -j"$(nproc)" ARCH=arm64 -C "$kh" M="$src/hwpm/drivers/tegra/hwpm" CONFIG_TEGRA_OOT_MODULE=m \
	srctree.hwpm="$src/hwpm" srctree.nvconftest="$conftest" modules
make -j"$(nproc)" ARCH=arm64 -C "$kh" M="$src/nvidia-oot" CONFIG_TEGRA_OOT_MODULE=m \
	srctree.nvidia-oot="$src/nvidia-oot" srctree.hwpm="$src/hwpm" srctree.nvconftest="$conftest" \
	kernel_name=noble system_type=l4t \
	KBUILD_EXTRA_SYMBOLS="$src/hwpm/drivers/tegra/hwpm/Module.symvers" modules
cp "$src/nvidia-oot/drivers/media/i2c/nv_imx708.ko" "$out/"

echo "== building device-tree overlays"
hw=$src/hardware/nvidia/t23x/nv-public
inc=()
for d in "$hw"/include/*; do inc+=(-I"$d"); done
for n in imx708 imx708-C; do
	f=tegra234-p3768-camera-rpicam3-$n
	cpp -nostdinc -undef -D__DTS__ -x assembler-with-cpp "${inc[@]}" -I"$kh/include" \
		-I"$kh/scripts/dtc/include-prefixes" "$hw/overlay/$f.dts" -o "$build/$f.pre"
	dtc -@ -q -I dts -O dtb -o "$out/$f.dtbo" "$build/$f.pre"
done
ls -l "$out"

if ((install)); then
	echo "== installing"
	sudo install -D -m 644 "$out/nv_imx708.ko" "/lib/modules/$(uname -r)/updates/drivers/media/i2c/nv_imx708.ko"
	sudo depmod -a
	sudo install -m 644 "$out"/*.dtbo /boot/
	echo "Now enable the overlay and reboot:"
	echo "  sudo /opt/nvidia/jetson-io/config-by-hardware.py -n 2=\"Camera IMX708-C\"   # camera on CAM1"
	echo "  sudo /opt/nvidia/jetson-io/config-by-hardware.py -n 2=\"Camera IMX708\"     # cameras on CAM0 + CAM1"
fi
