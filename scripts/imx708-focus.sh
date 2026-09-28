#!/bin/bash
# SPDX-License-Identifier: BSD-2-Clause
# Set the Raspberry Pi Camera Module 3 (IMX708) lens position via its DW9817 VCM.
# The VCM is only powered while the camera is streaming, so run this during a capture.
# Usage: imx708-focus.sh <0-1023>   (~445 = infinity, ~850 = ~10 cm, 1023 = closest)
set -e
pos=${1:?usage: $0 <0-1023>}
((pos >= 0 && pos <= 1023)) || { echo "position must be 0-1023" >&2; exit 1; }

devs=(/sys/bus/i2c/drivers/imx708/*-001a)
dev=${devs[0]}
[ -e "$dev" ] || { echo "imx708 sensor not bound" >&2; exit 1; }
bus=$(basename "$dev" | cut -d- -f1)

sudo i2ctransfer -f -y "$bus" w2@0x0c 0x02 0x00 2>/dev/null || {      # VCM power on
	echo "focus motor not responding: start a capture first (it is only powered while streaming)" >&2
	exit 1
}
sudo i2ctransfer -f -y "$bus" w3@0x0c 0x03 $((pos >> 8)) $((pos & 255)) # lens position
echo "bus $bus: focus set to $pos"
