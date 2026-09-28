# SPDX-License-Identifier: BSD-2-Clause
NVCC ?= /usr/local/cuda/bin/nvcc
PKGS := gstreamer-1.0 gstreamer-app-1.0 libtiff-4

imx708-live: imx708_live.cu
	$(NVCC) -O3 -std=c++17 -arch=sm_87 -diag-suppress 1217 -o $@ $< \
		$(shell pkg-config --cflags $(PKGS) | sed 's/-pthread/-Xcompiler -pthread/g') \
		$(shell pkg-config --libs $(PKGS) | sed 's/-pthread/-Xcompiler -pthread/g')

clean:
	rm -f imx708-live

.PHONY: clean
