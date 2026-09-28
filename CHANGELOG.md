# Changelog

## 0.1.0

First release, for JetPack 7.2.1 (L4T R39.2.1) on the Orin Nano.

- Driver: fixed analogue gain, exposure limit and full-resolution line length; added binned,
  crop and HDR modes; added a CAM1-only overlay.
- `imx708-live`: CUDA ISP with ports of Raspberry Pi's AGC, AWB, ALSC, CCM, contrast and AF;
  flicker detection; video to any GStreamer pipeline; stills as JPEG/PNG/BMP/YUV/RGB and DNG.
