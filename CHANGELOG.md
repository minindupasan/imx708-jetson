# Changelog

## Unreleased

- `imx708-live`: autofocus rewritten as a closer port of `rpi.af`: the same scan states, AF
  windows merged over a focus statistics grid, the Pi's default window, and the Pi's handling of
  mode changes and triggers. It prints each AF state change.
- `imx708-live`: `--af-window` and the `w` command set up to 10 autofocus windows. Like
  `AfWindows` on the Pi, a new window doesn't start a scan by itself (it restarts one in
  progress); in continuous mode the scene change it causes does.
- `imx708-live`: `f` switches to auto mode and scans once; `--af auto` scans at the start, as
  rpicam-apps does.
- `imx708-live`: manual lens positions are limited to the calibrated range, 445 to 925.
- `imx708-live`: commands sent together on stdin are all handled; all but the first used to
  wait for more input.
- `imx708-live`: no longer spins when stdin is closed, or reports Ctrl-C as a sensor timeout.

## 0.1.0 - 2026-09-28

First release, for JetPack 7.2.1 (L4T R39.2.1) on the Orin Nano.

- Driver: fixed analogue gain, exposure limit and full-resolution line length; added binned,
  crop and HDR modes; added a CAM1-only overlay.
- `imx708-live`: CUDA ISP with ports of Raspberry Pi's AGC, AWB, ALSC, CCM, contrast and AF;
  flicker detection; video to any GStreamer pipeline; stills as JPEG/PNG/BMP/YUV/RGB and DNG.
