# Contributing

Bug reports and pull requests are welcome.

When reporting a problem, include:

- Jetson module and carrier board, and the output of `cat /etc/nv_tegra_release`
- camera variant (standard / wide / NoIR) and which connector it is on
- `dmesg | grep -i imx708`
- the exact command you ran and its full output

## Changes to the driver

The patches in `kernel/` are a series on top of RidgeRun's 7.2.1 patch. `scripts/build-driver.sh`
leaves a git repository in `build/src` with the series applied as commits. Make your change there,
commit it (or amend the patch it belongs to), and regenerate the series:

```sh
cd build/src
git format-patch --no-signature -o ../../kernel HEAD~3   # adjust the count to the series length
```

Keep one logical change per patch, with a commit message that explains why.

## Changes to the ISP

- Follow the style of the surrounding code (tabs, C++17, no new dependencies unless needed).
- If you change a control algorithm, say which libcamera code it follows, and keep the tuning
  values in `rpi_imx708_tuning.h` in sync with `imx708.json`.
- Say what you tested it on and how: mode, frame rate, lighting.

Run `shellcheck scripts/*.sh` before sending changes to the scripts.

By contributing you agree that your changes are released under the licence of the files you
modify (GPL-2.0 for kernel and device tree, BSD-2-Clause for the rest).
