# Repository Guidelines

## Project Structure & Module Organization
- Root entry points: `boot.py` configures the board on power-up, `main.py` orchestrates runtime behavior, and `mainbak.py` stores the last verified build.
- Use `tools.py` for all `CANMotorController` frame helpers instead of duplicating bus logic.
- `classtest.py` is the jogging integration harness; tune values here before promoting changes to runtime scripts.
- Update `pin.py` (fixture pin map and defaults) whenever the wiring shifts.
- Keep vendor drivers under `lib/` and mechanical references under `print/`; ignore slicer scratch files.

- `mpremote connect [port] run main.py` executes the firmware in place without clearing flash after incremental updates.
- `mpremote connect [port] run classtest.py` verifies jogging behavior and streams position/velocity telemetry.
- `mpremote connect [port] fs cp tools.py :tools.py` (or another targeted file) synchronizes edits faster than a full upload.
- VS Code + Pymakr: trigger “Upload Project” to mirror the tree from `pymakr.conf` and keep the serial port aligned with your rig.

## Coding Style & Naming Conventions
- MicroPython-friendly PEP 8: four-space indents, `lowercase_with_underscores` for functions/modules, `UpperCamelCase` for classes, `UPPER_SNAKE_CASE` for constants.
- Annotate CAN packet builders with emitted units (rad, rad/s, amps) and avoid hidden globals; pass state explicitly.
- Keep helpers in `tools.py` or `lib/` to minimize duplication and flash usage; skip dependencies that bloat MCU storage.

## Testing Guidelines
- No automated framework yet—validate motion changes by running `classtest.py` while watching for stable telemetry.
- Place exploratory diagnostics next to their targets as `*_test.py` and describe intent in a top comment for reruns.
- Capture serial logs from successful hardware passes and attach excerpts to PRs when behavior, limits, or CAN timing changes.

## Commit & Pull Request Guidelines
- Use concise, imperative subjects (e.g., `first init`, `tune jog gains`) and expand with hardware configuration notes plus observed outputs.
- PRs should spell out the tested hardware scenario, list the exact `mpremote` or Pymakr commands executed, link related issues, and call out CAN parameter adjustments so reviewers can reproduce safely.

## Security & Configuration Tips
- Keep `pymakr.conf` aligned with your actual board path and baud rate; stale ports cause silent upload failures.
- Prefer `mpremote connect [port] fs ls` before copying files to confirm space, and never leave credentials or Wi-Fi keys in the repository or on the device flash.
