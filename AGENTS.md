# Repository Guidelines

## Project Structure & Module Organization
- Root MicroPython entry points: `boot.py` handles board boot, `main.py` is the runtime orchestrator placeholder, and `mainbak.py` keeps the last working snapshot.
- `tools.py` centralizes `CANMotorController` helpers for framing and parsing CAN traffic; import from here instead of duplicating bus logic.
- `classtest.py` demonstrates the jogging workflow for validation; treat it as the template for integration experiments.
- `lib/` is reserved for vendor drivers and shared modules you want bundled on the device.
- Mechanical assets live under `print/` (STEP/PDF). Leave temporary renders and slicer files out of version control.

## Build, Test, and Development Commands
- `mpremote connect [port] run main.py` executes the uploaded firmware without wiping storage.
- `mpremote connect [port] run classtest.py` exercises the jogging controller against hardware; monitor serial output for position/velocity feedback.
- `mpremote connect [port] fs cp tools.py :tools.py` syncs updates to the board; prefer targeted copies over full-project wipes.
- VS Code + Pymakr: use “Upload Project” to mirror the working tree defined by `pymakr.conf`.

## Coding Style & Naming Conventions
- Follow MicroPython’s subset of PEP 8: four-space indents, `lowercase_with_underscores` for functions, `UpperCamelCase` for classes, and `UPPER_SNAKE_CASE` for constants.
- Document expected units (rad, rad/s, amps) near CAN packet builders and avoid hidden state in helper functions.
- Keep shared helpers in `tools.py` or a `lib/` submodule; avoid heavyweight dependencies that bloat the firmware image.

## Testing Guidelines
- No automated harness yet; rely on `classtest.py` integration runs before pushing changes that touch motion control.
- Name new diagnostic scripts `*_test.py` and store them beside the module they exercise to simplify deployment.
- Capture serial logs from successful runs and attach excerpts to the PR when behavior changes.

## Commit & Pull Request Guidelines
- Use concise, imperative commit subjects (history example: `first init`); expand with rationale and hardware notes when needed.
- Each PR must describe the hardware scenario, reference related issues, and include the command sequence you ran plus observed responses.
- Provide updated CAN parameter tables or limits in the PR description when you alter them so reviewers can reproduce safely.
