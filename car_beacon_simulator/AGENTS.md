# Explicit experiment execution

The user requires experiments to run only on their explicit instruction.

- Do not execute firmware experiments, call `POST /api/run`, use headless mode,
  or run the Python simulator integration suite unless the user explicitly asks
  for those experiments or checks. `tests/test_simulator.py` executes firmware.
- Requests to inspect a trace, explain behaviour, edit code, or push changes do
  not authorize running experiments as verification.
- Static inspection and syntax checks are allowed. Interface tests that use
  mocked data and never execute firmware are also allowed.
- In the application, only the explicit **Build & run experiment** action may
  request a new firmware execution. Loading a page, editing starting conditions,
  selecting a source or preset, loading setup/trace files, inspecting history,
  and playback controls must never start a firmware execution.
- The user requested restoration of the original build flow: an explicit
  **Build & run experiment** click executes a fresh experiment, then starts its
  playback automatically (original default speed: 20x). Imported and history
  recordings stay paused until **Play** or **Replay** is pressed.
- Preserve the selected firmware's actual behaviour and success/failure rules.
  Do not modify navigation or bypass a stop condition to obtain a passing run.
