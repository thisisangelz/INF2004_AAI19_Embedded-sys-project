---
name: firmware-feature-development
description: Use when asked to implement one rover firmware feature or fix end to end (navigation step, BLE/Wi-Fi change, state logic), through to tests and a Codex handoff.
---
# Firmware feature development

Follow `CLAUDE.md`. One feature at a time, on a feature branch.

## Workflow
1. Inspect the touched modules and existing tests (`getting_there/tests/`); state affected interfaces and risks before editing.
2. Smallest change that fits current interfaces; preserve ownership rules and working behaviour.
3. Write/extend host tests first for new logic (normal, boundary, failure); mock hardware via stubs.
4. `scripts/dev/build.ps1 <target>`; `scripts/dev/host_tests.ps1`; `python scripts/qa/run.py quick`. Record exact results; if something can't run, say so.
5. Copy `docs/handoffs/TEMPLATE.md` to `docs/handoffs/YYYY-MM-DD_<branch>_<feature>.md` and fill every section.
6. Address Codex findings with focused fixes + tests, or a written rejection reason.

## Done when
Target builds, tests pass or gaps are stated, handoff written, nothing committed/flashed without approval.
