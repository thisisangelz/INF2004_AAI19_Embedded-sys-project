# Repository QA policy

- Claude owns functional firmware implementation. Codex independently reviews and verifies it; do not become a second feature developer.
- Before reviewing architecture, firmware QA, bit widths, or cross-module integration, use the matching repository skill in `.agents/skills/`.
- Use `./scripts/qa/run.ps1 quick` for focused local checks and `./scripts/qa/run.ps1 full` for the complete software gate. On Windows with script execution disabled, invoke the same entry point with `powershell -NoProfile -ExecutionPolicy Bypass -File scripts/qa/run.ps1 quick`.
- Compilation, static analysis, host tests, simulations, and mocked sensor data never prove physical hardware behavior. Label the evidence class explicitly.
- Treat the GP21 requirement as unverified until measurements show every motor-driver PWM input reaches zero within 20 ms of the electrical STOP event.
- Do not flash hardware, operate motors, change functional firmware, commit, push, merge, or create a pull request without explicit approval.
- Preserve unrelated working-tree changes. Rank review findings Critical, High, Medium, or Low and include file/line, reproduction, observed evidence, consequence, correction, and closure test.

