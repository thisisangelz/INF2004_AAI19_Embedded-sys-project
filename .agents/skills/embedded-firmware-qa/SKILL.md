---
name: embedded-firmware-qa
description: Run and interpret this repository's firmware builds, host tests, static analysis, coverage, and regression checks. Use for QA baselines, change verification, and release-readiness checks; not for flashing or physical validation.
---

Run `./scripts/qa/run.ps1 quick` for focused checks or `./scripts/qa/run.ps1 full` for the complete software gate. Preserve generated reports under `artifacts/qa/`.

- Establish the repository and tool versions in the report.
- Keep compiler/build/test failures mandatory. Treat Clang-Tidy, Cppcheck, and coverage as advisory until their baselines are accepted.
- Exclude Pico SDK, BTstack, generated files, and test stubs from project-owned static-analysis findings unless an integration defect involves them.
- Classify evidence as `SOURCE_CONFIRMED`, `HOST_VERIFIED`, `FIRMWARE_COMPILE_VERIFIED`, `SIMULATOR_VERIFIED`, `HARDWARE_UNVERIFIED`, or `HARDWARE_MEASURED`.
- Investigate failures from raw output; do not accept another agent's summary as evidence.

Do not flash devices, operate motors, auto-format, or rewrite production code.

