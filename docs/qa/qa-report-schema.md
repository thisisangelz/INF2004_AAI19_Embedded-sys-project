# QA report format

The shared runner writes `artifacts/qa/report.md`, `report.json`, and `junit.xml`. Coverage files are added when Gcovr is available.

Every review finding should contain:

- Stable identifier and severity: Critical, High, Medium, or Low.
- Status and evidence class.
- Affected target, file, and line.
- Base/head commits when reviewing a handoff.
- Reproduction command or physical procedure.
- Actual observed evidence and expected behavior.
- Failure consequence and recommended correction.
- Test or measurement required for closure.

Mandatory checks determine the runner exit status. Advisory failures remain visible but do not fail the software gate until their baseline is explicitly accepted. Hardware-required checks never pass automatically.

