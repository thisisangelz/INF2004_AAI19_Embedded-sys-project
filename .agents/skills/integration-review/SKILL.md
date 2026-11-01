---
name: integration-review
description: Review a firmware change across hardware, navigation, RF, coordinator, protocol, and telemetry boundaries. Use after a developer handoff or when a change spans modules; not for isolated local edits with no interface impact.
---

Verify the stated base and head commits, inspect the actual diff, and map every changed interface to its callers, state owners, timeouts, and tests.

Review sensor validity and staleness, navigation transitions, actuator safety, Wi-Fi/BLE handover, protocol compatibility, packed fields, reporting, and failure recovery. Run the relevant shared QA mode and inspect raw failures independently.

Report Critical, High, Medium, and Low findings with file/line, reproduction, observed evidence, expected behavior, consequence, correction, and closure test. State unverified claims explicitly. Do not silently modify the developer's functional branch.

