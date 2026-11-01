---
name: embedded-architecture-audit
description: Audit this Pico W firmware's execution model, timing, concurrency, ownership, radio coordination, and safety boundaries. Use for architecture reviews, concurrency investigations, or safety-path assessments; not for ordinary feature implementation.
---

Inspect actual source and build configuration rather than inferring implementation from documentation.

1. Identify foreground loops, interrupts, CYW43/BTstack callbacks, blocking calls, and transitions between them.
2. Trace ownership and synchronization of shared state. Distinguish `volatile`, atomic access, interrupt masking, critical sections, and consistent snapshots.
3. Review hardware boundaries, timeout behavior, Wi-Fi/BLE handover, STOP/FAULT propagation, and all paths that can leave actuators active.
4. Separate confirmed defects from risks that require instrumentation or hardware evidence.
5. Report findings by severity with source locations, reproduction or reasoning, consequence, and the evidence required for closure.

Do not modify functional firmware unless the user explicitly authorizes a corrective patch. Never claim that source inspection proves physical timing.

