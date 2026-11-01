---
name: minimum-bit-audit
description: Audit application fields and packed representations for minimum logical width, signed encoding, overflow, precision, concurrency, and measured memory effects. Use for tribunal bit-width reviews or packed-data changes; not for cosmetic type shrinking.
---

For every relevant field, record source, valid range and precision, minimum logical bits, physical storage, sentinel values, encoding, ownership, concurrency protection, and overflow behavior.

Check struct padding, enum and counter terminal values, timestamp rollover, narrowing order, signed conversions, precision loss, shared snapshots, and wire-layout boundaries. Require explicit wire encoders and decoders; C bit-fields are not portable wire formats.

Run round-trip and boundary tests through `python scripts/qa/run.py quick`. Compare compiler map/size evidence before proposing storage savings. Do not claim RAM, flash, message-size, or performance improvements from estimates alone.

