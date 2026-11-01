# GP21 motor PWM-off physical validation

Requirement: every motor-driver PWM input controlled by GP8, GP9, GP10, and GP11 reaches zero within 20 ms of the electrical GP21 STOP event.

This is a physical test. Host tests only verify the modeled polling and shutdown commands.

## Equipment and setup

- Assembled Pico W carrier and motor driver, secured with wheels lifted or motors disconnected.
- Four-channel logic analyser or oscilloscope for the PWM inputs. If fewer channels are available, repeat without changing firmware until all four pins are measured.
- A channel or digital marker on GP21 at the Pico pin.
- Probe grounds connected to the circuit ground.

Do not let CI or QA scripts flash or operate the device. Flashing and energizing the test setup require explicit human approval.

## Procedure

1. Record firmware commit, UF2 hash, board/carrier revision, supply voltage, instrument model, sample rate, and probe points.
2. Begin a controlled forward drive, turn, and a period containing IMU activity. Repeat each case separately.
3. Trigger on the falling electrical edge at GP21.
4. Measure from that edge to the last active PWM edge or non-zero duty on each motor input.
5. Repeat at least ten times per operating case, including repeated STOP presses and an injected sensor-failure case where safely possible.
6. Retain raw captures and record the worst observed latency. A pass requires every captured channel in every required case to be at or below 20 ms.

## Evidence record

| Field | Value |
|---|---|
| Commit / UF2 SHA-256 | |
| Hardware revision | |
| Instrument / sample rate | |
| Supply and load condition | |
| Operating case | |
| GP8 latency | |
| GP9 latency | |
| GP10 latency | |
| GP11 latency | |
| Worst latency | |
| Raw capture paths | |
| Observer / date | |
| Result | `HARDWARE_UNVERIFIED` until completed |

