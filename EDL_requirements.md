# EDL requirements

The following requirements define the flight-computer demonstration.

| Category | Requirement | Description |
| --- | --- | --- |
| Raspberry Pi sensors and code | Flight computer description | Document the flight computer, all triggers, fault conditions, and responses. |
| Raspberry Pi sensors and code | Real-time phase calculations | Run the simulation calculations in real time on the Raspberry Pi or similar device for every mission phase. |
| Raspberry Pi sensors and code | Mission phase triggers | Choose an altitude, velocity, time, or other trigger for each phase. |
| Raspberry Pi sensors and code | Critical-event LEDs | Demonstrate hardware in the loop with LEDs that turn on and off to indicate critical events. |
| Raspberry Pi sensors and code | 10-second landing countdown | Demonstrate a pulsing LED during the final 10 seconds before landing. |
| Raspberry Pi sensors and code | Descent sensor measurement | Use a sensor to measure an atmospheric variable or capture an image during simulated descent. |
| Raspberry Pi sensors and code | Terminal-descent accelerometer data | Release the packaged lander or probe, let it land, and record accelerometer data during terminal descent. |
| System reliability and safety | Fault injection | Change the measured variable and demonstrate that the sensor reading triggers a fault. |
| System reliability and safety | Fault detection and response | Read the fault, execute the programmed response, and report the fault in the EDL telecom stream. |

## Current implementation coverage

This is a source-code review snapshot, not hardware acceptance-test evidence.

| Requirement | Current coverage | Work remaining |
| --- | --- | --- |
| Flight computer description | The README and module docstrings describe initialization, sensor polling, and the status display. | Document the complete mission, phase triggers, fault conditions, and responses as they are implemented. |
| Real-time phase calculations | `run.monitor()` polls sensors and updates elapsed time. Initial altitude, velocity, and flight-path angle are stored in `MissionState`. The offline generator calculates a trajectory. | Add live state estimation and phase-specific calculations; the offline generator is not called by the live loop. |
| Mission phase triggers | Successful readiness checks change `INITIALIZING` to `ENTRY`. These are the only defined phases. | Define the remaining phases and their triggers, validity checks, and transition actions. |
| Critical-event LEDs | The real/emulated board API supports LED control. The calibration sensor checker cycles colors. | Connect LED patterns to mission events in the flight loop. |
| 10-second landing countdown | No landing-time estimator or countdown controller exists. | Define the touchdown-time basis and pulse an LED without blocking acquisition during the final 10 seconds of simulated descent. |
| Descent sensor measurement | `SensorReader` reads pressure, both temperature channels, humidity, and raw IMU measurements from hardware or the emulator. | Demonstrate acquisition during simulated descent and retain timestamped evidence. |
| Terminal-descent accelerometer data | Raw acceleration can be read. `telemetry.DropLog` can write CSV rows, but has no callers. The README references an absent `record_drop.py`. | Implement and verify continuous drop recording, then perform the packaged-lander demonstration and inspect the captured acceleration. |
| Fault injection | Emulator targets can be changed through its API or GUI. | Connect the injected measurement to a defined runtime fault detector and demonstrate the resulting event. |
| Fault detection and response | Initialization rejects invalid readings. The live loop catches `OSError`/`RuntimeError`, retains the previous readings, and reports read failure to the status window when open. | Add runtime validity, freshness, and plausibility checks; execute defined responses and report fault and response events through the EDL telecom stream. |

## Integration points

- `src/edl_flight_computer/run.py`: orchestrate acquisition, validation, state estimation, derived quantities, phase transitions, LED updates, and telemetry on each scheduled iteration.
- `src/edl_flight_computer/sensors.py`: retain raw measurements and expose per-group acquisition times and validity so repeated or failed reads are not treated as fresh samples.
- `src/edl_flight_computer/state.py`: store estimated mission quantities, uncertainty, phase, and fault status separately from raw measurements.
- `src/edl_flight_computer/mission.py`: retain readiness checks and add mission transition rules or call a dedicated phase controller.
- `src/edl_flight_computer/telemetry.py`: connect local logging to the live loop and extend the schema for derived values, phase transitions, fault identifiers, and responses. Add a transport if the demonstration requires a remote telecom stream.
- `data/create_atmospheric_data.py`: contains reusable atmosphere and motion calculations. Extract shared model calculations into the installed flight-computer package before using them in both offline generation and live estimation.

The simulated entry and the physical drop need separate initial conditions and
validation evidence. A short physical drop cannot provide a literal 10-second
pre-landing countdown; demonstrate that requirement on the simulated mission
timeline and use the physical drop to demonstrate terminal-descent recording.
