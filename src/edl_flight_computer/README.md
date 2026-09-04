# edl_flight_computer

| Module | Role |
|---|---|
| hat.py | Sense HAT handle; simulator if the board is missing |
| sensors.py | Temperature, IMU accel, barometer |
| gpio_io.py | LED-matrix event discretes; joystick input |
| sequence.py | EDL phase timeline |
| display.py | HDMI status + matrix wash |
| landing.py | Touchdown from IMU magnitude |
| comms.py | Touchdown UDP downlink |
| log.py | CSV / UDP telemetry |
| record.py | 50 Hz free-flight capture |
| plot.py | Telemetry PNG |
| faults.py | Safe mode |
| demo.py | Sequenced EDL demonstration |
