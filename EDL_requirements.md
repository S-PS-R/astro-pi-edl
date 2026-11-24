1. Flight Computer Demo 80 pts (to be demonstrated real time during your presentation with Raspberry Pi or similar device)
2. Conduct all calculations with real time EDL flight computer for each mission phase
3. You must choose altitude, velocity, time, or other trigger for each phase 
4. Demonstrate EDL telecon critical event coverage by outputting a message for each major event (atmospheric entry, peak heating, peak deceleration, parachute deploy, any separation, start of terminal descent, landing)
5. Show “hardware” in loop of critical events with LED(s) that turn on and off to indicate a critical event and a 10 seconds to landing counter (pulsing LED)
6. Make a measurement of an atmospheric variable or image during the simulated descent (temp, pressure, speed, image) with a sensor (thermocouple, pressure sensors, camera, etc)
7. Demonstrate measurement of a change in the variable to trigger a fault condition (e.g. create an overtemp condition  water, induce motion, blow air from a fan)
8. Have logic to read in and respond to the fault condition and include in the EDL telecom stream
9. Collect accelerometer data during the terminal descent (you will need to release the lander/probe (Pi packaged somehow) and let it land, fly, bounce on the surface)
10. Send a message at landing in the form of a text, email, or social media post

Implementation and demo checklist

- [ ] 1. Present the live flight computer demo on the Raspberry Pi.
- [x] 2. Calculate each modeled Mars phase during the run. Use 1× playback for the real-time demo.
- [x] 3. Define phase triggers. The chute opens at Mach 1.8. The heat shield and TPS drop 20 simulated seconds later. The backshell and chute separate at 2,100 m after heat shield release, and the engines ignite immediately. The canopy equilibrium event uses equilibrium-speed and vertical-acceleration limits for telemetry. Vertical descent uses lateral speed and target alignment or the divert cutoff. Contact occurs at zero altitude.
- [x] 4. Print and downlink entry, peak heating, peak entry deceleration, chute deployment, heat shield jettison, near-terminal descent, backshell/chute separation, powered descent start with mass and airspeed, and touchdown. The landing warning is a separate event.
- [x] 5. Implement Sense HAT LED event markers and a pulsing estimated ten-second landing warning. Verify on the physical board during the live demo.
- [x] 6. Read and log Sense HAT temperature, pressure, and humidity during simulated descent. Show a live sensor reading on the board during the demo.
- [x] 7. Detect an induced out-of-range sensor reading that lasts at least 0.5 seconds. Demonstrate the stimulus on the packaged board. Gyro fault thresholds are optional and disabled by default.
- [x] 8. Report faults and clears over UDP and include active faults in telemetry. The response is reporting only.
- [x] 9. Record accelerometer data during terminal descent using the Earth drop recorder. Use the physical drop CSV as demo evidence.
- [ ] 10. Send a landing text, email, or social media post. The implemented UDP landing event reaches the ground station. It does not match the listed delivery channels.
