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

Completed
1. (NOT STARTED) Live demo pending
2. (DONE)
3. (NOT STARTED) Need to get from team
4. (PARTIAL) Downlinked to ground station: entry interface, peak heating, peak deceleration, peak velocity, start of terminal descent (10 s to landing), landing. Not modeled: parachute deploy, separation
5. (DONE)
6. (DONE)
7. (DONE) Range checks on temperature, humidity, pressure (both missions) and gyro motion (Mars); limits in run.py fault_limits; tune temperature limit on packaged board
8. (DONE) fault / fault_cleared events in UDP telecom stream; active faults in every telemetry packet and ground station status line. Response is reporting only (no LED or flight action)
9. (DONE)
10. (AT RISK) Landing email replaced by UDP landing event to ground station laptop; may not count as text, email, or social media post
