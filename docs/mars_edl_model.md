# Mars Entry, Descent, and Landing Architecture and Entry Conditions

The simulation follows a guided capsule through atmospheric entry, parachute descent, heat shield jettison, backshell and chute separation, powered braking, vertical descent, and touchdown. The Raspberry Pi or desktop computes the trajectory during the run and publishes the state and critical events. It uses a three-dimensional point mass and a flat landing plane for the final segment. The landing target is 49.02° N, 188.15° E, with a 1 km acceptance radius. The current configuration is defined in `src/edl_flight_computer/run.py`; dynamics and geometry are implemented in `mars/model.py`.

## 1. Entry conditions and assumptions

| Input | Simulation value | Basis |
| --- | ---: | --- |
| Entry interface altitude | 120 km | Demo initial condition. Mars 2020 used an entry-interface radius of 3,522.2 km, conventionally described as 125 km above the reference radius |
| Entry speed and flight-path angle | 5,330 m/s, −15° | Inertial, measured from local horizontal |
| Initial latitude, longitude, and heading | 48.19735° N, 173.30° E, 76° clockwise from north | Entry longitude tuned for Mach 1.8 deployment and the nominal seed-40 wind |
| Wet mass at entry | 3,300 kg | Demo allocation near Mars 2020's reported 3,317.3 kg. Includes propellant, TPS and jettisoned hardware |
| Entry reference area, drag coefficient, and lift coefficient | 15.9043 m², 1.46, 0.36 | Constant coefficients inferred from Mars 2020 trim lift-to-drag ratio and ballistic coefficient |
| Aeroshell geometry | 4.50 m diameter, 70° half-angle, 1.125 m nose radius, 0.126 m shoulder radius | NASA Mars 2020 geometry drawing |
| Mars radius, surface gravity, rotation period | 3,389.5 km, 3.711 m/s², 24.6229 h | Spherical model with surface gravity from the workbook |
| Atmosphere | 214 K, 636 Pa at the surface, ideal CO₂ | Isothermal exponential atmosphere |
| Wind | Enabled with seed 40 | 2–10 m/s horizontal surface wind, gusts up to 3 m/s, a 20 km decay scale, and smooth interpolation between 20 s gust samples |

The state consists of Mars-fixed position $\mathbf r$, ground-relative velocity $\mathbf v$, and usable propellant mass $m_p$. On input, the inertial velocity is formed from the local horizontal heading, up vector, speed, and flight-path angle. The rotating-frame initial velocity is $\mathbf v_0=\mathbf v_{\rm inertial,0}-\boldsymbol\Omega\times\mathbf r_0$, where $\boldsymbol\Omega$ points along the Mars rotation axis. Mass at each point is the attached dry mass plus remaining propellant. Jettison drops mass and changes the aerodynamic configuration without adding an impulse.

### Aeroshell geometry and TPS mass

The projected disk $S=\pi(D/2)^2$ gives the **15.9043 m²** reference area for lift and drag. For TPS, the model sums the surface areas of a spherical nose cap, tangent conical frustum, and rounded shoulder swept to the widest point. For half-angle $\theta$, nose radius $R_n$, and shoulder radius $R_s$, the nose join is at $R_n\cos\theta$, the shoulder center is at $D/2-R_s$, and the cone joins the shoulder at $D/2-R_s+R_s\cos\theta$. The curved forebody area is **17.8335 m²**. With the demo's uniform 31.2 mm thickness and 256 kg/m³ density, the estimated TPS mass is **142.44 kg**. The 1.125 m nose curvature radius feeds the heating equation. The demo's fixed aerodynamic coefficients, prescribed entry mass, and 274.90 kg shield structure allocation are retained. The heavier TPS comes out of the remaining dry mass allocation at fixed entry mass. [NASA entry vehicle geometry guide](https://www.nasa.gov/wp-content/uploads/2023/08/final-planetary-mission-entry-vehicles-quick-reference-guide-v4.1-.pdf) · [Mars 2020 entry-interface definition](https://ntrs.nasa.gov/api/citations/20200002978/downloads/20200002978.pdf)

## 2. Atmospheric entry and aerodynamic forces

Over the spherical portion of the flight, $h=\lVert\mathbf r\rVert-R_M$, $\hat{\mathbf u}=\mathbf r/\lVert\mathbf r\rVert$, and $g(h)=\mu/(R_M+h)^2$. The model sets $\mu=g_{\rm surf}R_M^2$ to reproduce the workbook's surface gravity. The rotating air is stationary relative to Mars except for the prescribed wind $\mathbf w$. The force calculation uses $\mathbf v_a=\mathbf v-\mathbf w$, $V_a=\lVert\mathbf v_a\rVert$, and $\hat{\mathbf v}_a=\mathbf v_a/V_a$ when $V_a>0$.

$$
H=\frac{R_{\rm CO_2}T}{g_{\rm surf}},\qquad
p=p_0 e^{-h/H},\qquad
\rho=\frac{p}{R_{\rm CO_2}T},\qquad
q=\frac12\rho V_a^2.
$$

The specific gas constant is 188.9 J/(kg·K). Mach number is $V_a/\sqrt{\gamma R_{\rm CO_2}T}$, where $\gamma=c_p/(c_p-R_{\rm CO_2})$ and $c_p=517.7+1.1T$ in J/(kg·K). Convective heating telemetry uses the Sutton–Graves form $\dot q_{\rm heat}=C_{\rm SG}\sqrt{\rho/R_n}\,V_a^3$, with code coefficient $1.9027\times10^{-4}$ and output converted to kW/m². Heating and peak detection provide telemetry. The model omits ablation and heat shield temperature. [NASA CO₂ specific heat relation](https://ntrs.nasa.gov/api/citations/20190001449/downloads/20190001449.pdf#page=12)

The total drag includes the vehicle and any deployed canopy. Entry lift is perpendicular to $\mathbf v_a$. The bank angle rotates lift around the air-relative velocity vector.

$$
\begin{aligned}
\mathbf F_D &= -q(C_{D,v}S_v+C_{D,p}S_p)\hat{\mathbf v}_a,\\
\mathbf F_L &= qS_v C_L(\cos\sigma\,\hat{\mathbf l}_{\rm up}+
                                  \sin\sigma\,\hat{\mathbf l}_{\rm right}),\\
\dot{\mathbf r} &= \mathbf v,\\
m\dot{\mathbf v} &=
  -mg(h)\hat{\mathbf u}+\mathbf F_D+\mathbf F_L
  -2m\boldsymbol\Omega\times\mathbf v
  -m\boldsymbol\Omega\times(\boldsymbol\Omega\times\mathbf r).
\end{aligned}
$$

$S_p$ contributes only with the canopy deployed. The final two terms are the Coriolis and centrifugal terms in the Mars-fixed frame. Ground-relative speed and vertical speed are tracked separately from airspeed. The ground-relative vertical acceleration before separation includes the change in radial direction, $a_z=\dot{\mathbf v}\cdot\hat{\mathbf u}+V_h^2/\lVert\mathbf r\rVert$.

### How the banking controller works

Bank steers the existing lift force. At 0° all lift points in the lift-up direction, in the vertical plane of the velocity vector. At 60°, half of the lift magnitude remains in that direction and about 87% acts sideways. The radial part of the lift-up component and the curvature term both affect how quickly the flight path flattens. Bank is the roll of the lift vector around the air-relative velocity, while flight-path angle describes where the velocity points relative to local horizontal. The distinction matters when reading the FPA plot. See ASTE 577 Lecture 3 AFM Part 1, PDF pp. 18 and 71, and Lecture 4 Aerothermodynamics, PDF pp. 5 and 10–12.

1. **High-speed bank.** The demo uses a fixed-direction right bank scheduled by **ground-relative speed**. It commands 60° at or above 4,400 m/s, interpolates linearly to 45° at 2,200 m/s, and holds 45° down to 1,500 m/s. This reduces lift-up and limits the pull-up observed in the nearly lift-up entry case.
2. **Heading alignment.** Project the ground velocity and the direction to the waypoint onto the local tangent plane. The waypoint has the landing site's latitude and is 0.90° east of its longitude. With projected waypoint direction $\mathbf d$ and ground track $\mathbf t$, the signed right-positive heading error is $\psi_e=\operatorname{atan2}(\hat{\mathbf u}\cdot(\mathbf d\times\mathbf t),\mathbf d\cdot\mathbf t)$. Command five times that error in degrees, capped at ±30°. Positive bank steers right.
3. **Handoff.** Blend linearly from the scheduled bank at 1,500 m/s to the heading command at 1,100 m/s. Below 1,100 m/s, use heading alignment alone until chute deployment. The canopy phase has zero vehicle lift.

Perseverance used a speed-dependent bank profile with feedback on range and bank reversals. This demo uses a fixed right bank during the scheduled segment and adjusts the initial entry point to reach the target. It does not simulate a roll actuator, bank reversals, or range feedback during the scheduled segment. [NASA post-flight entry guidance report](https://ntrs.nasa.gov/citations/20240015538)

## 3. Descent architecture and event sequence

1. **Enter and guide.** Initialize the state at the entry interface. Evaluate density, wind, lift and drag at each integration step. Track peak heating and entry deceleration and announce them after a sustained drop from their running peaks.
2. **Deploy the parachute.** Open at Mach 1.8. The 17 m canopy contributes $S_p=\pi D^2/4\approx227.0$ m². Its Cd is interpolated from the NASA MSL DGB Mach table in `run.py`, clamped at the tabulated endpoints outside its range. Full drag is applied immediately and lift drops to zero. The deployment event estimates opening load as 1.4 times steady canopy drag at the crossing.
3. **Jettison the heat shield.** Twenty simulated seconds after deployment, release the 417.34 kg heat shield and TPS. The chute initially carries 3,300 kg and then 2,882.66 kg after shield release. Set the remaining vehicle to $C_D=1.1$ with 15 m² area. The near-terminal descent event fires when airspeed stays within 7% of local equilibrium and vertical acceleration stays within ±0.25 m/s² for one simulated second. The event reports canopy descent conditions.
4. **Separate the backshell and ignite.** Once the heat shield is gone, release the 206.47 kg backshell and chute at 2,100 m. The lander starts powered braking at separation with 2,676.18 kg wet mass. The canopy force is removed and the lander uses $C_D=1.0$ and 8 m².
5. **Brake and divert.** Apply upward thrust first and use the remaining thrust to reduce lateral velocity or steer toward the target. The target-based divert ends at or below 180 m. The controller continues damping horizontal drift.
6. **Capture vertical descent.** Transition once horizontal speed is at or below 0.1 m/s and the vehicle is within the target radius or below the divert cutoff. During powered braking, estimate time to touchdown as $2h/(-v_z+v_f)$, with $v_f$ the commanded touchdown speed. This assumes constant deceleration from the current downward speed to $v_f$. The status LED starts flashing at an estimated 10.5 s, including a 0.5 s allowance for display updates. A first estimate below 10 s suppresses the warning at that sample.
7. **Touch down.** Stop at zero height above the landing plane. The model classifies contact as a soft landing after vertical descent if the absolute vertical speed is at most 2.5 m/s and horizontal speed is at most 0.2 m/s. The commanded touchdown speed is 2.0 m/s. Report a target hit only for a soft landing within 1 km. Contact mechanics and bounce are omitted.

The integrator uses forward Euler, $\mathbf x_{k+1}=\mathbf x_k+\Delta t\,f(\mathbf x_k,t_k)$, with substeps at or below 0.02 simulated seconds. It locates event crossings inside a substep and recomputes forces on the new side of each transition. An in-step fuel exhaustion likewise splits the step and removes thrust for the rest. Playback speed controls simulation time per wall-clock second. The Sense HAT measurements are logged and checked for faults independently of the modeled dynamics. The console, UDP downlink, CSV, plots, and LEDs expose the flight state and events for the demo.

### Sense HAT LED matrix legend

The 8×8 matrix shows simulated time increasing from left to right and altitude decreasing from top to bottom. A dim blue-green trail shows the recorded trajectory. Event colors are defined in `mars/led_grid.py`:

| Color | Event or indication |
| --- | --- |
| Teal | Entry interface |
| Red | Peak entry deceleration (along-path acceleration) |
| Orange | Peak convective heating |
| Blue | Parachute deployment |
| Yellow | Near-terminal descent under canopy |
| Coral | Heat shield jettison |
| Cyan | Backshell and parachute separation |
| Pink | Powered descent starts |
| Flashing white | Current trajectory position, visible for half of each one-second cycle while airborne |
| Flashing green, bottom-right cell | Landing warning latched at an estimated 10–10.5 simulated seconds to touchdown during powered braking |
| Solid green, bottom-right cell | Soft landing; target acceptance is reported separately |
| Solid red, bottom-right cell | Impact, or deployment-window failure |

Event markers persist. When several events occupy the same cell, their colors cycle every 0.6 seconds. The flashing white position temporarily covers the underlying trail or event color, revealing it between flashes. The bottom-right cell is reserved for the landing warning and final outcome once those indications are active. Flash timing uses real time, while marker positions use simulation time. Peaks are marked at their observed locations after the peak detector confirms them. Telemetry reports powered descent from ignition through final braking.

### Deployment condition and nominal result

The demo deploys at the Mach 1.8 crossing, which occurs near 9.05 km in the nominal run. Mach also selects canopy Cd. The event records dynamic pressure for evaluating deployment conditions.

The nominal wind-seed-40 run opens at about 599 Pa, lands about 2 m from the site after 339 simulated seconds, reaches about 2.02 m/s downward speed, and retains about 92 kg of propellant. The entry longitude was adjusted to bring the lander within reach of the existing target at powered descent.

When the fixed-altitude setting is omitted, the model also supports a pressure window: arm after dynamic pressure exceeds the configured limit, then deploy on descent when it falls to or below the limit while above the minimum altitude. Missing that altitude window raises `DeploymentWindowMissed`; the simulation reports `deployment_failed` and stops. This option is inactive in the current run.

The opening-load estimate uses an MSL-style opening factor inspired by NASA's ASPIRE report. Flight dynamics use steady canopy drag, and the estimated load is recorded at deployment. [NASA ASPIRE parachute modeling report](https://ntrs.nasa.gov/api/citations/20200002925/downloads/20200002925.pdf)


## 4. Powered-descent force balance and guidance

At separation the simulation fixes an up vector $\hat{\mathbf u}_s$ from the Mars radius at the separation point. Height becomes $h=\mathbf r\cdot\hat{\mathbf u}_s-R_M$, where the tangent plane at $h=0$ stands in for terrain. Gravity acts downward and rotation terms are omitted. The simulator still uses altitude-dependent gravity and atmospheric drag.

### Equations of motion

The force balance and propellant update follow Lecture 9b's *Propulsive Landing* example (PDF pp. 21–23). The simulator integrates the vector form with forward-Euler steps of at most 0.02 s. In the vertical, zero-drag limit, taking downward speed as positive reduces the force equation to $m\dot V=mg-T_z$, as in the lecture. [ASTE 577 Lecture 9b, Propulsion Trajectories](https://drive.google.com/file/d/1K7y4tdJjaknbGzhOy36Su2j5LTrhr9dR/view?usp=drivesdk)

$$
\begin{aligned}
m\dot{\mathbf v} &= -mg(h)\hat{\mathbf u}_s+\mathbf F_D+
                    T_z\hat{\mathbf u}_s+\mathbf T_h,\\
m\dot v_z &= -mg(h)+F_{D,z}+T_z,\qquad
m\dot{\mathbf v}_h=\mathbf F_{D,h}+\mathbf T_h,\\
\dot m_p &= -\frac{\sqrt{T_z^2+\lVert\mathbf T_h\rVert^2}}{I_{sp}g_0},
\quad g_0=9.80665~\mathrm{m/s^2}.
\end{aligned}
$$

Mars gravity $g(h)$ enters the trajectory equations. Earth standard gravity $g_0$ converts the specified $I_{sp}$ into effective exhaust speed. The propellant step is $m_{p,i+1}=m_{p,i}-\lVert\mathbf T_i\rVert\Delta t/(I_{sp}g_0)$. Vehicle mass during powered descent is the attached dry mass plus the remaining propellant.

### Vertical braking and lateral guidance

The guidance commands separate vertical and lateral thrust components to brake within the altitude available and steer toward the landing target. In Lecture 9b's gravity turn (PDF pp. 8–11), thrust points opposite the velocity and gravity bends the path toward vertical. The demo ignites at the configured 2,100 m separation crossing and sets thrust direction from the vertical speed error and horizontal guidance command.

The vertical reference comes from constant-acceleration kinematics. For positive downward speeds and a remaining descent distance $h$, the net acceleration along the direction of motion is $-b$. Thus $v_f^2=V_{\rm ref}^2-2bh$. With the code's upward-positive convention, $v_{\rm ref}=-\sqrt{v_f^2+2b\max(0,h)}$, where $v_f=2$ m/s and $b=3.12$ m/s². Here $b$ is a nominal net braking acceleration. The controller adds a speed-error correction, $a_{\rm cmd}=-bv_z/|v_{\rm ref}|+(v_{\rm ref}-v_z)/\tau$, with $\tau=1$ s. During initial powered descent, the command is floored at zero. The controller subtracts gravity-plus-drag acceleration $a_{\rm free,z}$ to determine the upward thrust required.

$$
T_z=\min[T_{\max},\,m\max(0,a_{\rm cmd}-a_{\rm free,z})],\qquad
\lVert\mathbf T_h\rVert\le\sqrt{\max(0,T_{\max}^2-T_z^2)}.
$$

The lateral controller projects the offset to the target onto the landing plane and converts it to a desired horizontal velocity. It uses the velocity error to command horizontal acceleration. These flight-computer settings determine how the controller pursues the 1 km landing requirement.

| Setting | Nominal value | Purpose |
| --- | ---: | --- |
| `landing_target.response_time_s` | 4 s | Divide the target offset by this time. A 40 m offset requests 10 m/s toward the target. |
| `max_divert_speed_m_s` | 18 m/s | Cap desired horizontal speed while steering toward the target. |
| `stop_divert_altitude_m` | 180 m | At or below this altitude, command zero horizontal velocity and continue braking sideways motion. |
| `propulsion.response_time_s` | 1 s | Divide velocity error by this time to obtain an acceleration correction for both vertical and lateral guidance. |
| `horizontal_braking_m_s2` | 3 m/s² | Cap commanded horizontal acceleration before allocating the remaining thrust. |
| `horizontal_capture_m_s` | 0.1 m/s | Transition to final descent once speed is at or below this limit and the vehicle is inside the target radius or at or below the divert cutoff. |

Vertical braking gets first claim on the 22.075 kN thrust budget. The remaining capacity limits the lateral component. The model uses one equivalent thrust vector and assumes it can reorient instantly. [Powered descent controller](../src/edl_flight_computer/mars/powered_descent.py) · [Nominal flight settings](../src/edl_flight_computer/run.py)

## 5. Workbook sizing and mission heritage

The revised sizing sheet supplies the center-case canopy diameter and propulsion settings. Its sizing calculation assumes a fixed 3,436 kg suspended mass, vertical flight at the canopy-only equilibrium speed $V_c=\sqrt{2m_{\rm susp}g/(\rho C_{D,p}S_p)}$, and $\rho=0.015$ kg/m³. It computes $m_s=m_{\rm susp}-m_{\rm backshell}-m_{\rm chute}$, braking acceleration $b=(V_c^2-v_f^2)/(2h_s)$, and $\Delta v=(V_c-v_f)(1+g/b)$ using a **2,000 m braking distance**. It sizes propellant from the rocket equation and uses the mass at ignition for thrust:

$$
m_{p,0}=1.10\,m_s(1-e^{-\Delta v/(I_{sp}g_0)}),\qquad
T_{\rm ignition}=m_s(g+b).
$$

| Quantity | Workbook value | How the simulation uses it |
| --- | ---: | --- |
| Chute fabric mass | 13.6 kg | Jettisoned with the 192.87 kg backshell from the entry mass budget |
| Canopy equilibrium speed | 111.8 m/s | Sizing reference. The trajectory calculates speed from actual forces |
| Wet lander after separation | 3,230.4 kg | Sizing-sheet estimate. With the updated entry mass and shield release, the simulation reaches ignition at 2,676.18 kg |
| Ideal braking acceleration and total delta-v | 3.12 m/s², 240.2 m/s | Braking acceleration sets the reference profile. Delta-v is a sizing result |
| Usable propellant and specific impulse | 366.6 kg, 225 s | The propellant includes the workbook's 10% sizing addition. It is depleted at the commanded thrust rate |
| Total ignition thrust and eight-engine split | 22.075 kN total, 2.759 kN per engine | Total ignition thrust is the model's thrust cap. At the cap, modeled use is about 10.0 kg/s |
| Engine and tank estimates | 55.2 kg and 55.0 kg | Workbook mass allocations within the vehicle budget |
| Descent system subtotal | 490.36 kg | Chute, propellant, engines, and tanks |

Goal Seek selects the chute diameter where the finite-difference slope of the descent system mass is approximately zero. A larger canopy reduces the ideal descent speed and thrust demand, while increasing canopy mass. The workbook's 10% addition is applied to its calculated propellant requirement. The simulator reports the actual remaining propellant at touchdown. Its atmospheric density varies with altitude, and its force balance includes vehicle drag and lateral motion, so its burn differs from the workbook estimate.

The sizing sheet's 3,436 kg suspended mass and constant-density assumption differ from the current trajectory's 3,300 kg entry mass, heat shield release, and altitude-dependent density. The simulation retains the selected 366.6 kg propellant allocation and 22.075 kN thrust cap; it does not recalculate an optimum propulsion design during flight. Section 3 gives the current nominal trajectory result.

Curiosity and Perseverance informed the entry, parachute, heat shield release, and throttleable powered-descent sequence. Curiosity's published system uses eight descent engines and about 387 kg of usable hydrazine. Perseverance provides a reference for the parachute-to-powered-descent handoff. Those vehicles used a sky crane. This classroom architecture ends with the modeled powered lander on the surface. The workbook supplies this project's chute, propellant and ignition thrust. The earlier Mars Science Laboratory canopy curve is a reference for higher-fidelity parachute modeling. [Curiosity press kit](https://www.jpl.nasa.gov/news/press_kits/MSLLaunch.pdf) · [Perseverance landing overview](https://www.jpl.nasa.gov/news/press_kits/mars_2020/landing/mission/) · [NASA Mars Science Laboratory parachute model](https://ntrs.nasa.gov/citations/20130012762)

## 6. Demo limits to revisit

- **Chute deployment.** The demo uses a fixed Mach trigger and the project sizing sheet's 17 m canopy. Mars 2020 used a 21.5 m canopy and a guidance-based range trigger. [NASA Mars 2020 entry vehicle data](https://www.nasa.gov/wp-content/uploads/2023/08/final-planetary-mission-entry-vehicles-quick-reference-guide-v4.1-.pdf) · [Perseverance landing overview](https://www.jpl.nasa.gov/news/press_kits/mars_2020/landing/mission/)
- **Staging.** The 20-second shield timer, zero ignition delay, and instantaneous changes in drag and mass define the modeled staging sequence.
- **Landing.** The combined thrust vector can reorient instantly within the thrust cap. The 180 m divert cutoff and 1 km target radius are demo settings. The warning estimate uses the planned braking profile and is evaluated during powered descent.
- **Environment and aero.** The demo uses a 120 km start, a fixed atmospheric profile, a seeded wind profile, constant vehicle aerodynamic coefficients, and a flat landing plane. Canopy Cd varies with Mach. The target hit is verified for the nominal case.

## 7. Analytical sanity checks: ballistic and lifting entry

The capsule performs a low-lift guided entry: $L/D=C_L/C_D=0.36/1.46=0.2466$. The ballistic approximation is a useful zero-lift baseline, but it is not the only comparison. The supplied `Lifting Body Derivation.pdf`, pp. 1–2, describes **equilibrium glide**, with small flight-path angle and negligible flight-path-angle rate. Having nonzero lift does not by itself establish equilibrium glide. Our trajectory starts steeply, changes bank, and passes through a transient braking pulse.

The following checks use the current `run.py` settings and no wind. Entry airspeed is 5,174.9 m/s and air-relative FPA is −15.4606°, after converting the specified 5,330 m/s inertial velocity and −15° inertial FPA into the rotating frame. Using these air-relative inputs keeps the analytical drag/heating estimates consistent with the simulator. The comparison uses $\beta=m/(C_DS)=142.117$ kg/m², $H=10,893.2$ m, $\rho_0=0.015733$ kg/m³, and surface gravity $g_s=3.711$ m/s². The numerical runs were sampled every 0.1 simulated seconds, with the existing integration substeps at or below 0.02 s. The zero-lift comparison changes only $C_L$ to zero in memory; mission settings are unchanged.

| Quantity | Ballistic analytical approximation | Lifting equilibrium glide, bank 0° | Lifting equilibrium glide, fixed bank 60° | Numerical zero-lift entry | Current numerical lifting entry |
| --- | ---: | ---: | ---: | ---: | ---: |
| Peak deceleration magnitude / glide limiting value | 120.5 m/s² | 15.05 m/s², formal glide limit | 30.10 m/s², formal glide limit | 95.5 m/s² | 82.4 m/s² |
| Altitude at peak deceleration | 16.44 km | No finite peak in this approximation | No finite peak in this approximation | 19.39 km | 22.71 km |
| Speed at peak deceleration | 3,138.7 m/s | No finite peak in this approximation | No finite peak in this approximation | 3,209.5 m/s | 3,470.4 m/s |
| Peak convective heating | 513.4 kW/m² | 56.8 kW/m², formal glide estimate | 80.3 kW/m², formal glide estimate | 476.3 kW/m² | 465.2 kW/m² |
| Altitude at peak heating | 28.41 km | 49.32 km, formal glide estimate | 41.77 km, formal glide estimate | 31.12 km | 32.12 km |

Deceleration here means the magnitude of the negative along-path speed derivative, not the total aerodynamic acceleration sensed by an accelerometer. The glide columns approximate it by $D/m$, neglecting the small tangential gravity term. All acceleration values are in m/s² to avoid mixing Earth-g units with the simulator's local-gravity normalization. The fixed 60° column is a sensitivity check, not the variable-bank mission. **The glide values are not predictions or bounds for our transient trajectory.**

### Ballistic formulas

For $s=|\sin\gamma_e|$, constant FPA, zero lift, an exponential atmosphere, and drag dominating tangential gravity, the Allen–Eggers approximation gives:

$$
V(\rho)=V_e\exp\left(-\frac{\rho H}{2\beta s}\right),\qquad
\rho_{a,\max}=\frac{\beta s}{H},\qquad
h_{a,\max}=H\ln\frac{\rho_0}{\rho_{a,\max}},
$$

$$
a_{\max}=\frac{V_e^2s}{2eH},\qquad
V_{a,\max}=e^{-1/2}V_e.
$$

Here $V_{a,\max}$ is the **speed at maximum deceleration**. Combining this approximation with the simulation's Sutton–Graves heating law gives $\rho_{Q,\max}=\beta s/(3H)$ and $V_{Q,\max}=e^{-1/6}V_e$. These heating formulas are derived by maximizing $\sqrt{\rho}\,V(\rho)^3$. Source for the ballistic assumptions and deceleration result: Lecture 3 AFM Part 2, PDF pp. 74–75. The finite 120 km starting density is neglected in these analytical expressions.

### Lifting equilibrium-glide formulas and applicability

The supplied lifting derivation sets $\gamma\approx0$ and $\dot\gamma\approx0$. Extend its lift-up balance to a constant bank $\sigma$, define $\ell=(L/D)\cos\sigma$, and approximate radius and gravity as $R_M$ and $g_s$:

$$
\frac{L\cos\sigma}{m}=g_s-\frac{V^2}{R_M},\qquad
V_c=\sqrt{g_sR_M}=3,546.6\ \mathrm{m/s},
$$

$$
V^2(\rho)=\frac{g_sR_M}{1+\rho R_M\ell/(2\beta)},\qquad
\frac{D}{m}=\frac{g_s-V^2/R_M}{\ell}
\longrightarrow\frac{g_s}{\ell}\quad\text{as }V\to0.
$$

The last expression is the formal low-speed limit of the simplified glide model, not a finite hypersonic peak; the shallow-glide assumptions eventually fail at low speed. It is 15.05 m/s² for lift-up and 30.10 m/s² for fixed 60° bank. This follows the supplied derivation's limiting deceleration result, with bank included through $\ell$.

For positive lift-up, this equilibrium requires $V<V_c$. Our entry airspeed is about 1.46 times $V_c$, and the numerical peak heating occurs at 4,486.9 m/s, also above $V_c$. Substituting those speeds into the equilibrium relation would require negative density. The entry and heating pulse therefore cannot belong to that positive-lift equilibrium-glide family. Even at peak deceleration the trajectory still has FPA about −8.52° and changing flight direction.

For completeness, maximizing the same Sutton–Graves law along the ideal constant-bank glide relation gives:

$$
V_{Q,\max}=\sqrt{\frac23}\,V_c,\qquad
\rho_{Q,\max}=\frac{\beta}{\ell R_M},\qquad
h_{Q,\max}=H\ln\frac{\rho_0}{\rho_{Q,\max}}.
$$

The glide heating estimates in the table are calculated from these equations; they are an extension of the equilibrium relation using our heating law, not a separate quoted lecture formula. Their much lower heating values describe an ideal shallow glide after entry capture, which this run does not achieve during its heating pulse. The equilibrium assumptions are also discussed in [NASA CR-1000, Volume XVI, Section 2.6.2.2](https://archive.psas.pdx.edu/RocketScience/NASA_CR_GUIDANCE_FLIGHT_MECHANICS_AND_TRAJECTORY_OPTIMIZATION_Volume16.pdf#page=117).

The zero-lift numerical speed at peak deceleration is within 2.3% of the ballistic estimate. Its deceleration differs because gravity, curvature, and changing FPA remain active. Adding lift moves peak braking higher and lowers the peak load and heating relative to that zero-lift run. These checks support the direction and scale of the simulation's response; the numerical lifting trajectory remains the applicable result for the configured mission.

### Current simulation values beside the analytical checks

| Check | Analytical result | Current simulation result |
| --- | --- | --- |
| Ballistic peak braking, $V_e^2|\sin\gamma_e|/(2eH)$ | 120.5 m/s² at 16.44 km | 82.4 m/s² at 22.71 km, T+87.7 s |
| Ballistic speed at peak braking, $V_e/\sqrt e$ | 3,138.7 m/s | 3,470.4 m/s at the simulated peak braking point |
| Equilibrium-glide limiting braking, $g_s/\ell$ | 15.05 m/s² at 0° bank; 30.10 m/s² at fixed 60° | 82.4 m/s²; actual bank about 53.7° at peak braking, outside equilibrium glide |
| Ballistic peak heating from Sutton–Graves | 513.4 kW/m² at 28.41 km | 465.2 kW/m² at 32.12 km, T+73.9 s |
| Equilibrium-glide peak heating from Sutton–Graves | 56.8 kW/m² at 0° bank; 80.3 kW/m² at fixed 60° | 465.2 kW/m²; the transient heating pulse is outside equilibrium glide |

## 8. Trajectory comparison with Mars flight heritage

A run with the default wind seed gives the values below. NASA's quick-reference guide compiles mission data, and the landing press kit gives approximate preflight milestones.

| Quantity or milestone | Current simulation | Perseverance reference | Assessment |
| --- | --- | --- | --- |
| Entry mass / ballistic coefficient | 3,300 kg / 142.1 kg/m² | 3,317.3 kg / 143.2 kg/m² [1] | Very similar aerodynamic mass loading |
| Lift-to-drag ratio | 0.2466 | 0.248 [1] | Same low-lift capsule regime |
| Air-relative entry speed / FPA | 5.175 km/s / −15.46° | 5.33 km/s / −16.18° [1] | Similar scale; our 5.33 km/s input is inertial, so it is not an exact match to NASA's relative speed |
| Peak heating | 465 kW/m² (46.5 W/cm²), T+73.9 s | 684 kW/m² (68.4 W/cm²) [1]; about T+75 s in the press kit [2] | Timing agrees in scale; our simple heating estimate is about 32% lower |
| Peak aerodynamic acceleration | 85.4 m/s², or 8.71 Earth g | 10.73 g [1] | Same order, about 19% lower; this comparison uses the aerodynamic force magnitude rather than the code's local-gravity `g_load` channel |
| Chute deployment | T+235.2 s, 9.05 km, 418 m/s, Mach 1.80, 599 Pa | About T+240 s, 11 km and 420 m/s [2]; Mach 1.81 and 519.6 Pa [1] | Similar speed and timing, about 2 km lower |
| Chute diameter | 17 m | 21.5 m [1] | Our smaller canopy and shorter chute segment help explain the different descent timeline |
| Heat shield release | 20 s after chute opening, at 6.98 km and 175 m/s | About 20 s after opening at 7–8 km [2] | Delay and altitude agree at the precision of the milestone |
| Backshell separation / start of powered descent | T+303.2 s, 2.10 km, 107 m/s airspeed and 103 m/s downward | About 2.1 km and 85 m/s at powered descent [2] | Similar altitude with higher simulated speed |
| Chute deployment to backshell separation | 68 s | About 110 s in the preflight profile [2] | About 42 s shorter under canopy |
| Backshell separation to touchdown | 36 s | About 60 s in the preflight profile [2] | About 24 s shorter during powered descent |
| Entry-to-touchdown duration | 339 s | About 410 s in the preflight profile [2], 419 s in flight [3] | About 71 s shorter than the illustrated timeline |
| Final landing | 2.02 m/s downward, under 0.01 m/s horizontal, and about 2 m from target | Sky-crane architecture [2] | The powered lander meets the demo's touchdown speed limits and 1 km target radius |

[1] [NASA Planetary Mission Entry Vehicles Quick Reference Guide v4.1, Mars 2020, PDF p. 41 / printed p. 34](https://www.nasa.gov/wp-content/uploads/2023/08/final-planetary-mission-entry-vehicles-quick-reference-guide-v4.1-.pdf#page=41). The same guide's MSL entry lists $L/D=0.24$ and a ballistic coefficient around 146 kg/m², supporting the broader comparison to Curiosity-class entry capsules.

[2] [NASA/JPL Perseverance landing press kit, mission overview](https://www.jpl.nasa.gov/news/press_kits/mars_2020/landing/mission/). Altitudes from different missions use different surface/reference definitions; our ground is a sphere followed by a tangent landing plane, so this is a kilometer-scale comparison rather than an exact altitude alignment.

[3] [NASA/JPL Perseverance trajectory reconstruction](https://ntrs.nasa.gov/citations/20230006978). The 419 s interval comes from the reported spacecraft event times for atmospheric entry and touchdown.

The simulated trajectory remains descending throughout entry: sampled air-relative FPA stays between approximately −25.68° and −0.45°, with no sampled altitude reversal. It briefly flattens under lift before descending more steeply toward chute deployment. Heating peaks before braking, followed by a long decelerating entry segment, supersonic canopy deployment, subsonic backshell separation, and powered braking. Those features are consistent with the broad guided Mars-entry sequence. NASA's post-flight report likewise describes bank-guided low-lift entry followed by heading alignment, while the actual guidance includes range feedback and bank reversals that this demo omits. [Mars 2020 post-flight entry guidance report](https://ntrs.nasa.gov/citations/20240015538)

The demo deploys its smaller chute lower, descends faster under canopy, and reaches the 2.1 km separation altitude sooner. After ignition it brakes directly to 2.0 m/s near contact. Perseverance reached about 0.75 m/s at roughly 20 m and used its sky crane to lower the rover. [NASA/JPL powered-descent sequence](https://www.jpl.nasa.gov/images/pia24318-powered-descent-for-perseverance-illustration/)
