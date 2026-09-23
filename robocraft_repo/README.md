# RoboCraft: Reconfigurable Planar Parallel Manipulator 🤖🔧

A mechatronics graduation project investigating a **multi-degree-of-freedom redundant, reconfigurable planar manipulator** built from three detachable 2-DOF serial manipulators.

The original project combines mechanical design, kinematic and dynamic analysis, trajectory generation, embedded control, Raspberry Pi ↔ ATmega communication, image processing, obstacle avoidance, and task-level motion such as platform rotation and square drawing.

> **Project status:** academic/graduation-project code reconstructed from the supplied thesis and code appendix. The repository is organized for readability and reuse, but the legacy code has **not** been independently validated or made hardware-safe.

## What the project does

The thesis describes a system with:

- **Three 2-DOF serial manipulators**
- **6 motorized joints** for a **3-DOF task-space platform**
- Reconfigurable operation as individual serial manipulators or as a synchronized parallel mechanism
- A compactness constraint based on an **800 mm radius** footprint
- A target dexterous workspace reported as **383,438 mm²**
- Link lengths reported as **220 mm + 220 mm**
- A **60 mm hexagonal platform**
- A reported workspace performance measure of **PC1 ≈ 0.95**
- A minimum payload requirement of **0.200 kg**
- A maximum allowable deflection of **2.5 mm**
- DC motors with potentiometer-based position feedback
- H-bridge motor driving
- Raspberry Pi and ATmega microcontroller communication over **I²C**
- Trajectory generation using via points and polynomial interpolation
- OpenCV-based green-obstacle detection
- A mechanical locking concept for the platform/gripper
- Control discussion based on **Ziegler–Nichols tuning**

These values and descriptions are taken from the supplied thesis; they should be treated as project-specific design data rather than universal specifications.

## Repository structure

```text
RoboCraft/
├── README.md
├── requirements.txt
├── docs/
│   └── CODE_ORIGIN.md
└── src/
    ├── trajectory/
    │   └── trajectory_generation.py
    ├── communication/
    │   ├── master_raspberry_pi.py
    │   └── slave_atmega.ino
    └── tasks/
        ├── pure_rotation.py
        └── drawing_square.py
```

### Source modules

| File | Role |
|---|---|
| `src/trajectory/trajectory_generation.py` | Via-point generation, inverse kinematics and polynomial trajectory generation for manipulator motion |
| `src/communication/master_raspberry_pi.py` | Raspberry Pi side of the control/communication workflow |
| `src/communication/slave_atmega.ino` | ATmega/Arduino-side I²C reception, potentiometer feedback and motor control |
| `src/tasks/pure_rotation.py` | Platform pure-rotation / attachment-detachment task |
| `src/tasks/drawing_square.py` | Coordinated second/third manipulator motion for square drawing |

## System architecture

```text
                    ┌──────────────────────┐
                    │     Task / Target    │
                    └──────────┬───────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │ Trajectory Generation│
                    │  + Inverse Kinematics│
                    └──────────┬───────────┘
                               │
                               ▼
                    ┌──────────────────────┐
                    │   Raspberry Pi       │
                    │   Master Controller  │
                    └──────────┬───────────┘
                               │ I²C
              ┌────────────────┼────────────────┐
              ▼                ▼                ▼
        ┌──────────┐     ┌──────────┐     ┌──────────┐
        │ ATmega / │     │ ATmega / │     │ ATmega / │
        │ Slave 1  │     │ Slave 2  │     │ Slave 3  │
        └────┬─────┘     └────┬─────┘     └────┬─────┘
             │                │                │
          Motors +          Motors +        Motors +
       potentiometers    potentiometers   potentiometers
             └────────────────┼────────────────┘
                              ▼
                  Reconfigurable robot mechanism
```

The exact electrical topology and addressing should be checked against the project's hardware build before deployment.

## Kinematics and trajectory generation

The thesis develops:

- Denavit–Hartenberg parameterization
- Forward kinematics
- Jacobian-based velocity analysis
- Singular-configuration discussion
- Inverse kinematics
- Dynamic/torque analysis
- Trajectory generation

The supplied trajectory code calculates an initial end-effector position, constructs a via point from geometric constraints, computes inverse-kinematic joint angles, and generates polynomial segments for the actuators.

The appendix uses project-specific constants such as `a2 = 230` and `b2 = 230` in several code paths. These should not be confused with the thesis's reported design values unless the corresponding implementation stage is being reproduced.

## Main control tasks

### 1. Trajectory generation

The trajectory-generation code is intended to make the path configurable through end-effector positions and orientations. It computes via points and joint trajectories for the manipulator.

### 2. Communication

The Raspberry Pi master uses I²C to exchange trajectory/feedback data with an ATmega slave. The slave reads potentiometers and drives motor outputs according to the received references.

### 3. Pure rotation

The pure-rotation task is associated with attaching/detaching and reconfiguring the platform. The appendix uses servo outputs and geometric calculations for the hexagonal platform.

### 4. Drawing a square

The square task coordinates the second and third manipulators and moves through successive square segments. The thesis discusses this as an example task and notes possible applications such as laser cutting.

### 5. Image processing / obstacle avoidance

The thesis also documents an OpenCV pipeline that:

1. Captures frames from an external camera.
2. Converts BGR/RGB imagery to HSV.
3. Thresholds the green obstacle.
4. Applies masking and median filtering.
5. Detects image features/edges.
6. Uses the detected obstacle information to support path generation.

The image-processing implementation is documented in the thesis; the supplied code appendix is primarily focused on the trajectory, communication, rotation, and square-drawing programs.

## Hardware and software

### Hardware described by the thesis

- Three planar 2-DOF manipulator units
- DC motors
- Potentiometers for position feedback
- H-bridge motor drivers
- ATmega microcontrollers
- Raspberry Pi
- Camera
- Grippers and a mechanical platform-locking mechanism
- Bearings, belts, shafts/gears and fabricated links

### Software/tools described by the thesis

- Python
- NumPy
- SymPy
- Matplotlib
- PySerial
- SMBus / I²C
- RPi.GPIO
- OpenCV
- SolidWorks
- Mathematica
- Arduino/ATmega development environment

Install the Python dependencies with:

```bash
pip install -r requirements.txt
```

Hardware-specific packages such as `RPi.GPIO` and `smbus2` are intended for Raspberry Pi environments.

## Important: legacy-code warning

The source in this repository was reconstructed from a PDF appendix. It should therefore be regarded as **research/project source material**, not production firmware.

Before connecting motors:

- verify every GPIO and I²C address;
- verify motor direction and H-bridge wiring;
- verify potentiometer calibration and reference offsets;
- verify angle units and scaling;
- test with motors mechanically unloaded;
- add current/position/limit protections appropriate to the hardware;
- check all trajectory limits;
- confirm the code against the actual PCB and wiring;
- never run the recovered code on a physical robot without a controlled test procedure.

The appendix contains hard-coded calibration values and hardware addresses. These are project-specific and should be moved into configuration when the code is modernized.

## Suggested next cleanup

For a maintainable research repository, the next iteration should separate:

```text
src/
├── kinematics/
├── trajectory/
├── control/
├── communication/
├── vision/
├── tasks/
└── hardware/
```

and move constants such as link lengths, joint offsets, I²C addresses, GPIO pins, PWM limits and camera settings into a configuration module.

Unit tests should then cover forward/inverse kinematics, trajectory continuity, coordinate transforms and communication packet encoding before hardware testing.

## Figures recommended for the GitHub README

Do **not** put all thesis figures into the README. A repository README works better with a small visual story.

### Recommended hero image — include this first

**Final assembled robot / complete mechanism**

Use the clearest photograph or SolidWorks assembly showing all three manipulators, the central platform, and the overall geometry.

Suggested caption:

> **RoboCraft — reconfigurable three-arm planar manipulator**

### Figure 2 — system concept

Use the thesis figure that best shows the **three manipulators arranged around the platform**.

Caption:

> **Three detachable 2-DOF serial manipulators operating as a synchronized parallel mechanism.**

### Figure 3 — workspace

Use the **dexterous workspace** figure (thesis Figure 2.1.3 / related workspace figures).

Caption:

> **Dexterous workspace used to evaluate the reachable operating region of the manipulator.**

This is especially valuable because the thesis reports a dexterous area of **383,438 mm²** and a PC1 value of approximately **0.95**.

### Figure 4 — mechanical design / CAD

Use a clean CAD view showing:

- link geometry,
- motor mounting,
- bearings/shafts,
- gripper,
- platform,
- or the complete assembly.

Prefer one uncluttered CAD render over a collage of many manufacturing screenshots.

### Figure 5 — electronics

Use **one** clear electronics/driver-board figure showing the motor driver or control architecture.

Avoid putting every Proteus/PCB manufacturing screenshot in the README. Those belong in `docs/`.

### Figure 6 — trajectory/path result

Use the figure showing the generated trajectory or the square-drawing task.

A before/after pair is ideal:

```text
Start position → generated path → final position
```

### Figure 7 — obstacle avoidance / vision

Use the strongest image-processing figure showing the green obstacle being segmented or the resulting path around it.

This communicates one of the project's distinctive control features much better than a code screenshot.

### Optional Figure 8 — platform locking / gripper

Include this if the mechanical locking mechanism is an important part of the repository's story.

## Recommended README image order

```text
1. Final robot photograph / assembly render
2. System architecture or three-arm configuration
3. Dexterous workspace
4. CAD/mechanical design
5. Electronics
6. Trajectory or square-drawing result
7. Green-obstacle image processing
8. Optional platform-locking mechanism
```

For GitHub, I would keep the README to **5–7 images maximum** and place the remaining thesis figures in `docs/figures/`.

## Source

This repository organization is based on the supplied graduation thesis and code appendix. The thesis describes the project as a reconfigurable system composed of three serial manipulators that can also operate together as a parallel mechanism, with kinematics, dynamics, electronics, trajectory generation, image processing and control covered across the project. The thesis's appendix list identifies trajectory generation, Raspberry Pi/ATmega communication, master/slave code, pure rotation, and square-drawing code as the main software artifacts.

See `docs/CODE_ORIGIN.md` for the provenance and limitations of the reconstructed source files.
