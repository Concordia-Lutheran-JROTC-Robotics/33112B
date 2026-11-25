# 33112B - VEX V5 Override (2026-27)

Robot code for team 33112B (Concordia Lutheran JROTC Robotics) for the 2026-27 VEX Robotics Competition season, Override.

The main idea of this project is to provide two programs for one robot. A VEX V5 robot only runs one binary at a time, and a match program has to do different things than a skills program, so the code is split into two PROS projects that share the same hardware map but serve different purposes:

- `matchDrive` — the program used in a normal match. It provides driver control (arcade drive, lift, intake). The 15 autonomous modes are still under development.
- `autonSkills` — the program used for a skills run, where a single robot gets 60 seconds of autonomous  and 60 seconds of driver control with no other teams on the field.

Everything is hand-written C++ on top of PROS (except for parts of the `drivetrain` class).

## What's in the repository

| Path | What it is |
| --- | --- |
| `matchDrive/` | PROS project for match play: driver control + left-side match autonomous |
| `autonSkills/` | PROS project for the autonomous skills run |
| `matchDrive/src/main.cpp` | Driver control, LCD autonomous selector, `Drivetrain` class, left-side auton |
| `autonSkills/src/main.cpp` | Full skills routine (the bulk of the autonomous work) |
| `matchDrive/notebookImages/` | Engineering notebook screenshots of the control schemes |
| `*/project.pros` | PROS project metadata: kernel `4.2.2`, template `liblvgl@9.2.0` |

Both projects are independent PROS projects.

## The robot

+ **drivetrain** — four 11W motors + two 5.5W motors
+ **lift** — two 5.5W motors
+ **intake** — two 5.5W motors

### Ports

Subsystem | Ports | Name
|---|---|---
Left drivetrain | 17, 14, 18 | `left_mg`
Right drivetrain | -4, -8, -10 | `right_mg`
Lift | 16, -6 | `lift_mg `
Intake | -2, 12 | `intake_mg`
Inertial sensor (not in use) | 21 | `imu`
Controller | `E_CONTROLLER_MASTER` | `master`
 
*negative numbers are reversed


### Driver controls

| Input | Action |
| --- | --- |
| Left stick Y / right stick X | Arcade drive (`left = dir - turn`, `right = dir + turn`) |
| L1 | Lift up while held |
| L2 | Lift down while held, but never below the position the lift started at |
| D-pad Down | Toggle intake outward |
| D-pad Right | Toggle intake inward |
| R1 | Score macro: drop lift to its start position, then reverse intake and back the robot up briefly |

## Autonomous

### Match autonomous not implemented yet...

### `autonSkills` — encoder turns

The skills routing is an open-loop sequence: ~40 `drive()` / `turn()` / lift / intake calls with fixed delays. It uses the encoder unit sensing built in to the motors:

```cpp
static constexpr double TICKS_PER_INCH   = 40.1;  // ← CALIBRATE THIS
static constexpr double TICKS_PER_DEGREE = 4.06;  // ← 3.83 gave 85° for a 90° turn → 3.83 × (90/85) ≈ 4.06
```

`turn()` converts degrees to ticks, drives the two sides in opposite directions, and stops when the average absolute encoder position reaches the target. `drive()` also prints live left/right/target values to the LCD for debugging purposes.

### Autonomous selection

Both projects register LCD button presses in `initialize()`:

+ **Button 0 (left)** — left-side autonomous
+ **Button 2 (right** — right-side autonomous

The selection is stored in `autonLeftSide` /  `autonRightSide` and reported on the LCD.

## Tech stack

+ **Language**: C++ (PROS 4 kernel, PROS 4 library)
+ **Framework**: PROS 4.2.2 for VEX V5
+ **Build**: PROS CLI / `pros make`
+ **Brain UI**: LVGL 9.2.0 via PROS `liblvgl` template, driven through PROS's ``llemu`` LCD emulation
+ **Hardware**: VEX V5 brain, V5 smart motors, VEX controller, various VEX construction parts
+ **Version control**: Git + GitHub


## How it was made

Commit history / build log from August 18, 2026 to November 25, 2026:

| Date | Commit | What happened |
| --- | --- | --- |
| 2026-08-18 | `83124ed` | Initial PROS project commit at the repo root |
| 2026-08-20 | `fb3239c` | "Variable Mismatch condition" — the first autonomous-selector branch guard |
| 2026-09-15 | `e4de357` | Left-side autonomous loaded by default |
| 2026-10-14 | `b41df06` | Pneumatic claw piston added (ADI port `A`) |
| 2026-10-15 | `7cc7c13` | Piston toggle on the A button, later remapped to Y |
| 2026-10-23 | `0d92b86` | Notebook screenshots added; source renamed to `src/main.cpp` |
| 2026-10-26 | `ab9bc7e` | Thread crash + "continuous press" bug fixed |
| 2026-10-26 | `5a8aa6d` | Lift motor group on L1/L2, `get_digital_new_press` for the piston |
| 2026-10-27 | `396903e`, `065a36f` | Claw rotation + arm motors integrated, brake-hold modes set |
| 2026-10-28 | `910d210`, `1fe7b0c` | Button handlers moved into `pros::Task`s; `delay(20)` added to stop CPU starvation |
| 2026-10-31 | `e267661` → `af4566c` | LemLib + `{fmt}` added, then removed hours later |
| 2026-10-31 | `cf9cd19`, `d5e279a` | Timeout protection, encoder taring, `autonSkills/` split out |
| 2026-11-09 | `783b820` | Intake added |
| 2026-11-10 | `b211bb4` | **Bot redo**: ports reassigned, claw motors and piston removed |
| 2026-11-13 | `c137e0d` | Skills routine created and reorganised (directory fixes in `02337e8`, `17027e6`, `43e77a1`) |
| 2026-11-16 → 18 | `82f1664`, `cf7d55b`, `a92ebb3` | IMU calibration moved, reverted, then replaced with a fixed 2500 ms delay |
| 2026-11-19/20 | `54cee6b`, `7ea19dc` | Acceleration and deceleration bursts added to `drive()` |
| 2026-11-21 | `2584454` | **"WE HAVE A WORKING AUTONOMOUS!!!"** — the skills routine finally runs end to end |
| 2026-11-23 | `1848609` | Autonomous upgraded, `drivetain_basic/` renamed to `matchDrive/` |
| 2026-11-24/25 | `15e4260`, `a5150c1` | Skills tuning, match-side updates, merge |
| 2026-11-25 | `0b26e66` | Removed non-functional autonomous match code. |
| 2026-11-25 | `0b26e66`, `c65dc4b` , `fd72c6e`| Remove non-functional match autonomous code, duplicate `main.cpp` file from `matchDrive`, and `autonomous()` call from `opcontrol()` |
| 2026-11-25 | `latest` | Add detailed README explaining project |


1. 2 weeks of building driver-control program with the robot mechanisms
2. Robot rebuild that deleted most of the code
3. Month of autonomous creation and tuning

## Setup and Run

### ** Will not run without 33112B's robot, see video for example of program. **

### Prerequisites

1. **PROS CLI 4.x** — follow the [PROS install instructions](https://pros.cs.purdue.edu/v5/getting-started/)
   (PROS Toolchain + CLI). `pros --version` should work.
2. **VS Code with the PROS extension** (the normal way to build and upload), or the CLI alone.
3. **VEX V5 brain**
4. **Micro USB cable** — connect either to brain or controller linked to brain
5. **Git**

### Clone

```bash
git clone https://github.com/Concordia-Lutheran-JROTC-Robotics/33112B.git
cd 33112B
```

### Build projects

Each folder is a separate PROS project — open the folder corresponding to the correct project.

Using PROS CLI:
```bash
cd matchDrive
# OR
cd autonSkills
pros make
pros upload
```

Using VSCode:
**File** — **Open Folder** — `matchDrive` or `autonSkills`, then use PROS sidebar to select *Build & Upload*

### Running the robot

+ `matchDrive`: enable driver control and drive. For autonomous (not functional yet), pick the side using the left and right buttons on the brain's LCD before the match.
+ `autonSkills`: enable autonomous mode. The `autonomous()` function is called and run, then the program exits.

## Credits

+ Team 33112B, Concordia Lutheran JROTC Robotics
+ Code: `RoboWhisperer`
+ Tuning and autonomous planning: `benk4455`
- Built on [PROS](https://pros.cs.purdue.edu/) by Purdue ACM SIGBots, and
  [LVGL](https://lvgl.io/) for the brain's screen.

