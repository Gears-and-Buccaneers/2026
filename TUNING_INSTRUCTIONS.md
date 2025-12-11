# Swerve Drive Tuning Instructions

This guide outlines the step-by-step process for tuning your swerve drive using Phoenix Tuner X and the robot code's Test Mode.

## Prerequisites

1.  **Phoenix Tuner X** installed and connected to the robot.
2.  **Driver Station** connected.
3.  **Robot Code** deployed to the robot.
4.  **Battery** fully charged (tuning with a low battery gives bad results).

---

## Phase 1: Steer Motor Tuning (On Ground)

**Goal:** Tune the steering motors to rotate quickly and accurately to the target angle.

**Setup:**
1.  Place the robot on the **carpet** (competition surface).
    *   *Why?* Tire scrub friction is the main force the steer motors fight. Tuning in the air is inaccurate.
2.  Open **Phoenix Tuner X**.
3.  Select one **Steer Motor** (e.g., `Front Left Steer`).

**Step 1: Find kS (Static Friction)**
1.  Go to **Control** tab.
    *   Mode: `TorqueCurrentFOC`.
2.  Go to **Plot** tab.
    *   Add Signal: `StatorCurrent`.
3.  **Enable** the robot.
4.  Slowly increase the **Output** slider.
5.  Find the current value where the wheel *just* starts to rotate in place against the carpet friction.
6.  Update `_steer_gains.k_s` in `generated/team_tuned_constants.py`.

**Step 2: Tune kP (Proportional Gain)**
1.  Go to **Control** tab.
    *   Mode: `Position TorqueCurrentFOC`.
2.  Go to **Plot** tab.
    *   Add Signals: `Position`, `ClosedLoopTarget`, `ClosedLoopError`.
3.  **Enable** the robot.
4.  Move the **Position** slider abruptly (e.g., 0 to 0.25 rotations).
5.  Watch the graph:
    *   **Too Slow:** Graph curves up slowly. -> **Increase kP**.
    *   **Overshoot/Wobble:** Graph goes past target and comes back. -> **Decrease kP**.
    *   **Good:** Graph snaps to target instantly with no overshoot.
6.  Update `_steer_gains.k_p` in `generated/team_tuned_constants.py`.

---

## Phase 2: Drive Motor Friction & Velocity Feedforward (On Blocks)

**Goal:** Tune the baseline friction and velocity constants for the drive motors.

**Setup:**
1.  Put the robot on **BLOCKS** (wheels off the ground).
    *   *Why?* Tuner X spins only one wheel. If on the ground, it drags the other 3 dead wheels, causing burnout.
2.  Select one **Drive Motor** (e.g., `Front Left Drive`).

**Step 1: Find kS (Static Friction)**
1.  Go to **Control** tab.
    *   Mode: `TorqueCurrentFOC`.
2.  **Enable** the robot.
3.  Slowly increase the **Output** slider.
4.  Find the current value where the wheel spins consistently.
5.  Update `_drive_gains.k_s` in `generated/team_tuned_constants.py`.

**Step 2: Verify kV (Velocity Feedforward)**
1.  Go to **Control** tab.
    *   Mode: `Velocity TorqueCurrentFOC`.
2.  Set Velocity to ~50% of max speed.
3.  Check if `Velocity` matches `ClosedLoopTarget`.
    *   If `Velocity` < `Target`: **Increase kV**.
    *   If `Velocity` > `Target`: **Decrease kV**.
4.  Update `_drive_gains.k_v` in `generated/team_tuned_constants.py`.

---

## Phase 3: Drive Motor PID (On Ground - Test Mode)

**Goal:** Tune the velocity PID under real robot weight and inertia.

**Setup:**
1.  Put the robot on the **carpet** (plenty of space, at least 10ft).
2.  **Deploy** the latest code (with the Test Mode logic).
3.  Open **Phoenix Tuner X** and select a **Drive Motor**.
4.  Go to **Plot** tab.
    *   Add Signals: `Velocity`, `ClosedLoopTarget`, `ClosedLoopError`.

**Procedure:**
1.  Open **Driver Station**.
2.  Switch to **Test** mode.
3.  **Enable**.
    *   *Action:* The robot will drive forward 1 meter (at 2.0 m/s), stop, drive backward 1 meter, stop, and repeat.
4.  Watch the **Tuner X Plot**:
    *   **Square Wave:** The Target will look like a square wave.
    *   **Sluggish (Curved Rise):** The Velocity takes too long to reach the Target. -> **Increase kP**.
    *   **Overshoot/Oscillation:** The Velocity spikes above the Target or wobbles. -> **Decrease kP**.
5.  Update `_drive_gains.k_p` in `generated/team_tuned_constants.py`.
    *   *Typical Range:* 1.0 - 10.0 Amps/RPS.

---

## Summary of Constants

| Constant | Tuned Where? | Method |
| :--- | :--- | :--- |
| **Steer kS** | Ground | Tuner X (TorqueCurrentFOC) |
| **Steer kP** | Ground | Tuner X (Position TorqueCurrentFOC) |
| **Drive kS** | **Blocks** | Tuner X (TorqueCurrentFOC) |
| **Drive kV** | **Blocks** | Tuner X (Velocity TorqueCurrentFOC) |
| **Drive kP** | Ground | **Driver Station Test Mode** |
