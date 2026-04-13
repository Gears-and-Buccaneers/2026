# Shooter Velocity Tuning Guide

The shooter uses Phoenix6 **VelocityVoltage** closed-loop control on two TalonFX/Kraken X60 motors (top and bottom). The PID loop runs on the motor controller itself at 1 kHz — much faster than the roboRIO loop.

## Key facts

| Parameter     | Value                                      |
|---------------|--------------------------------------------|
| Motor         | Kraken X60                                 |
| Free speed    | 6065 RPM ≈ **101 RPS**                     |
| Observed      | 75% duty cycle → balls travel ~20 ft       |
| Velocity unit | Motor shaft **rotations per second (RPS)** |
| Backspin      | Bottom wheels a little faster than top     |

## What to tune

There are two independent sets of numbers:

1. **RPS targets** — *how fast* the wheels should spin at a given distance (determines where the ball lands)
2. **PID gains** — *how well* the motors track those targets (determines consistency shot-to-shot)

## Tuning order

### Phase 1: Verify PID tracking (do this once)

The default gains should be close. You're checking that the motors can accurately reach whatever speed you ask for.

**Where to edit gains:** [`robot.py`](../robot.py), method `createMotors()`, block that builds `shooter_slot0 = p6.configs.Slot0Configs()` and sets `shooter_slot0.k_s`, `shooter_slot0.k_v`, `shooter_slot0.k_p`, etc., then applies it to `shooterMotorTop` and `shooterMotorBottom`. That is the **velocity** Slot0 used by `VelocityVoltage` on the shooter. Do **not** change the `k_s` / `k_v` / `k_p` passed to `utils.setMotorMotionMagic()` earlier in the same method—that path configures Motion Magic for all non-swerve motors (extension, intake, transit) and is not the shooter flywheel velocity loop.

1. Put the robot on blocks (wheels off the ground) or just spin up without shooting.
2. Set fallback spin and watch **`/components/pewpew/`** NetworkTables values on the dashboard:
   - `TopRPSTarget` / `TopRPSActual`
   - `BottomRPSTarget` / `BottomRPSActual`
   - `TopRPSError` / `BottomRPSError`
3. **If error is small (< 2 RPS)** → PID is fine, move to Phase 2.
4. **If error is large and positive** (motors too slow) → increase `k_v` by 0.01 at a time.
5. **If error oscillates** (bouncing ±) → decrease `k_p` by half.
6. **If wheels won't start at low targets** (< 20 RPS) → increase `k_s` by 0.05.

Once the error stays under ~2 RPS at multiple speed targets, the PID is done. You shouldn't need to touch it again.

### Phase 2: Tune 6 ft RPS values

1. Place the robot shooter exactly **6 feet** from the center of the hub.
2. Spin up (fallback or smart aim) and shoot balls.
3. Adjust `topRPS6ft` and `bottomRPS6ft` on the dashboard until balls consistently land in the hub.
   - **Balls going too far?** Decrease both values by 2–3 RPS.
   - **Balls falling short?** Increase both values by 2–3 RPS.
   - **Balls tumbling or curving?** Adjust the top/bottom ratio (see Backspin below).
4. Write down the final values.

### Phase 3: Tune 20 ft RPS values

1. Move the robot shooter to **20 feet** from the hub.
2. Same process: adjust `topRPS20ft` and `bottomRPS20ft` until balls land in the hub.
3. Write down the final values.

The code linearly interpolates between 6 ft and 20 ft (and extrapolates beyond). You can verify at 13 ft (midpoint) without any extra tuning — if balls land short or long there, it may mean the relationship isn't perfectly linear, but it's hopefully close enough.

### Phase 4: Set fallback values

Set `fallbackTopRPS` and `fallbackBottomRPS` to whatever works from a typical scoring position your drivers use most. Maybe backed up against the climber, or adjacent to it?

## Backspin

Running the bottom wheel faster than the top gives the ball backspin, which stabilizes its flight and helps it drop into the hub. The defaults use a ~10% split (bottom is ~10% faster than top). If balls are nosediving, reduce the split. If balls are floating past the hub, increase it slightly.

## Starting values

These are derived from the observed 75% duty ≈ 76 RPS → 20 ft, with a ~10% bottom/top split for backspin:

| Tunable             | Starting value | What it controls                            |
|---------------------|----------------|---------------------------------------------|
| `topRPS6ft`         |       38       | Top motor speed at 6 ft                     |
| `bottomRPS6ft`      |       42       | Bottom motor speed at 6 ft                  |
| `topRPS20ft`        |       72       | Top motor speed at 20 ft                    |
| `bottomRPS20ft`     |       80       | Bottom motor speed at 20 ft                 |
| `fallbackTopRPS`    |       52       | Top motor speed for fixed-distance shots    |
| `fallbackBottomRPS` |       58       | Bottom motor speed for fixed-distance shots |

## PID gain reference

Gains are configured in [`robot.py`](../robot.py) inside `createMotors()` on **`shooter_slot0`** (`p6.configs.Slot0Configs`), then applied to both `shooterMotorTop` and `shooterMotorBottom`. Lines move over time; search that method for `shooter_slot0.k_s` / `k_v` / `k_p`.

| Gain  | Initially | Role                               | When to change                                       |
|-------|-----------|------------------------------------|------------------------------------------------------|
| `k_s` |    0.15   | Overcomes static friction          | Shooters stall at low targets → increase by 0.05     |
| `k_v` |    0.12   | Main feedforward (≈ 12V / 101 RPS) | Consistent undershoot → +0.01; overshoot → −0.01     |
| `k_a` |    0.01   | Acceleration feedforward           | Rarely needs changing                                |
| `k_p` |    0.1    | Corrects remaining error           | Slow to reach target → double; oscillating → halve   |
| `k_i` |    0.0    | Fix persistent offset              | Try 0.01 if ~1 RPS offset never goes away            |
| `k_d` |    0.0    | Damps overshoot/ringing            | Try 0.005 if wheels ring after a ball passes through |

## Tips

- **Change one thing at a time.** If you adjust gains and RPS simultaneously you won't know what fixed (or broke) it.
- **Shoot 3–5 balls** at each setting before deciding to change. One shot can be an outlier.
- **Watch the dashboard**, not just the ball. If `TopRPSActual` matches `TopRPSTarget` but balls aren't landing right, the PID is fine — change the RPS target.
- **Log your changes.** Write down each value you try and what happened. It's easy to lose track.
