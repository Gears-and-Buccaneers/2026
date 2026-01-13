# Adding Auto Modes

Use **Choreo trajectories** to create autonomous routines with smooth, optimized paths for your robot.

---

## Choreo Trajectory-Based Autonomous

Choreo is a graphical trajectory planner that creates smooth, optimized paths for your robot to follow. This is the recommended approach for complex autonomous routines.

### Getting Started with Choreo

1. **Download Choreo** from [choreo.autos](https://choreo.autos/installation/)
2. **Open your project's Choreo file** (or create a new one)
3. **Design your path** by placing waypoints
4. **Generate the trajectory** - Choreo will optimize the path for your robot
5. **Save** - The trajectory file goes to `deploy/choreo/your_trajectory.traj`

### Creating a Simple Choreo Auto

1. Create a new Python file in the `autonomous/` folder
2. Extend the `ChoreoAuto` base class:

```python
from autonomous.choreo_auto import ChoreoAuto

class MyAuto(ChoreoAuto):
    MODE_NAME = "My Cool Auto"                    # Display name on Driver Station
    TRAJECTORY_NAME = "my_trajectory"             # Loads deploy/choreo/my_trajectory.traj

    def on_trajectory_start(self) -> None:
        """Called when auto starts - spin up shooter, deploy intake, etc."""
        pass

    def on_trajectory_end(self) -> None:
        """Called when trajectory finishes - score, stop intake, etc."""
        pass
```

### Creating a Multi-Trajectory Auto

For more complex autos that chain multiple paths with actions between them:

```python
from autonomous.choreo_auto import ChoreoMultiTrajectoryAuto
import components

class TwoPieceAuto(ChoreoMultiTrajectoryAuto):
    MODE_NAME = "Two Piece Auto"

    shooter: components.Shooter  # Will be injected by MagicBot

    def setup_trajectories(self):
        return [
            ("start_to_piece", self.intake),      # Drive to piece, then intake
            ("piece_to_hub", self.shoot),     # Drive to hub, then shoot
        ]

    def intake(self):
        self.shooter.intake()

    def shoot(self):
        self.shooter.shoot()
```

### Trajectory Files Location

Place your `.traj` files in:
```
deploy/
  choreo/
    simple_path.traj
    start_to_piece.traj
    piece_to_hub.traj
```

### Tuning Trajectory Following

The trajectory follower uses PID controllers to correct position errors. Tune these values in `components/swerve.py`:

```python
class Drivetrain:
    TRAJECTORY_X_KP = 10.0      # Position correction in X (forward/back)
    TRAJECTORY_Y_KP = 10.0      # Position correction in Y (left/right)
    TRAJECTORY_HEADING_KP = 7.5  # Heading correction
```

If the robot overshoots the path, reduce the gains. If it doesn't track well, increase them.

### Running Actions During a Trajectory

Override `during_trajectory()` to perform actions while the robot is moving (called every loop):

```python
class MyAuto(ChoreoAuto):
    MODE_NAME = "Shoot While Moving"
    TRAJECTORY_NAME = "drive_to_speaker"

    shooter: components.Shooter

    def during_trajectory(self, elapsed_time: float, total_time: float) -> None:
        # Run intake the whole time
        self.intake.run()

        # Spin up shooter when 75% through the path
        if elapsed_time > total_time * 0.75:
            self.shooter.spin_up()
```

### Running Actions During Multi-Trajectory Autos

For `ChoreoMultiTrajectoryAuto`, you can run actions both **during** and **between** trajectories:

```python
class TwoPieceAuto(ChoreoMultiTrajectoryAuto):
    MODE_NAME = "Two Piece"

    shooter: components.Shooter

    def setup_trajectories(self):
        return [
            ("drive_to_piece", self.intake),   # intake() runs AFTER this path
            ("drive_to_speaker", self.shoot),  # shoot() runs AFTER this path
        ]

    def during_trajectory(self, trajectory_index, trajectory_name, elapsed_time, total_time):
        """Runs every loop while moving."""
        # Spin up shooter halfway through the drive to speaker
        if trajectory_name == "drive_to_speaker" and elapsed_time > total_time * 0.5:
            self.shooter.spin_up()

    def intake(self):
        self.shooter.intake()

    def shoot(self):
        self.shooter.shoot()
```

---

## Choreo Resources

- [Choreo Documentation](https://choreo.autos/)
- [ChoreoLib Getting Started](https://choreo.autos/choreolib/getting-started/)
- [Trajectory API](https://choreo.autos/choreolib/trajectory-api/)
