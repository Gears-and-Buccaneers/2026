"""Base class for Choreo-based autonomous routines.

This module provides a base class that makes it easy to create autonomous
routines using Choreo trajectories. Students can extend ChoreoAuto to
create their own autonomous modes.

Example usage:
    class MyAuto(ChoreoAuto):
        MODE_NAME = "My Cool Auto"
        TRAJECTORY_NAME = "my_trajectory"  # Loads from deploy/choreo/my_trajectory.traj

        # Optional: Override to add actions at different points
        def on_trajectory_start(self):
            self.shooter.spin_up()

        def on_trajectory_end(self):
            self.shooter.shoot()
"""

import re
from typing import Callable, TypeAlias

import magicbot as mb
import wpilib
from choreo import load_swerve_trajectory
from choreo.trajectory import EventMarker, SwerveSample, SwerveTrajectory
from wpilib import DriverStation
from wpimath.geometry import Pose2d

import components

Callback: TypeAlias = Callable[[], None]
MultiTrajectoryStep: TypeAlias = tuple[str | None, Callback | None]
TrajectoriesAndCallbacks: TypeAlias = list[MultiTrajectoryStep]
EventCallbacks: TypeAlias = dict[str, Callback]


class _EventMarkerTracker:
    """Tracks and fires event callbacks as trajectory time progresses.

    Created once per trajectory load, this fires each matching callback
    exactly once when its marker timestamp is reached.
    """

    def __init__(self, events: list[EventMarker], callbacks: EventCallbacks, mode_name: str) -> None:
        self._events = sorted(events, key=lambda e: e.timestamp)
        self._callbacks = callbacks
        self._mode_name = mode_name
        self._next_index = 0

    def fire_up_to(self, elapsed_time: float) -> None:
        """Fire all event callbacks whose timestamps have been reached."""
        while self._next_index < len(self._events):
            marker = self._events[self._next_index]
            if marker.timestamp > elapsed_time:
                return
            self._next_index += 1
            callback = self._callbacks.get(marker.event)
            if callback is not None:
                try:
                    callback()
                except Exception as e:
                    wpilib.reportError(f"{self._mode_name}: event callback '{marker.event}' failed: {e}", True)


class ChoreoAuto(mb.AutonomousStateMachine):
    """Base class for Choreo trajectory-based autonomous routines.

    Subclass this and set TRAJECTORY_NAME to the name of your trajectory
    (without the .traj extension). The trajectory file should be in
    deploy/choreo/your_trajectory.traj.

    Important: This base class has DISABLED = True so it doesn't appear in the
    Driver Station autonomous selector. Your subclass must set DISABLED = False
    to be selectable.

    Example:
        class MyAuto(ChoreoAuto):
            MODE_NAME = "My Auto"
            TRAJECTORY_NAME = "my_path"
            DISABLED = False  # Required!

    Attributes:
        MODE_NAME: Display name for the auto mode (required by MagicBot).
        TRAJECTORY_NAME: Name of the Choreo trajectory file to load.
        DISABLED: Set to False in your subclass to enable selection.
        DEFAULT: Set to True to make this the default auto mode.
    """

    # Subclasses must override these
    MODE_NAME = "Choreo Auto Base"
    TRAJECTORY_NAME: str = ""
    DISABLED = True  # Base class - subclasses set DISABLED = False

    # Components injected by MagicBot
    drivetrain: components.Drivetrain

    # Class-level trajectory cache to avoid reloading
    _trajectory_cache: dict[str, SwerveTrajectory | None] = {}
    _segment_cache: dict[tuple[str, int], SwerveTrajectory | None] = {}

    def __init__(self) -> None:
        """Initialize the autonomous state machine."""
        super().__init__()
        self._trajectory: SwerveTrajectory | None = None
        self._timer = wpilib.Timer()
        self._trajectory_finished = False
        self._event_tracker: _EventMarkerTracker | None = None

    @classmethod
    def load_trajectory(cls, name: str) -> SwerveTrajectory | None:
        """Load a trajectory by name, with caching.

        Trajectories are cached at the class level so they only need to be
        loaded once, even if multiple auto modes use the same trajectory.

        Args:
            name: The trajectory name (without .traj extension).

        Returns:
            The loaded trajectory, or None if it couldn't be loaded.
        """
        if name not in cls._trajectory_cache:
            try:
                # load_swerve_trajectory returns Optional[SwerveTrajectory]
                # It loads from deploy/choreo/{name}.traj
                trajectory = load_swerve_trajectory(name)
                cls._trajectory_cache[name] = trajectory
                if trajectory is None:
                    wpilib.reportWarning(
                        f"Choreo: Could not load trajectory '{name}'. Make sure deploy/choreo/{name}.traj exists.",
                        False,
                    )
            except Exception as e:
                wpilib.reportError(f"Choreo: Error loading trajectory '{name}': {e}", True)
                cls._trajectory_cache[name] = None

        return cls._trajectory_cache[name]

    @staticmethod
    def parse_trajectory_segment_spec(name: str) -> tuple[str, int] | None:
        """Parse names like "my_path[2]" into ("my_path", 2)."""
        match = re.fullmatch(r"(.+)\[(\d+)\]", name)
        if match is None:
            return None
        return match.group(1), int(match.group(2))

    @classmethod
    def load_trajectory_or_segment(cls, name: str) -> SwerveTrajectory | None:
        """Load a full trajectory, or a split segment via name[index]."""
        parsed = cls.parse_trajectory_segment_spec(name)
        if parsed is None:
            return cls.load_trajectory(name)

        base_name, segment_index = parsed
        return cls.load_trajectory_segment(base_name, segment_index)

    @classmethod
    def load_trajectory_segment(cls, name: str, segment_index: int) -> SwerveTrajectory | None:
        """Load one split segment from a trajectory, with caching.

        Segment i starts at split[i] and ends at split[i+1] (inclusive) if a
        following split exists, otherwise at the final sample.
        """
        cache_key = (name, segment_index)
        if cache_key in cls._segment_cache:
            return cls._segment_cache[cache_key]

        if segment_index < 0:
            wpilib.reportWarning(f"Choreo: invalid segment index {segment_index} for '{name}'.", False)
            cls._segment_cache[cache_key] = None
            return None

        full = cls.load_trajectory(name)
        if full is None:
            cls._segment_cache[cache_key] = None
            return None

        samples = full.samples
        if len(samples) == 0:
            wpilib.reportWarning(f"Choreo: trajectory '{name}' has no samples.", False)
            cls._segment_cache[cache_key] = None
            return None

        split_starts = sorted({0, *[i for i in full.splits if 0 <= i < len(samples)]})
        if segment_index >= len(split_starts):
            wpilib.reportWarning(
                f"Choreo: trajectory '{name}' has {len(split_starts)} segment(s); requested [{segment_index}]",
                False,
            )
            cls._segment_cache[cache_key] = None
            return None

        start_idx = split_starts[segment_index]
        if segment_index + 1 < len(split_starts):
            end_idx_inclusive = split_starts[segment_index + 1]
        else:
            end_idx_inclusive = len(samples) - 1

        segment_source_samples = samples[start_idx : end_idx_inclusive + 1]
        if len(segment_source_samples) == 0:
            wpilib.reportWarning(f"Choreo: segment '{name}[{segment_index}]' has no samples.", False)
            cls._segment_cache[cache_key] = None
            return None

        start_time = segment_source_samples[0].timestamp
        end_time = segment_source_samples[-1].timestamp

        # Rebase timestamps so each segment starts at t=0.
        segment_samples = [
            SwerveSample(
                sample.timestamp - start_time,
                sample.x,
                sample.y,
                sample.heading,
                sample.vx,
                sample.vy,
                sample.omega,
                sample.ax,
                sample.ay,
                sample.alpha,
                list(sample.fx),
                list(sample.fy),
            )
            for sample in segment_source_samples
        ]

        segment_events = [
            EventMarker(event.timestamp - start_time, event.event)
            for event in full.events
            if start_time <= event.timestamp <= end_time
        ]

        segment = SwerveTrajectory(
            f"{name}[{segment_index}]",
            segment_samples,
            [0],
            segment_events,
        )
        cls._segment_cache[cache_key] = segment
        return segment

    @staticmethod
    def is_red_alliance() -> bool:
        """Check if we're on the red alliance.

        Choreo trajectories can be mirrored for the red alliance.

        Returns:
            True if on red alliance, False otherwise.
        """
        alliance = DriverStation.getAlliance()
        if alliance is not None:
            return alliance == DriverStation.Alliance.kRed
        return False

    def on_enable(self) -> None:
        """Called when autonomous mode starts."""
        super().on_enable()

        # Load the trajectory if not already loaded
        if self.TRAJECTORY_NAME:
            self._trajectory = self.load_trajectory(self.TRAJECTORY_NAME)
        else:
            wpilib.reportError(f"{self.MODE_NAME} has no TRAJECTORY_NAME set!", False)
            self._trajectory = None

        # Reset state
        self._trajectory_finished = False

        # Set up event marker tracker
        if self._trajectory is not None:
            self._event_tracker = _EventMarkerTracker(
                self._trajectory.events or [],
                self.setup_event_callbacks(),
                self.MODE_NAME,
            )
        else:
            self._event_tracker = None

        # Set the robot's starting pose to match the trajectory
        if self._trajectory is not None:
            initial_pose = self._trajectory.get_initial_pose(self.is_red_alliance())
            if initial_pose is not None:
                self.drivetrain.resetPose(initial_pose)
                wpilib.reportError(
                    f"ATTENTION: Starting pose set to {initial_pose}, make sure that the right alliance is selected.",
                    False,
                )

        # Call the user-defined start hook
        self.on_trajectory_start()

    def on_trajectory_start(self) -> None:
        """Called when the trajectory starts. Override this to add custom behavior.

        Example: Start spinning up a shooter, deploy an intake, etc.
        """
        pass

    def on_trajectory_end(self) -> None:
        """Called when the trajectory ends. Override this to add custom behavior.

        Example: Shoot a game piece, stop the intake, etc.
        """
        pass

    def setup_event_callbacks(self) -> EventCallbacks:
        """Define callbacks for trajectory event marker names.

        Override this to map marker names to callbacks. Each callback fires
        once when its marker timestamp is reached during trajectory following.

        Returns:
            A dict mapping event marker names to callback functions.
        """
        return {}

    def during_trajectory(self, elapsed_time: float, total_time: float) -> None:
        """Called every loop iteration while the trajectory is running.

        Override this to perform actions while the robot is moving.
        This is useful for time-based actions like:
        - Spinning up a shooter halfway through the path
        - Running an intake the entire time
        - Deploying mechanisms at specific times

        Args:
            elapsed_time: Seconds since the trajectory started.
            total_time: Total duration of the trajectory in seconds.

        Example:
            def during_trajectory(self, elapsed_time: float, total_time: float) -> None:
                # Run intake the whole time
                self.intake.run()

                # Spin up shooter when 75% through the path
                if elapsed_time > total_time * 0.75:
                    self.shooter.spin_up()
        """
        pass

    # -------------------------------------------------------------------------
    # State Machine States
    # -------------------------------------------------------------------------

    @mb.state(first=True)
    def start_trajectory(self) -> None:
        """Start following the trajectory."""
        if self._trajectory is None:
            # No trajectory loaded, just stop
            self.drivetrain.stop()
            return

        # Start the timer for trajectory sampling
        self._timer.restart()
        self.next_state("follow_trajectory")

    @mb.state()
    def follow_trajectory(self) -> None:
        """Follow the trajectory by sampling it at the current time."""
        if self._trajectory is None:
            self.next_state("trajectory_complete")
            return

        # Get the current time in the trajectory
        elapsed_time = self._timer.get()

        # Check if we've finished the trajectory
        total_time = self._trajectory.get_total_time()

        # Fire event markers up to the current (or final) time
        if self._event_tracker is not None:
            self._event_tracker.fire_up_to(min(elapsed_time, total_time))

        if elapsed_time >= total_time:
            self.next_state("trajectory_complete")
            return

        # Call the during_trajectory hook for user actions while moving
        self.during_trajectory(elapsed_time, total_time)

        # Sample the trajectory at the current time
        sample = self._trajectory.sample_at(elapsed_time, self.is_red_alliance())

        if sample is not None:
            # Follow the trajectory sample
            self.drivetrain.followTrajectory(sample)
        else:
            # Couldn't get a sample, just stop
            self.drivetrain.stop()

    @mb.state()
    def trajectory_complete(self) -> None:
        """Called when the trajectory is finished."""
        if not self._trajectory_finished:
            self._trajectory_finished = True
            # Stop the robot
            self.drivetrain.stop()
            # Call user-defined end hook
            self.on_trajectory_end()

        # Stay stopped
        self.drivetrain.stop()


class ChoreoMultiTrajectoryAuto(mb.AutonomousStateMachine):
    """Base class for autonomous routines with multiple Choreo trajectories.

    This class allows you to chain multiple trajectories together with
    custom actions between them. Override the setup_trajectories() method
    to define your sequence.

    Any step may use trajectory_name=None to run an action-only step.

    Example:
        class MyComplexAuto(ChoreoMultiTrajectoryAuto):
            MODE_NAME = "Complex Auto"

            shooter: components.Shooter  # Will be injected

            def setup_trajectories(self):
                # Define a sequence of (trajectory_name, action_after) tuples
                return [
                    ("drive_to_piece1", self.intake_piece),
                    ("drive_to_hub", self.shoot_piece),
                    ("drive_to_piece2", None),  # No action after last trajectory
                ]

            def intake_piece(self):
                self.shooter.intake()

            def shoot_piece(self):
                self.shooter.shoot()
    """

    MODE_NAME = "Multi-Trajectory Auto Base"
    DISABLED = True  # Base class should not be registered as an autonomous mode

    drivetrain: components.Drivetrain

    def __init__(self) -> None:
        """Initialize the autonomous state machine."""
        super().__init__()
        self._trajectories: TrajectoriesAndCallbacks = []
        self._event_callbacks: EventCallbacks = {}
        self._current_trajectory_index = 0
        self._current_trajectory: SwerveTrajectory | None = None
        self._event_tracker: _EventMarkerTracker | None = None
        self._timer = wpilib.Timer()

    def setup_trajectories(self) -> TrajectoriesAndCallbacks:
        """Define the sequence of trajectories and actions.

        Override this method to define your autonomous sequence.

        Returns:
            A list of (trajectory_name, action_callable) tuples.
            The action_callable will be called after the trajectory completes.
            Use trajectory_name=None for an action-only step.
            Use None for no action.
            Use "trajectory_name[index]" to run a split segment.
        """
        return []

    def setup_event_callbacks(self) -> EventCallbacks:
        """Define callbacks for trajectory event marker names.

        Override this to map marker names to callbacks. As each marker time is
        reached during trajectory execution, the callback for that marker name
        (if present) will be invoked once.

        Returns:
            A dict mapping event marker names to callback functions.
        """
        return {}

    def during_trajectory(
        self, trajectory_index: int, trajectory_name: str | None, elapsed_time: float, total_time: float
    ) -> None:
        """Called every loop iteration while any trajectory is running.

        Override this to perform actions while the robot is moving.

        Args:
            trajectory_index: Which trajectory we're on (0, 1, 2, ...).
            trajectory_name: Name of the current trajectory, or None for an
                action-only step.
            elapsed_time: Seconds since this trajectory started.
            total_time: Total duration of this trajectory in seconds.

        Example:
            def during_trajectory(self, trajectory_index, trajectory_name, elapsed_time, total_time):
                # Always run intake while moving
                self.intake.run()

                # Spin up shooter during the "drive_to_speaker" trajectory
                if trajectory_name == "drive_to_speaker" and elapsed_time > total_time * 0.5:
                    self.shooter.spin_up()
        """
        pass

    def while_running(self) -> None:
        """Called every autonomous loop iteration while this mode is active.

        Override this for logic that should run each frame in addition to
        trajectory following and action callbacks.
        """
        pass

    def on_iteration(self, tm: float) -> None:
        """Run state-machine iteration, then per-frame hook."""
        super().on_iteration(tm)
        self.while_running()

    def on_enable(self) -> None:
        """Called when autonomous mode starts."""
        super().on_enable()

        self._trajectories = self.setup_trajectories()
        self._event_callbacks = self.setup_event_callbacks()

        self._current_trajectory_index = 0
        self._load_current_trajectory()

        # Set initial pose from first trajectory
        if self._current_trajectory is not None:
            initial_pose = self._current_trajectory.get_initial_pose(ChoreoAuto.is_red_alliance())
            if initial_pose is not None:
                self.drivetrain.resetPose(initial_pose)

    def _load_current_trajectory(self) -> None:
        """Load the current trajectory from the sequence."""
        if self._current_trajectory_index < len(self._trajectories):
            name, _ = self._trajectories[self._current_trajectory_index]
            if name is None:
                self._current_trajectory = None
            else:
                self._current_trajectory = ChoreoAuto.load_trajectory_or_segment(name)
        else:
            self._current_trajectory = None

        if self._current_trajectory is not None:
            self._event_tracker = _EventMarkerTracker(
                self._current_trajectory.events or [],
                self._event_callbacks,
                self.MODE_NAME,
            )
        else:
            self._event_tracker = None

    @mb.state(first=True)
    def start_trajectory(self) -> None:
        """Start following the current trajectory."""
        if self._current_trajectory is None:
            if self._current_trajectory_index < len(self._trajectories):
                self.next_state("run_action")
            else:
                self.next_state("all_complete")
            return

        self._timer.restart()
        self.next_state("follow_trajectory")

    @mb.state()
    def follow_trajectory(self) -> None:
        """Follow the current trajectory."""
        if self._current_trajectory is None:
            self.next_state("run_action")
            return

        elapsed_time = self._timer.get()
        total_time = self._current_trajectory.get_total_time()

        # Trigger markers as soon as their timestamp is reached.
        if self._event_tracker is not None:
            self._event_tracker.fire_up_to(min(elapsed_time, total_time))

        if elapsed_time >= total_time:
            self.next_state("run_action")
            return

        # Call the during_trajectory hook for user actions while moving
        if self._current_trajectory_index < len(self._trajectories):
            trajectory_name, _ = self._trajectories[self._current_trajectory_index]
            self.during_trajectory(self._current_trajectory_index, trajectory_name, elapsed_time, total_time)

        sample = self._current_trajectory.sample_at(elapsed_time, ChoreoAuto.is_red_alliance())
        if sample is not None:
            self.drivetrain.followTrajectory(sample)
        else:
            self.drivetrain.stop()

    @mb.state()
    def run_action(self) -> None:
        """Run the action for the current trajectory, then move to the next."""
        self.drivetrain.stop()

        # Run the action if one was defined
        if self._current_trajectory_index < len(self._trajectories):
            _, action = self._trajectories[self._current_trajectory_index]
            if action is not None:
                action()

        # Move to the next trajectory
        self._current_trajectory_index += 1
        self._load_current_trajectory()

        if self._current_trajectory_index < len(self._trajectories):
            self.next_state("start_trajectory")
        else:
            self.next_state("all_complete")

    @mb.state()
    def all_complete(self) -> None:
        """All trajectories complete, just stay stopped."""
        self.drivetrain.stop()


class ChoreoStateMachine(mb.AutonomousStateMachine):
    """Base class for state-machine autos that can follow Choreo trajectories.

    Unlike ChoreoAuto (single trajectory) and ChoreoMultiTrajectoryAuto (a fixed
    sequence of trajectories), this class gives subclasses full control over their
    state machine while providing helpers for trajectory following.

    The active trajectory is followed automatically every loop — there is no need
    to call a follow method manually from each state.

    Example subclass state:

        @mb.state(first=True)
        def drive_to_hub(self, initial_call: bool):
            if initial_call:
                self.run_trajectory("DriveToHub[0]", reset_pose=True)
            elif self.is_trajectory_done():
                self.next_state("spin_up")

    Important: This base class has DISABLED = True so it doesn't appear in the
    Driver Station autonomous selector. Your subclass must set DISABLED = False.
    """

    MODE_NAME = "(Replace this with your own Auto Name)"
    DISABLED = True

    # Injected by MagicBot
    drivetrain: components.Drivetrain

    def __init__(self) -> None:
        """Initialize trajectory state."""
        super().__init__()
        self._current_trajectory: SwerveTrajectory | None = None
        self._traj_timer = wpilib.Timer()
        self._event_tracker: _EventMarkerTracker | None = None
        self._has_run_trajectory: bool = False

    def on_enable(self) -> None:
        """Reset trajectory state when autonomous starts."""
        super().on_enable()
        self._current_trajectory = None
        self._event_tracker = None
        self._has_run_trajectory = False

    def run_trajectory(
        self,
        name: str,
        event_callbacks: EventCallbacks | None = None,
        reset_pose: bool | None = None,
    ) -> None:
        """Start following a named Choreo trajectory.

        Call this on ``initial_call`` inside a ``@state`` method. The trajectory
        will be followed automatically each loop. Use ``is_trajectory_done()`` to
        detect completion and transition to the next state.

        Args:
            name: Trajectory name (without .traj). Use ``"Name[N]"`` for a split segment.
            event_callbacks: Optional dict mapping event marker names to callback
                functions. Each callback is called once when its marker time is reached.
            reset_pose: If True, reset odometry to the trajectory's starting pose.
                Use this for the very first trajectory in an auto routine.
        """
        traj = ChoreoAuto.load_trajectory_or_segment(name)
        if traj is None:
            wpilib.reportError(f"{self.MODE_NAME}: could not load trajectory '{name}'", True)
            self._current_trajectory = None
            return

        if reset_pose is None:
            reset_pose = not self._has_run_trajectory

        if reset_pose:
            initial_pose = traj.get_initial_pose(ChoreoAuto.is_red_alliance())
            if initial_pose is not None:
                self.drivetrain.resetPose(initial_pose)

        self._has_run_trajectory = True

        self._current_trajectory = traj
        self._traj_timer.restart()
        self._event_tracker = _EventMarkerTracker(traj.events or [], event_callbacks or {}, self.MODE_NAME)

    def is_trajectory_done(self) -> bool:
        """Return True when the active trajectory has finished (or none is loaded)."""
        if self._current_trajectory is None:
            return True
        return self._traj_timer.get() >= self._current_trajectory.get_total_time()

    def on_iteration(self, tm: float) -> None:
        """Run the state machine, then advance the active trajectory."""
        super().on_iteration(tm)
        self._follow_active_trajectory()

    def _follow_active_trajectory(self) -> None:
        """Follow the active trajectory and fire event callbacks. Called each loop."""
        if self._current_trajectory is None:
            return

        elapsed = self._traj_timer.get()
        total = self._current_trajectory.get_total_time()

        # Fire event markers up to the current (or final) time
        if self._event_tracker is not None:
            self._event_tracker.fire_up_to(min(elapsed, total))

        if elapsed >= total:
            # Trajectory finished — clear it so is_trajectory_done() returns True
            # and we don't keep sampling past the end
            self._current_trajectory = None
            self._event_tracker = None
            return

        sample = self._current_trajectory.sample_at(elapsed, ChoreoAuto.is_red_alliance())
        if sample is not None:
            self.drivetrain.followTrajectory(sample)
        else:
            self.drivetrain.stop()
