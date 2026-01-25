"""Base class for Choreo-based autonomous routines.

This module provides a base class that makes it easy to create autonomous
routines using Choreo trajectories. Students can extend ChoreoAuto to
create their own autonomous modes.

Example usage:
    class MyAuto(ChoreoAuto):
        MODE_NAME = "My Cool Auto"
        TRAJECTORY_NAME = "my_trajectory"  # Loads from deploy/choreo/my_trajectory.traj

        # Optional: Override to add actions at different points
        def onTrajectoryStart(self):
            self.shooter.spin_up()

        def onTrajectoryEnd(self):
            self.shooter.shoot()
"""

from typing import Callable, TypeAlias

import magicbot as mb
import wpilib
from choreo import load_swerve_trajectory
from choreo.trajectory import SwerveSample, SwerveTrajectory
from wpilib import DriverStation
from wpimath.geometry import Pose2d

import components

ListNamedCallbacks: TypeAlias = list[tuple[str, Callable[[], None] | None]]


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
    _trajectoryCache: dict[str, SwerveTrajectory | None] = {}

    def __init__(self) -> None:
        """Initialize the autonomous state machine."""
        super().__init__()
        self._trajectory: SwerveTrajectory | None = None
        self._timer = wpilib.Timer()
        self._trajectoryFinished = False

    @classmethod
    def loadTrajectory(cls, name: str) -> SwerveTrajectory | None:
        """Load a trajectory by name, with caching.

        Trajectories are cached at the class level so they only need to be
        loaded once, even if multiple auto modes use the same trajectory.

        Args:
            name: The trajectory name (without .traj extension).

        Returns:
            The loaded trajectory, or None if it couldn't be loaded.
        """
        if name not in cls._trajectoryCache:
            try:
                # load_swerve_trajectory returns Optional[SwerveTrajectory]
                # It loads from deploy/choreo/{name}.traj
                trajectory = load_swerve_trajectory(name)
                cls._trajectoryCache[name] = trajectory
                if trajectory is None:
                    wpilib.reportWarning(
                        f"Choreo: Could not load trajectory '{name}'. Make sure deploy/choreo/{name}.traj exists.",
                        False,
                    )
            except Exception as e:
                wpilib.reportError(f"Choreo: Error loading trajectory '{name}': {e}", True)
                cls._trajectoryCache[name] = None

        return cls._trajectoryCache.get(name)

    @staticmethod
    def isRedAlliance() -> bool:
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
            self._trajectory = self.loadTrajectory(self.TRAJECTORY_NAME)
        else:
            wpilib.reportError(f"{self.MODE_NAME} has no TRAJECTORY_NAME set!", False)
            self._trajectory = None

        # Reset state
        self._trajectoryFinished = False

        # Set the robot's starting pose to match the trajectory
        if self._trajectory is not None:
            initial_pose = self._trajectory.get_initial_pose(self.isRedAlliance())
            if initial_pose is not None:
                self.drivetrain.resetPose(initial_pose)
                wpilib.reportError(
                    f"ATTENTION: Starting pose set to {initial_pose}, make sure that the right alliance is selected.",
                    False,
                )

        # Call the user-defined start hook
        self.onTrajectoryStart()

    def onTrajectoryStart(self) -> None:
        """Called when the trajectory starts. Override this to add custom behavior.

        Example: Start spinning up a shooter, deploy an intake, etc.
        """
        pass

    def onTrajectoryEnd(self) -> None:
        """Called when the trajectory ends. Override this to add custom behavior.

        Example: Shoot a game piece, stop the intake, etc.
        """
        pass

    def duringTrajectory(self, elapsedTime: float, totalTime: float) -> None:
        """Called every loop iteration while the trajectory is running.

        Override this to perform actions while the robot is moving.
        This is useful for time-based actions like:
        - Spinning up a shooter halfway through the path
        - Running an intake the entire time
        - Deploying mechanisms at specific times

        Args:
            elapsedTime: Seconds since the trajectory started.
            totalTime: Total duration of the trajectory in seconds.

        Example:
            def duringTrajectory(self, elapsedTime: float, totalTime: float) -> None:
                # Run intake the whole time
                self.intake.run()

                # Spin up shooter when 75% through the path
                if elapsedTime > totalTime * 0.75:
                    self.shooter.spin_up()
        """
        pass

    # -------------------------------------------------------------------------
    # State Machine States
    # -------------------------------------------------------------------------

    @mb.state(first=True)
    def startTrajectory(self) -> None:
        """Start following the trajectory."""
        if self._trajectory is None:
            # No trajectory loaded, just stop
            self.drivetrain.stop()
            return

        # Start the timer for trajectory sampling
        self._timer.restart()
        self.next_state("followTrajectory")

    @mb.state()
    def followTrajectory(self) -> None:
        """Follow the trajectory by sampling it at the current time."""
        if self._trajectory is None:
            self.next_state("trajectoryComplete")
            return

        # Get the current time in the trajectory
        elapsedTime = self._timer.get()

        # Check if we've finished the trajectory
        totalTime = self._trajectory.get_total_time()
        if elapsedTime >= totalTime:
            self.next_state("trajectoryComplete")
            return

        # Call the duringTrajectory hook for user actions while moving
        self.duringTrajectory(elapsedTime, totalTime)

        # Sample the trajectory at the current time
        sample = self._trajectory.sample_at(elapsedTime, self.isRedAlliance())

        if sample is not None:
            # Follow the trajectory sample
            self.drivetrain.followTrajectory(sample)
        else:
            # Couldn't get a sample, just stop
            self.drivetrain.stop()

    @mb.state()
    def trajectoryComplete(self) -> None:
        """Called when the trajectory is finished."""
        if not self._trajectoryFinished:
            self._trajectoryFinished = True
            # Stop the robot
            self.drivetrain.stop()
            # Call user-defined end hook
            self.onTrajectoryEnd()

        # Stay stopped
        self.drivetrain.stop()


class ChoreoMultiTrajectoryAuto(mb.AutonomousStateMachine):
    """Base class for autonomous routines with multiple Choreo trajectories.

    This class allows you to chain multiple trajectories together with
    custom actions between them. Override the setupTrajectories() method
    to define your sequence.

    Example:
        class MyComplexAuto(ChoreoMultiTrajectoryAuto):
            MODE_NAME = "Complex Auto"

            shooter: components.Shooter  # Will be injected

            def setupTrajectories(self):
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
        self._trajectoryList: ListNamedCallbacks = []
        self._currentTrajectoryIndex = 0
        self._currentTrajectory: SwerveTrajectory | None = None
        self._timer = wpilib.Timer()

    def setupTrajectories(self) -> ListNamedCallbacks:
        """Define the sequence of trajectories and actions.

        Override this method to define your autonomous sequence.

        Returns:
            A list of (trajectory_name, action_callable) tuples.
            The action_callable will be called after the trajectory completes.
            Use None for no action.
        """
        return []

    def duringTrajectory(self, trajectoryIndex: int, trajectoryName: str, elapsedTime: float, totalTime: float) -> None:
        """Called every loop iteration while any trajectory is running.

        Override this to perform actions while the robot is moving.

        Args:
            trajectoryIndex: Which trajectory we're on (0, 1, 2, ...).
            trajectoryName: Name of the current trajectory.
            elapsedTime: Seconds since this trajectory started.
            totalTime: Total duration of this trajectory in seconds.

        Example:
            def duringTrajectory(self, trajectoryIndex, trajectoryName, elapsedTime, totalTime):
                # Always run intake while moving
                self.intake.run()

                # Spin up shooter during the "drive_to_speaker" trajectory
                if trajectoryName == "drive_to_speaker" and elapsedTime > totalTime * 0.5:
                    self.shooter.spin_up()
        """
        pass

    def onEnable(self) -> None:
        """Called when autonomous mode starts."""
        super().on_enable()

        self._trajectoryList = self.setupTrajectories()
        self._currentTrajectoryIndex = 0
        self._loadCurrentTrajectory()

        # Set initial pose from first trajectory
        if self._currentTrajectory is not None:
            initial_pose = self._currentTrajectory.get_initial_pose(ChoreoAuto.isRedAlliance())
            if initial_pose is not None:
                self.drivetrain.resetPose(initial_pose)

    def _loadCurrentTrajectory(self) -> None:
        """Load the current trajectory from the sequence."""
        if self._currentTrajectoryIndex < len(self._trajectoryList):
            name, _ = self._trajectoryList[self._currentTrajectoryIndex]
            self._currentTrajectory = ChoreoAuto.loadTrajectory(name)
        else:
            self._currentTrajectory = None

    @mb.state(first=True)
    def startTrajectory(self) -> None:
        """Start following the current trajectory."""
        if self._currentTrajectory is None:
            self.next_state("allComplete")
            return

        self._timer.restart()
        self.next_state("followTrajectory")

    @mb.state()
    def followTrajectory(self) -> None:
        """Follow the current trajectory."""
        if self._currentTrajectory is None:
            self.next_state("runAction")
            return

        elapsedTime = self._timer.get()
        totalTime = self._currentTrajectory.get_total_time()

        if elapsedTime >= totalTime:
            self.next_state("runAction")
            return

        # Call the duringTrajectory hook for user actions while moving
        if self._currentTrajectoryIndex < len(self._trajectoryList):
            trajectoryName, _ = self._trajectoryList[self._currentTrajectoryIndex]
            self.duringTrajectory(self._currentTrajectoryIndex, trajectoryName, elapsedTime, totalTime)

        sample = self._currentTrajectory.sample_at(elapsedTime, ChoreoAuto.isRedAlliance())
        if sample is not None:
            self.drivetrain.followTrajectory(sample)
        else:
            self.drivetrain.stop()

    @mb.state()
    def runAction(self) -> None:
        """Run the action for the current trajectory, then move to the next."""
        self.drivetrain.stop()

        # Run the action if one was defined
        if self._currentTrajectoryIndex < len(self._trajectoryList):
            _, action = self._trajectoryList[self._currentTrajectoryIndex]
            if action is not None:
                action()

        # Move to the next trajectory
        self._currentTrajectoryIndex += 1
        self._loadCurrentTrajectory()

        if self._currentTrajectory is not None:
            self.next_state("startTrajectory")
        else:
            self.next_state("allComplete")

    @mb.state()
    def allComplete(self) -> None:
        """All trajectories complete, just stay stopped."""
        self.drivetrain.stop()
