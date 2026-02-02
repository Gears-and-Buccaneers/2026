# Choreo Trajectories

Place your Choreo trajectory files (`.traj`) in this folder.

## How to create trajectories

1. Download and install Choreo from https://choreo.autos/installation/
2. Create a new project or open an existing one
3. Configure your robot's parameters (dimensions, max velocity, etc.)
4. Design your path using waypoints
5. Generate the trajectory
6. Save the `.traj` file to this folder

## File naming

The trajectory file name (without extension) should match what you use in your
autonomous code. For example:

- File: `simple_path.traj`
- Code: `TRAJECTORY_NAME = "simple_path"`

## Example autonomous that uses these trajectories

See `autonomous/choreo_examples.py` for example implementations.
