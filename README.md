# Gears and Buccaneers FRC Robot Code

Pure Python codebase using [RobotPy](https://robotpy.readthedocs.io/en/stable/)
with the [MagicBot](https://robotpy.readthedocs.io/en/latest/frameworks/magicbot.html) framework.


## Installation

0. If not already installed, install [uv](https://docs.astral.sh/uv/getting-started/installation/)
   and [git](https://git-scm.com/downloads).
   * Not used git before? Be sure to [set your username and email](https://stackoverflow.com/a/33024593/405017).

1. Clone the GitHub repo. One way is to run this command in a terminal:

   ```sh
   git clone git@github.com:Gears-and-Buccaneers/2026.git
   ```

2. In a terminal in the working directory for the cloned files (hint: `cd 2026`), run:

   ```sh
   uv sync             # Create a virtual python environment with necessary pre-requisites
   uv run robotpy sync # Install all the robotpy libraries
   ```

3. Open the folder in VS Code.
   * Though there's little likelihood of conflict, you may wish to create a new
     [VS Code profile](https://code.visualstudio.com/docs/configure/profiles)
     with only the recommended VS Code extensions installed.


## Developing the Robot Code

See lots of sample code in
[RobotPy Examples](https://github.com/robotpy/examples/tree/main)
for isolated snippets demonstrating how to accomplish common tasks.


## Running the Simulator

Open a Terminal in VS Code (hint: `` ctrl+` ``), and then run `robotpy sim` to start the simulator.

   * _Note that when you open a Terminal inside VS Code it should automatically activate the virtual environment for you._
     _This is what enables `robotpy` as its own command, without the `uv run` prefix._

     _If VS Code does not activate the virtual environment automatically,_
     _you can run `source .venv/bin/activate` to activate the venv (🤮)_
     _or you can just run `uv run robotpy sim` to run a command inside the venv._

   * If you connect a PS4 controller, you should be able to drive the robot around in the simulator
     using the left and right sticks.

     Note that the first time you will need to drag the controller from the "System Joysticks" window onto
     "Joystick 0" in the "Joysticks" window to connect the controller to the right joystick "port".

     Don't have a controller? You can drag "Keyboard 0" from the "System Joysticks" window onto "Joystick 0"
     instead to use the keyboard for driving.


## What are all these files?!

There's a lot of non-robot files and directories at the root of this repo,
especially after you run `uv sync` to create the virtual environment and run `robotpy sim` to run the simulator.

Here's what they are for:

* `.github/` - GitHub-specific files, such as for setting up automated testing.
* `.venv/` - The Python virtual environment created by `uv sync`; contains all the installed libraries and Python interpreter.
* `.vscode/` - VS Code-specific settings and configurations, so that everyone working on the code can have a consistent experience.
* `autonomous/` - **ROBOT CODE**: our autonomous routines go here.
* `components/` - **ROBOT CODE**: our robot components (subsystems) go here.
* `ctre_sim/` - _simulation spam_
* `.gitignore` - Lists files and directories that Git should ignore (not track).
* `constants.py` - **ROBOT CODE**: constants used throughout the robot code go here.
* `networktables.json` - _simulation spam_
* `pyproject.toml` - Configuration file for `uv` and `robotpy`; what libraries to install.
* `README.md` - This file!
* `robot.py` - **ROBOT CODE**: main robot code goes here.
* `simgui*` - _simulation spam_
* `uv.lock` - file created by `uv` to ensure consistent library versions.
