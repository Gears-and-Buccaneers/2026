"""All the auto modes.

IMPORTANT: For MagicBot to discover auto modes, they must be imported here.
MagicBot scans this package and looks for AutonomousStateMachine subclasses.
"""

# Import all autonomous mode classes so MagicBot can discover them
from autonomous.choreo_examples import SimpleChoreoAuto
from autonomous.just_leave import JustLeavePlease

# Optionally list them for explicit exports
__all__ = [
    "JustLeavePlease",
    "SimpleChoreoAuto",
]
