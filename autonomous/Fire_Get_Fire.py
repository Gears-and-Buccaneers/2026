from autonomous.choreo_auto import ChoreoAuto, ChoreoMultiTrajectoryAuto


class FireGetFire(ChoreoMultiTrajectoryAuto):
    #     This auto mode:
    #     1. Drives from trench to neutral zone
    #     2. Intakes balls
    #     3. Drives to left trench
    #     4. Shoots balls
    #
    MODE_NAME = "Fire Get Fire"
    DISABLED = False  # Enable this auto mode
