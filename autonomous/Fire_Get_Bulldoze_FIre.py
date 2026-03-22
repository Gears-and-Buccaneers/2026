from autonomous.choreo_auto import ChoreoAuto, ChoreoMultiTrajectoryAuto


class FireGetBulldozeFire(ChoreoMultiTrajectoryAuto):
    #     This auto mode:
    #     1. Drives from trench to neutral zone
    #     2. Intakes balls
    #     3. Bulldozes balls from neutral zone to right trench
    #     4. Shoots balls

    MODE_NAME = "Fire Get Bulldoze Fire"
    DISABLED = False  # Enable this auto mode
