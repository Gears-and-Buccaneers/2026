from autonomous.choreo_auto import ChoreoAuto, ChoreoMultiTrajectoryAuto


class FireDepotFire(ChoreoMultiTrajectoryAuto):
    #     This auto mode:
    #     1. Drives from trench to depot
    #     2. Intakes balls
    #     3. Drives to corner
    #     4. Shoots balls
    #
    MODE_NAME = "Fire Depot Fire"
    DISABLED = False  # Enable this auto mode
