"""Single CF: takeoff, follow absolute-coords waypoints, land."""

import numpy as np

from pycrazyswarm import Crazyswarm


Z = 0.4
TAKEOFF_DURATION = 2.5
GOTO_DURATION = 4.0
SQUARE_LENGTH = 1.0
WAYPOINTS = np.array([
    (SQUARE_LENGTH, 0.0, Z),
    (SQUARE_LENGTH, SQUARE_LENGTH, Z),
    (0.0, SQUARE_LENGTH, Z),
    (0.0, 0.0, Z),
])


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    initial_pos = cf.position()
    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

    for p in WAYPOINTS:
        cf.goTo(initial_pos + p, yaw=0.0, duration=GOTO_DURATION)
        timeHelper.sleep(GOTO_DURATION + 1.0)

    cf.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)


if __name__ == "__main__":
    main()