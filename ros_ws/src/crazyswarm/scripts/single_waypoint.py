"""Single CF: takeoff, follow absolute-coords waypoints, land."""

import numpy as np
from pycrazyswarm import Crazyswarm

Z = 0.5
TAKEOFF_DURATION = 2.5
GOTO_DURATION = 3.0
WAYPOINTS = np.array([
    (0,0, Z),
])


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)
    print("Initial Position", cf.position())
    for p in WAYPOINTS:
        print("Go to: ", cf.position() + p)
        cf.goTo(cf.position() + p, yaw=0.0, duration=GOTO_DURATION)
        timeHelper.sleep(GOTO_DURATION + 1.0)
        

    cf.land(targetHeight=0.1, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

if __name__ == "__main__":
    main()
