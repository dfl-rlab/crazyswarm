import numpy as np

from pycrazyswarm import Crazyswarm


Z = 0.8
TAKEOFF_DURATION = 2.5
GOTO_DURATION = 4.0
SQUARE_LENGTH = 0.5
WAYPOINTS = np.array([
    (SQUARE_LENGTH, 0.0, 0),
    (0, SQUARE_LENGTH, 0),
    (SQUARE_LENGTH * -1, 0, 0),
    (0.0, SQUARE_LENGTH * -1, 0),
])



def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    allcfs.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)
    
    for p in WAYPOINTS:
        allcfs.goTo(p, yaw=0.0, duration=GOTO_DURATION)
        timeHelper.sleep(GOTO_DURATION + 1.0)

    allcfs.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)


if __name__ == "__main__":
    main()