

from pycrazyswarm import Crazyswarm
import numpy as np


TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
MOVE_DURATION = 2.5


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    # print(cf.getParam(range.zrange))
    current_pos = cf.position()
    print(current_pos)
    current_pos[0] = current_pos[0] - 0.3
    print(current_pos)
    cf.goTo(current_pos, 0, MOVE_DURATION)
    timeHelper.sleep(MOVE_DURATION)

    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
