
"""Takeoff-hover-land for all CFs. Useful to validate hardware config."""

from pycrazyswarm import Crazyswarm


TAKEOFF_DURATION = 2.5
HOVER_DURATION = 3.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    for i in range(len(swarm.allcfs.crazyflies)):
        cf = swarm.allcfs.crazyflies[i]

        cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
        timeElapsed = 0
        while timeElapsed < TAKEOFF_DURATION + HOVER_DURATION:
            print(cf.position())
            timeHelper.sleep(0.5)
            timeElapsed += 0.5
        # print(cf.getParam(range.zrange))
        cf.land(targetHeight=0.04, duration=2.5)
        timeHelper.sleep(TAKEOFF_DURATION + 1.0)


if __name__ == "__main__":
    main()
