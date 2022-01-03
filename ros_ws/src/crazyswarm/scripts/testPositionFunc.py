"""Test position() for single crazyflie (cf) drone"""

from pycrazyswarm import Crazyswarm


TAKEOFF_DURATION = 2.5
HOVER_DURATION = 30.0

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    timeElapsed = 0.0
    while timeElapsed < TAKEOFF_DURATION + HOVER_DURATION:
        print(cf.position())
        timeHelper.sleep(0.5)
        timeElapsed += 0.5


if __name__ == "__main__":
    main()
