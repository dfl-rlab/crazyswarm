
from pycrazyswarm import Crazyswarm


TAKEOFF_DURATION = 2.5
HOVER_DURATION = 10.0
MOVEMENT_DURATION = 5.0
WAIT_DURATION = 1.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    for cf in swarm.allcfs.crazyflies:
        cf.enableCollisionAvoidance
        cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)

    for cf in swarm.allcfs.crazyflies:
        
            cf.
    
    timeHelper.sleep(MOVEMENT_DURATION + WAIT_DURATION)


    for cf in swarm.allcfs.crazyflies:
       
        cf.land(targetHeight=0.04, duration=2.5)

    timeHelper.sleep(TAKEOFF_DURATION)



if __name__ == "__main__":
    main()