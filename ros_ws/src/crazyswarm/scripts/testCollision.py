
from pycrazyswarm import Crazyswarm


TAKEOFF_DURATION = 2.0

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    firstCF = swarm.allcfs.crazyflies[0]
    secondCF = swarm.allcfs.crazyflies[1]
    firstCF.enableCollisionAvoidance(swarm.allcfs.crazyflies, [0.4, 0.4, 0.8])


    for cf in swarm.allcfs.crazyflies:
        cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION)

    secondPos = secondCF.position()
    
    distance_vector = secondPos - firstCF.position()
    
    firstCF.goTo(goal=(distance_vector * 1.8) + firstCF.position(), yaw=0, duration=10.0)
    timeHelper.sleep(10.0)
    for cf in swarm.allcfs.crazyflies:
        try: 
            cf.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
        except:
            print("cf x crashed")
    timeHelper.sleep(TAKEOFF_DURATION)


if __name__ == "__main__":
    main()
