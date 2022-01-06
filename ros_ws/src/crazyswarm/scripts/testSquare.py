
from pycrazyswarm import Crazyswarm
import numpy as np
from optitrack_broadcast.msg import Mocap
import math
import rospy

HEIGHT = 1.4

SQUARE_LENGTH = 1.0

Initial_Centroid = 0.0

def find_Initial_centroid(cfList):
    global Initial_Centroid
    firstCF = cfList[0]
    firstPos = firstCF.position()
    centroid = firstPos + np.array([-(SQUARE_LENGTH / math.sqrt(2)), 0, 0])
    Initial_Centroid = [1.8, -4.6, HEIGHT]
    return centroid

def getFirstPos():
    position = Initial_Centroid + np.array([(SQUARE_LENGTH / math.sqrt(2)), 0, 0])
    
    return position


def getSecondPos():
    position = Initial_Centroid + np.array([0, SQUARE_LENGTH / math.sqrt(2), 0])
    
    return position


def getThirdPos():
    position = Initial_Centroid + np.array([-(SQUARE_LENGTH / math.sqrt(2)), 0, 0])
    
    return position


def getFourthPos():
    position = Initial_Centroid + np.array([0, -(SQUARE_LENGTH / math.sqrt(2)), 0])
    return position

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    firstCf = swarm.allcfs.crazyflies[0]
    firstCf.takeoff(targetHeight=HEIGHT, duration=2.0)
    timeHelper.sleep(3.0)
    find_Initial_centroid(allcfs.crazyflies)
    firstCf.goTo(goal=getFirstPos(), yaw=0.0, duration = 3.0)
    timeHelper.sleep(4.0)
    
    secondCF = allcfs.crazyflies[1]
    secondCF.enableCollisionAvoidance(others=[firstCf], ellipsoidRadii=np.array([0.3, 0.3, 1.0]))
    secondCF.takeoff(targetHeight=0.5, duration = 1.5)
    secondCF.enableCollisionAvoidance(others=[firstCf], ellipsoidRadii=np.array([0.3, 0.3, 1.0]))
    timeHelper.sleep(2.0)
    
    secondCF.goTo(goal=getSecondPos(), yaw=0.0, duration = 3.0)
    timeHelper.sleep(5.0)
    secondCF.disableCollisionAvoidance()
    thirdCF = allcfs.crazyflies[2]
    thirdCF.enableCollisionAvoidance(others=[firstCf, secondCF], ellipsoidRadii=np.array([0.3, 0.3, 1.0]))
    thirdCF.takeoff(targetHeight=0.5, duration = 1.5)
    timeHelper.sleep(2.0)
    print(getThirdPos())
    thirdCF.goTo(goal=getThirdPos(), yaw=0.0, duration = 3.0)
    timeHelper.sleep(5.0)
    thirdCF.disableCollisionAvoidance()

    fourthCF = allcfs.crazyflies[3]
    fourthCF.enableCollisionAvoidance(others=[firstCf, secondCF, thirdCF], ellipsoidRadii=np.array([0.3, 0.3, 1.0]))
    fourthCF.takeoff(targetHeight=0.5, duration = 1.5)
    timeHelper.sleep(2.0)
    print(getFourthPos())
    fourthCF.goTo(goal=getFourthPos(), yaw=0.0, duration = 3.0)
    timeHelper.sleep(8.0)

    allcfs.land(targetHeight=0.05, duration=2.0)
    timeHelper.sleep(3.0)