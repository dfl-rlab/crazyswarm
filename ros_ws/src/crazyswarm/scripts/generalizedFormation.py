
from pycrazyswarm import Crazyswarm
import numpy as np
from optitrack_broadcast.msg import Mocap
import math

import rospy

#Static variables

CIRCLE_RADIUS = 1.0

HEIGHT = 1.5

INITIAL_CENTROID = [1.8, -4.6, HEIGHT]

INITIAL_YAW = 0.0    #Angle measured in radians that goes clockwise from x-axis.

DEFAULT_ELLIPSOID = [0.3, 0.3, 0.8]

NUMBER_OF_DRONES = 1  #Must be an integer


# Variables that the program will change - DO NOT CHANGE THESE YOURSELF

ANGLE_PER_DRONE = 2 * math.pi

CURR_CENTROID = 0.0

CURR_YAW = 0.0


def getDronePosition(id):
    ANGLE = (ANGLE_PER_DRONE * (id - 1)) + INITIAL_YAW
    Pos_x = CIRCLE_RADIUS * math.cos(ANGLE)
    Pos_y = CIRCLE_RADIUS * math.sin(ANGLE)
    Position_to_add = np.array([Pos_x, Pos_y, 0])
    Position = INITIAL_CENTROID + Position_to_add
    return Position

def main():
    global NUMBER_OF_DRONES
    global ANGLE_PER_DRONE
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    
    Drone_list = swarm.allcfs.crazyflies
    allcfs = swarm.allcfs
    
    NUMBER_OF_DRONES = len(Drone_list)

    ANGLE_PER_DRONE = (2 * math.pi) / NUMBER_OF_DRONES 

    for i, cf in enumerate(Drone_list):
        
        cf.enableCollisionAvoidance(others=Drone_list[:i], ellipsoidRadii=DEFAULT_ELLIPSOID)
        cf.takeoff(targetHeight=0.4, duration=1.0)
        timeHelper.sleep(2.0)
        
        goToPos = getDronePosition(i + 1)
        cf.goTo(goal=goToPos, yaw=0.0, duration = 3.0)
        timeHelper.sleep(4.0)
        cf.disableCollisionAvoidance()

    timeHelper.sleep(5.0)
    allcfs.land(targetHeight=0.05, duration=2.0)
    timeHelper.sleep(3.0)


if __name__ == "__main__":
    main()


        



