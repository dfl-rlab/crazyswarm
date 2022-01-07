
from pycrazyswarm import Crazyswarm
from crazyswarm.msg import GenericLogData
import numpy as np
import rospy

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 10.0

z_range = 0

POSITION_TO_KEEP = None

def range_callback(data):
    up_range = data.values[2] / 1000
    pos = cf.position()
    go = pos + np.array([0.0, 0, -0.1])
    print("up:  ", up_range )
    print("go:  ", go[2])
    print(cf.position())
    if front_range < 0.5:
        cf.cmdPosition(go, yaw=0.0)
    else:                                                                                                                     
        cf.cmdPosition(POSITION_TO_KEEP, yaw=0.0)
    

def land():
    timeHelper.sleep(1.0)
    print("Switching to high level")
    timeHelper.sleep(3.0)
    cf.notifySetpointsStop()
    print("switched")
    cf.land(targetHeight=0.05, duration=2.0)
    timeHelper.sleep(3.0)
    print("Successfuly landed!")
    




def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    # print(cf.getParam(range.zrange))
    cf.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(TAKEOFF_DURATION)



def listener(programEnd):
    rospy.on_shutdown(land)
    s = rospy.Subscriber("/cf5/log1", GenericLogData, range_callback)
    rospy.spin()
    
    
        

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    cf.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(3.0)
    POSITION_TO_KEEP = cf.position()

    listener(False)
    