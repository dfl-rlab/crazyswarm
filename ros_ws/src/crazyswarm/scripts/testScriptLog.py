
from pycrazyswarm import Crazyswarm
from crazyswarm.msg import GenericLogData
import numpy as np
import rospy

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 10.0

z_range = 0

POSITION_TO_KEEP = None

def range_callback(data):
    front_range = data.values[2] / 1000
    pos = cf.position()
    go = pos + np.array([0.0, 0, -0.1])
    print("up:  ", front_range )
    print("go:  ", go[2])
    print(cf.position())
    if front_range < 0.5:
        cf.cmdPosition(go, yaw=0.0)
    else:                                                                                                                     
        cf.cmdPosition(POSITION_TO_KEEP, yaw=0.0)
    
    # if z_range < 0.5:
    # #    cf.cmdVel(0, 0, 0, 1)

    #     cf.goTo(go, yaw=0.0, duration=1.0)
    

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
    # rospy.init_node('ayo', anonymous=True)
    # rospy.on_shutdown(land)
    while not rospy.is_shutdown():
        s = rospy.Subscriber("/cf5/log1", GenericLogData, range_callback)
    land()
    
    rospy.spin()
    # if not programEnd:
    #     rospy.spin()=
    # else:
    #     print("here")
    #     cf.land(targetHeight=0.05, duration=1.5)
    #     rospy.wait_for_service('/cf2/land')
    #     land = rospy.ServicePRoxy('/cf2/land', Land)
    #     land(0, 0.05, 2)
    
        

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]
    cf.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(3.0)
    POSITION_TO_KEEP = cf.position()
    

    listener(False)
    

    # listener(True)
    # swarm = Crazyswarm()
    # timeHelper = swarm.timeHelper
    # cf = swarm.allcfs.crazyflies[0]
    # try:
    #     listener(False)
    # except KeyboardInterrupt:
    #     listener(True)



    
    
    
    
    
    
    # cf.land(targetHeight=0.05, duration=1.5)
    # timeHelper.sleep(3.0)
    # print("here")

    # swarm = Crazyswarm()
    # timeHelper = swarm.timeHelper
    # cf = swarm.allcfs.crazyflies[0]
    # # rospy.wait_for_service('/cf2/land')
    # # land = rospy.ServicePRoxy('/cf2/land', Land)
    # # land(0, 0.05, 2)
    # cf.land(targetHeight=0.05, duration=1.5)
    # timeHelper.sleep(3.0)
