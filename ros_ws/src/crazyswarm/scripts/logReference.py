'''
AUTHOR: 
Kaushik Balasundar (kbalasun@andrew.cmu.edu)
DEFINITION: 
Function to get End-effector pose and linear velocity from optitrack
DEPENDENCIES: 
1. vrpn-client-ros: https://github.com/ros-drivers/vrpn_client_ros.git
2. optitrack_broadcast: https://github.com/LonghaoQian/optitrack_broadcast 
'''

from pycrazyswarm import Crazyswarm
import rospy 
from optitrack_broadcast.msg import Mocap

#Function that gets the end effector's pose
def getEEpos(data):
    #Initializing node  
    # rospy.init_node('EE_PoseVel_listener', anonymous=True)

    #Extracting position 
    eePos = data.position

    #Extracting orientation 
    eeOri = data.quaternion

    #Adding to pose list 
    eePose = [eePos, eeOri]
    print("Pose[(x,y,z), (x,y,z,w)]: ", eePose)
    
    #Returning pose list 
    return eePose
    



def listener(marker="reference_body"):
    # rospy.init_node('EE_PoseVel_listener', anonymous=True)

    #Extracting position 
    rospy.Subscriber("/mocap/" + str(marker), Mocap, getEEpos)
    
    rospy.spin()
    #Extracting orientation 
    



#Function that gets th eend
def getEEvel(marker="reference_body"):
    #Extracting linear velocity 
    eeVel = rospy.wait_for_message("/mocap/" + str(marker), Mocap).velocity
    print("Linear Velocity: ", [eeVel])

    #Returning linear velocity
    return eeVel

def getTargetPose(marker="reference_body"):
       
    #Call EE pose to get target marker position 
    targetPose = getEEpos(marker)

    #Print and return the target pose 
    print("Target Pose [(x,y,z), (x,y,z,w)]:", targetPose)
    return targetPose


if __name__ == "__main__":
    print("/mocap/" + str("reference_body"))
    swarm = Crazyswarm()
    listener("reference_body")