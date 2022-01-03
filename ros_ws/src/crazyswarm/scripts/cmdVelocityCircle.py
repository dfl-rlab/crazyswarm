#!/usr/bin/env python

import numpy as np
    from pycrazyswarm import *


Z = 1.0
sleepRate = 30
SCRIPT_DURATION = 10.0

def goCircle(timeHelper, cf, totalTime, radius, kPosition):
        startTime = timeHelper.time()
        
        startPos = cf.position()
        center_circle = startPos - np.array([radius, 0, 0])

        while timeHelper.time() < startTime + SCRIPT_DURATION:
            time = timeHelper.time() - startTime
            omega = 2 * np.pi / totalTime
            vx = -radius * omega * np.sin(omega * time)  
            vy = radius * omega * np.cos(omega * time)
            desiredPos = center_circle + radius * np.array(
                [np.cos(omega * time), np.sin(omega * time), 0])
            errorX = desiredPos - cf.position() 
            cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * errorX), yawRate=0)
            timeHelper.sleepForRate(sleepRate)
        cf.cmdVelocityWorld(np.array([0, 0, 0],yawRate = 0))
        timeHelper.sleep(Duration = 3.0)
        cf.land(targetHeight=0.04, duration=2.5)
        timeHelper.sleep(Duration=2.5)

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    cf = allcfs.crazyflies[0]
    cf.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(2 + Z)
    goCircle(timeHelper, cf, totalTime=6.0, radius=0.5, kPosition=1)
