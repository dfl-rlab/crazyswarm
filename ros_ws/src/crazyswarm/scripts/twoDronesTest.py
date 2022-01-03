"""Flying two drones at the same time"""

import logging
import time

import cflib.crtp

from pycrazyswarm import Crazyswarm


from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger


TAKEOFF_DURATION = 2.5
HOVER_DURATION = 10.0


def simple_log(scf, logconf):

    with SyncLogger(scf, lg_stab) as logger:

        for log_entry in logger:

            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]
            print('[%d][%s]: %s' % (timestamp, logconf_name, data))

def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    for i in range(len(swarm.allcfs.crazyflies)):
        cf = swarm.allcfs.crazyflies[i]
        cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)

    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    for i in range(len(swarm.allcfs.crazyflies)):
        cf = swarm.allcfs.crazyflies[i]
        cf.land(targetHeight=0.04, duration=2.5)

    timeHelper.sleep(TAKEOFF_DURATION)



if __name__ == "__main__":
    main()



