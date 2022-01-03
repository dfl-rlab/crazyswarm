import time
import logging

from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from pycrazyswarm import Crazyswarm
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0

uri = 'radio://0/80/2M/E7E7E7E703'


def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def main(scf, logconf):
    

    # swarm = Crazyswarm()
    # timeHelper = swarm.timeHelper
    
    # for cf in swarm.allcfs.crazyflies:
    #     cf.log.add_config(logconf)
    #     logconf.data_received_cb.add_callback(log_stab_callback)
    #     logconf.start()

    #     cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    #     timeHelper.sleep(TAKEOFF_DURATION)

    #     cf.land(targetHeight = 0.04, duration=2.5)
    #     timeHelper.sleep(TAKEOFF_DURATION)
    #     logconf.stop()

    # cf = swarm.allcfs.crazyflies[1]
    cf = scf.cf
    cf.log.add_config(logconf)

    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()

    cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    time.sleep(TAKEOFF_DURATION)

    cf.land(targetHeight = 0.04, duration=2.5)
    time.sleep(TAKEOFF_DURATION)
    logconf.stop()



if __name__ == '__main__':

    lg_stab = LogConfig(name='range', period_in_ms=10)
    # lg_stab.add_variable('range.front', 'float')
    # lg_stab.add_variable('range.back', 'float')
    # lg_stab.add_variable('range.up', 'float')
    # lg_stab.add_variable('range.left', 'float')
    # lg_stab.add_variable('range.right', 'float')
    lg_stab.add_variable('range.zrange', 'float')


    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        main(scf, lg_stab)