import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M/E7E7E7E702'
is_deck_attached = False

logging.basicConfig(level=logging.ERROR)

def log_pos_callback(timestamp, data, logconf):
    print(data)

def param_deck_flow(name, value_str):
    value = int(value_str)
    print(value)
    global is_deck_attached
    if value:
        is_deck_attached = True
        print('Deck is attached!')
    else:
        is_deck_attached = False
        print('Deck is NOT attached!')


def take_off_simple(scf):
    with MotionCommander(scf) as mc:
        mc.up(0.3)
        time.sleep(3)


if __name__ == '__main__':

    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.param.add_update_callback(group="deck", name="bcFlow2",
                                cb=param_deck_flow)
        
        # logconf = LogConfig(name='range', period_in_ms=10)
        # logconf.add_variable('range.zrange', 'float')

        logconf = LogConfig(name=)
        cf = scf.cf
        cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)


        if is_deck_attached:
            logconf.start()
            
            take_off_simple(scf)

            logconf.stop()
