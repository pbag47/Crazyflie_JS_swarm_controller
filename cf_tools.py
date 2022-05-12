import time
import asyncio
import cflib.crtp
from cflib.crazyflie import Crazyflie
from typing import List

# noinspection PyProtectedMember
from cflib.crazyflie.log import LogConfig


def scan_available_cf(address_list: List[List[str]],
                      expected_cf_number: int = 0):
    cflib.crtp.init_drivers()
    radio_links = []
    available = []
    if expected_cf_number != 0:
        for try_number in range(3):
            available = []
            for address in address_list:
                result = cflib.crtp.scan_interfaces(int(address[2], 16))
                if result:
                    # (1)
                    # The scan_interfaces() method returns the uri of any available Crazyflie found, using the default
                    # radio number.
                    # To match the specified radio numbers in 'address_list', these uris have to be modified.
                    # An uri is built as follows :
                    # 'radio://' <radio number> '/' <channel number> '/' <emission rate> '/' <Crazyflie address>
                    #       -> Example : radio://0/80/2M/E7E7E7E701
                    # To change the radio number, the 8th character and its followers until '/' have to be modified.
                    i = result[0][0].find('/', 8)
                    result[0][0] = result[0][0][:8] + address[1] + result[0][0][i:]
                    available.append([address[0], result[0][0]])
            if len(available) == expected_cf_number:
                break
    else:
        for address in address_list:
            result = cflib.crtp.scan_interfaces(int(address[2], 16))
            if result:
                # See (1), line 22
                i = result[0][0].find('/', 8)
                result[0][0] = result[0][0][:8] + address[1] + result[0][0][i:]
                available.append([address[0], result[0][0]])
    print('cf radio scan results :', len(available), 'available Crazyflie found')
    return available, radio_links


def connect_to_cf(target_addresses: List[List[str]]):
    connected_cf = []
    connected_cf_addresses = []
    for address in target_addresses:
        cf_object = Crazyflie()
        cf_object.open_link(address[1])
        cf_object.commander.send_setpoint(0, 0, 0, 0)
        connected_cf.append(cf_object)
        connected_cf_addresses.append([address[0], address[1]])
    return connected_cf, connected_cf_addresses


def disconnect_cf(cf_target_list: List[Crazyflie]):
    for target in cf_target_list:
        target.commander.send_stop_setpoint()
        target.close_link()


def reset_kalman_estimation(cf_target, cf_name):
    stabilizer_estimator = 2
    flightmode_pos_set = 1
    pos_ctl_pid_thrust_base = 38000
    pos_ctl_pid_xkp = 1.2
    pos_ctl_pid_ykp = 1.2

    time.sleep(5)

    cf_target.param.set_value('stabilizer.estimator', str(stabilizer_estimator))
    cf_target.param.set_value('flightmode.posSet', str(flightmode_pos_set))
    print(cf_name, 'flight mode parameters updated')
    cf_target.param.set_value('kalman.resetEstimation', '1')
    print(cf_name, 'Kalman estimation reset')
    cf_target.param.set_value('posCtlPid.thrustBase', str(pos_ctl_pid_thrust_base))
    cf_target.param.set_value('posCtlPid.xKp', str(pos_ctl_pid_xkp))
    cf_target.param.set_value('posCtlPid.yKp', str(pos_ctl_pid_ykp))
    print(cf_name, 'PID parameters updated')
    return True


def start_log(cf_target: Crazyflie, logconfig: LogConfig):
    cf_target.log.add_config(logconfig)
    logconfig.data_received_cb.add_callback(print_log_results)
    logconfig.start()
    asyncio.sleep(30)
    logconfig.stop()
    
    
def print_log_results(timestamp, data, logconf):
    print('Log packet received :', timestamp, data, logconf)


if __name__ == '__main__':
    cf_addresses: List[List[str]] = [['cf1', '0', 'E7E7E7E701'],
                                     ['cf2', '0', 'E7E7E7E702'],
                                     ['cf3', '1', 'E7E7E7E703'],
                                     ['cf4', '1', 'E7E7E7E704']]

    cf_found, links = scan_available_cf(cf_addresses, 1)
    print(cf_found)
    cf_connected, addresses = connect_to_cf(cf_found)

    thrust_base = 40000
    print('Time started')
    start_time = time.time()
    while time.time() - start_time < 20:
        cf_connected[0].loc.send_extpos([0, 0, 0])
    A = cf_connected[0].param.get_value('posCtlPid.thrustBase')
    print(A)
    print('Time started')
    start_time = time.time()
    while time.time() - start_time < 2:
        cf_connected[0].loc.send_extpos([0, 0, 0])
    cf_connected[0].param.set_value('posCtlPid.thrustBase', str(thrust_base))
    print('Time started')
    start_time = time.time()
    while time.time() - start_time < 0.5:
        cf_connected[0].loc.send_extpos([0, 0, 0])
    A = cf_connected[0].param.get_value('posCtlPid.thrustBase')
    print(A)

    # for cf in cf_connected:
    #     cf.commander.send_setpoint(0, 0, 0, 10000)
    #     time.sleep(1)
    disconnect_cf(cf_connected)
