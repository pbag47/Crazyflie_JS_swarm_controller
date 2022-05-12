import asyncio
import qtm
import qtm_tools
import cflib.crtp
import pynput
from typing import List, Union
from cflib.crazyflie import Crazyflie
from swarm_object_class import SwarmObject
from agent_class import Agent
from joystick_handler import Joystick


async def start_qtm_streaming(connection: qtm.QRTConnection):
    """ Starts a QTM stream, and assigns a callback method to run each time a QRTPacket is received from QTM
     This method is made to run forever in an asyncio event loop """
    print('QTM streaming started')
    await connection.stream_frames(components=['6deuler'], on_packet=packet_reception_callback)


def cf_connected_callback(link_uri):
    global SWARM_MANAGER

    for agt in SWARM_MANAGER.swarm_agent_list:
        if agt.cf.link_uri == link_uri:
            agt.setup_parameters()


def cf_disconnected_callback(link_uri):
    global SWARM_MANAGER

    for agt in SWARM_MANAGER.swarm_agent_list:
        if agt.cf.link_uri == link_uri:
            agt.enabled = False
            print(agt.name, 'disconnected')


async def keyboard_handler():
    global SWARM_MANAGER

    key_queue = detect_keyboard_input()
    while True:
        key = await key_queue.get()
        if key == pynput.keyboard.Key.esc:
            print('Ending program...')
            for agt in SWARM_MANAGER.swarm_agent_list:
                agt.cf.commander.send_stop_setpoint()
                agt.stop()
            asyncio.get_event_loop().stop()

        if key == pynput.keyboard.Key.ctrl:
            print('  -> Height consensus started')
            for agt in SWARM_MANAGER.swarm_agent_list:
                if agt.enabled:
                    agt.z_consensus()

        if key == pynput.keyboard.Key.shift:
            print('  -> Takeoff')
            for agt in SWARM_MANAGER.swarm_agent_list:
                if agt.enabled:
                    agt.takeoff()

        if key == pynput.keyboard.Key.down:
            print('  -> Land')
            for agt in SWARM_MANAGER.swarm_agent_list:
                if agt.enabled:
                    agt.land()

        if key == pynput.keyboard.Key.alt:
            print('  -> XY auto-avoidance mode')
            for agt in SWARM_MANAGER.swarm_agent_list:
                if agt.enabled:
                    agt.xy_auto_avoid()

        if key == pynput.keyboard.Key.ctrl_r:
            print('  -> Wingman mode')
            for agt in SWARM_MANAGER.swarm_agent_list:
                if agt.enabled:
                    agt.wingman_behaviour()


def detect_keyboard_input():
    queue = asyncio.Queue()
    loop = asyncio.get_event_loop()

    def on_press_callback(key):
        try:
            loop.call_soon_threadsafe(queue.put_nowait, key.char)
        except AttributeError:
            loop.call_soon_threadsafe(queue.put_nowait, key)

    pynput.keyboard.Listener(on_press=on_press_callback).start()
    return queue


def packet_reception_callback(packet: qtm.packet.QRTPacket):
    """ Callback : method called each time a packet is received from QTM """
    global QTM_BODIES
    global RUN_TRACKER
    global PACKET_COUNT
    global SWARM_MANAGER

    #         -------- Security checks                 --------         #

    # The packet reception callback may contain a lot of instructions, and its execution time might be a problem.
    # When the callback execution time is too long, it cannot be run until the end before a new packet arrives and
    # interrupts its execution, resulting in instruction losses and unpredictable behaviours.
    # To avoid this issue, a tracker is implemented to make sure that, each time a new packet is received, the previous
    # packet callback has been executed until the end. If not, the flight sequence is stopped for every UAV.
    # Most common reasons for this issue to happen :
    #       -> A silent error somewhere which blocks program execution without interrupting it
    #       -> Too much computational load
    #               \_ Try to optimize the program
    #               \_ Try to decrease the packet emission rate on QTM
    if not RUN_TRACKER:
        print(' ---- Warning ---- Callback execution interrupted by new QTM packet')
        print('                   -> There might be an error occurring during callback execution')
        print('                   -> Or, the sequence might require too much computing load')
        for agents_to_stop in SWARM_MANAGER.swarm_agent_list:
            agents_to_stop.stop()

    RUN_TRACKER = False

    #         -------- Data extraction and UAV control --------         #

    QTM_BODIES = qtm_tools.extract_packet_data(packet, QTM_BODIES, BODIES_NAMES)

    if PACKET_COUNT > -1:
        PACKET_COUNT = 0
        send_packet = True
    else:
        PACKET_COUNT = PACKET_COUNT + 1
        send_packet = False

    for agt in SWARM_MANAGER.swarm_agent_list:
        qtm_body_index = [body.body_name for body in QTM_BODIES].index(agt.name)
        agt.extpos = QTM_BODIES[qtm_body_index]
        agt.check_attitude()

        if agt.extpos.valid_6dof:
            agt.invalid_6dof_count = 0
            agt.send_external_position()
        elif agt.enabled:
            agt.invalid_6dof_count = agt.invalid_6dof_count + 1
            if agt.invalid_6dof_count > 10:
                print(agt.name, 'off camera for too long, switching the engines off')
                agt.stop()

    if send_packet:
        SWARM_MANAGER.flight_sequence()

    RUN_TRACKER = True


if __name__ == '__main__':
    #         -------- Flight parameters               --------         #

    # QTM server IP address
    qtm_ip_address: str = '192.168.0.1'

    # Association array between QTM body names and Crazyflies radio identification numbers
    # [[1st cf name on QTM, 1st cf uri, 1st cf takeoff height (m), 1st cf connectivity, 1st cf obstacles],
    #  [2nd cf name on QTM, 2nd cf uri, 2nd cf takeoff height (m), 2nd cf connectivity, 2nd cf obstacles],
    #                     :
    #  [nth cf name on QTM, nth cf uri, nth cf takeoff height (m), nth cf connectivity, nth cf obstacles]]
    cf_addresses: List[List[Union[str, float, List[str]]]] = [['cf1', 'radio://0/80/2M/E7E7E7E701', 0.5,
                                                               ['cf2', 'cf3', 'cf4'],
                                                               ['cf3']],
                                                              ['cf2', 'radio://1/85/2M/E7E7E7E702', 1.0,
                                                               ['cf1', 'cf3', "cf4"],
                                                               ['cf1', 'cf3', 'cf4']],
                                                              ['cf3', 'radio://0/90/2M/E7E7E7E703', 0.55,
                                                               ['cf1', 'cf2', 'cf4'],
                                                               ['cf1', 'cf2', 'cf4']],
                                                              ['cf4', 'radio://1/95/2M/E7E7E7E704', 0.60,
                                                               ['cf1', 'cf2', 'cf3'],
                                                               ['cf1', 'cf3']]]

    #         -------- QTM connection                  --------         #

    # Connects to QTM at provided IP address, and retrieves the tracked bodies names and their positions
    QTM_CONNECTION: qtm.QRTConnection = asyncio.get_event_loop().run_until_complete(
        qtm_tools.connect_to_qtm(qtm_ip_address))
    QTM_BODIES, BODIES_NAMES = asyncio.get_event_loop().run_until_complete(qtm_tools.scan_qtm_bodies(QTM_CONNECTION))
    print('Bodies tracked by QTM :', BODIES_NAMES)

    cflib.crtp.init_drivers()

    #         -------- Global variables initialization --------         #

    SWARM_MANAGER = SwarmObject()
    PACKET_COUNT = 0
    RUN_TRACKER = True

    js = Joystick(SWARM_MANAGER)

    #         -------- Crazyflies connection            --------         #
    for agent_specs in cf_addresses:
        agent = Agent(Crazyflie(), agent_specs[0])
        agent.cf.open_link(agent_specs[1])
        agent.set_takeoff_height(agent_specs[2])
        agent.set_z_consensus_connectivity(agent_specs[3])
        agent.set_xy_auto_avoid_obstacles(agent_specs[4])
        agent.cf.connected.add_callback(cf_connected_callback)
        agent.cf.disconnected.add_callback(cf_disconnected_callback)
        SWARM_MANAGER.add_agent(agent)

    SWARM_MANAGER.swarm_leader = 'cf3'
    SWARM_MANAGER.manual_flight_agents_list = ['cf3']

    #         -------- Main loop                        --------         #

    # Creates an event loop which runs the methods start_qtm_streaming() and keyboard_handler() as long as
    # the loop is not interrupted
    asyncio.ensure_future(start_qtm_streaming(QTM_CONNECTION))
    asyncio.ensure_future(keyboard_handler())
    asyncio.get_event_loop().run_forever()

    #         -------- End of the program               --------         #

    # Disconnects the Crazyflies, stops the QTM stream and disconnects from QTM
    for agent in SWARM_MANAGER.swarm_agent_list:
        agent.stop()
        agent.enabled = False
        agent.cf.close_link()
    asyncio.get_event_loop().run_until_complete(qtm_tools.disconnect_qtm(QTM_CONNECTION))
