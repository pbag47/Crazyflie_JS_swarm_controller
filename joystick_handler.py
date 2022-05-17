import array
import asyncio
import os
import struct
import threading
from fcntl import ioctl
from typing import Union
from swarm_object_class import SwarmObject
from network_communication_class import NetworkCommunication


class Joystick:
    def __init__(self, swarm_object: Union[SwarmObject, None] = None,
                 network_communication: NetworkCommunication = None, js_connected: bool = True):
        self.queue = asyncio.Queue()
        self.loop = asyncio.get_event_loop()
        self.swarm = swarm_object
        self.network_comm = network_communication
        self.joystick_connected = js_connected
        self.stopped = False

        if not self.joystick_connected:
            self.packet_rec_thread = threading.Thread(target=self.await_packet)
        else:
            print('Joystick init:')
            print('    Available devices:', [('/dev/input/%s' % fn)
                                                 for fn in os.listdir('/dev/input') if fn.startswith('js')])

            # We will store the states here.
            self.axis_states = {}
            self.button_states = {}

            # These constants were borrowed from linux/input.h
            self.axis_names = {
                    0x00: 'roll',
                    0x01: 'pitch',
                    0x02: 'height',
                    0x03: 'rx',
                    0x04: 'ry',
                    0x05: 'yaw',
                    0x06: 'throttle',
                    0x07: 'rudder',
                    0x08: 'wheel',
                    0x09: 'gas',
                    0x0a: 'brake',
                    0x10: 'hat0x',
                    0x11: 'hat0y',
                    0x12: 'hat1x',
                    0x13: 'hat1y',
                    0x14: 'hat2x',
                    0x15: 'hat2y',
                    0x16: 'hat3x',
                    0x17: 'hat3y',
                    0x18: 'pressure',
                    0x19: 'distance',
                    0x1a: 'tilt_x',
                    0x1b: 'tilt_y',
                    0x1c: 'tool_width',
                    0x20: 'volume',
                    0x21: 'misc',
                }

            self.button_names = {
                    0x120: 'trigger',
                    0x121: '2',
                    0x122: '3',
                    0x123: '4',
                    0x124: '5',
                    0x125: '6',
                    0x126: 'base',
                    0x127: 'base2',
                    0x128: 'base3',
                    0x129: 'base4',
                    0x12a: '11',
                    0x12b: '12',
                    0x12f: 'dead',
                    0x130: 'a',
                    0x131: 'b',
                    0x132: 'c',
                    0x133: 'x',
                    0x134: 'y',
                    0x135: 'z',
                    0x136: 'tl',
                    0x137: 'tr',
                    0x138: 'tl2',
                    0x139: 'tr2',
                    0x13a: 'select',
                    0x13b: 'start',
                    0x13c: 'mode',
                    0x13d: 'thumb_left',
                    0x13e: 'thumb_right',

                    0x220: 'dpad_up',
                    0x221: 'dpad_down',
                    0x222: 'dpad_left',
                    0x223: 'dpad_right',

                    # xBox 360 controller uses these codes.
                    0x2c0: 'dpad_left',
                    0x2c1: 'dpad_right',
                    0x2c2: 'dpad_up',
                    0x2c3: 'dpad_down'
                }

            self.axis_map = []
            self.button_map = []

            # Open the Joystick device
            self.fn = '/dev/input/js0'
            print('    Opening %s...' % self.fn)
            self.js_device = open(self.fn, 'rb')

            # Get the device name.
            buf = array.array('B', [0] * 64)
            ioctl(self.js_device, 0x80006a13 + (0x10000 * len(buf)), buf)
            self.js_name = buf.tostring().rstrip(b'\x00').decode('utf-8')
            print('        Device name: %s' % self.js_name)

            # Get number of axes and buttons.
            buf = array.array('B', [0])
            ioctl(self.js_device, 0x80016a11, buf)
            self.num_axes = buf[0]

            buf = array.array('B', [0])
            ioctl(self.js_device, 0x80016a12, buf)
            self.num_buttons = buf[0]
            # print('        %s axes and' % self.num_axes, self.num_buttons, 'buttons found')

            # Get the axis map.
            buf = array.array('B', [0] * 0x40)
            ioctl(self.js_device, 0x80406a32, buf)
            for axis in buf[:self.num_axes]:
                axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
                self.axis_map.append(axis_name)
                self.axis_states[axis_name] = 0.0

            # Get the button map.
            buf = array.array('H', [0] * 200)
            ioctl(self.js_device, 0x80406a34, buf)
            for btn in buf[:self.num_buttons]:
                btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
                self.button_map.append(btn_name)
                self.button_states[btn_name] = 0.0

            print('        %d axes found: %s' % (self.num_axes, ', '.join(self.axis_map)))
            print('        %d buttons found: %s' % (self.num_buttons, ', '.join(self.button_map)))

            if self.swarm is not None:
                self.jsl = threading.Thread(target=self.joystick_inputs)
                self.jsl.start()

    def await_packet(self):
        while not self.stopped:
            packet, address = self.network_comm.receiver_socket.recvfrom(4096)
            data = packet.decode('utf-8').split()
            sender_ip = data[0]
            value = float(data[1])
            command_type = data[2]
            if command_type == 'button':
                button = data[3]
                if button:
                    self.buttons(button, value)
            if command_type == 'axis':
                axis = data[3]
                if axis:
                    self.axis(axis, value)

    def joystick_inputs(self):
        while not self.stopped:
            ev_buf = self.js_device.read(8)
            time, value, buf_type, number = struct.unpack('IhBB', ev_buf)

            if buf_type & 0x01:
                button = self.button_map[number]
                if button:
                    self.network_comm.send_packet(str(value) + ' ' + 'button' + ' ' + str(button))
                    self.buttons(button, value)

            if buf_type & 0x02:
                axis = self.axis_map[number]
                if axis:
                    self.network_comm.send_packet(str(value) + ' ' + 'axis' + ' ' + str(axis))
                    self.axis(axis, value)

    def buttons(self, button, value):
        if button == 'trigger' and value:
            print(' ---- Warning ---- Emergency stop triggered / Joystick disconnected')
            for agt in self.swarm.swarm_agent_list:
                agt.cf.commander.send_stop_setpoint()
                agt.stop()
            self.stopped = True

        if button == '2' and value:
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and not agt.is_flying:
                    print(agt.name, 'takeoff')
                    agt.takeoff()
                elif agt.enabled and agt.is_flying:
                    print(agt.name, 'landing')
                    agt.land()

        if button == '3' and value:
            print('Standby')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and agt.is_flying:
                    agt.standby()

        if button == '4' and value:
            print('Manual flight activated')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and agt.is_flying and any([agt.name == manual for manual in
                                                          self.swarm.manual_flight_agents_list]):
                    agt.manual_flight()

        if button == '5' and value:
            self.swarm.manual_yaw = self.swarm.manual_yaw + 22.5

        if button == '6' and value:
            self.swarm.manual_yaw = self.swarm.manual_yaw - 22.5

        if button == '11' and value:
            print('Wingman')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and agt.is_flying and not agt.state == 'Manual':
                    agt.wingman_behaviour()

        if button == '12' and value:
            print('xy automatic avoidance mode')
            for agt in self.swarm.swarm_agent_list:
                if agt.enabled and agt.is_flying and not agt.state == 'Manual':
                    agt.xy_auto_avoid()

    def axis(self, axis, value):
        fvalue = value / 32767.0

        if axis == 'pitch':
            if -0.05 < fvalue < 0.05:
                self.swarm.manual_x = 0.0
            else:
                self.swarm.manual_x = fvalue

        if axis == 'roll':
            if -0.05 < fvalue < 0.05:
                self.swarm.manual_y = 0.0
            else:
                self.swarm.manual_y = fvalue

        if axis == 'height':
            self.swarm.manual_z = fvalue


if __name__ == '__main__':
    js = Joystick()
    swarm = SwarmObject()
    asyncio.get_event_loop().run_forever()
