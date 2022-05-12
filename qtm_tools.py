from typing import List

import qtm
import asyncio
import math
import xml.etree.cElementTree as ElementTree


class QTMBodyData:
    """ Class to handle QTM data """

    body_name: str
    timestamp: float
    valid_6dof: bool
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    qw: float
    qx: float
    qy: float
    qz: float

    def __init__(self):
        self.body_name = ' '
        self.timestamp = 0.0
        self.valid_6dof = False
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.qw = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0


async def connect_to_qtm(ip: str):
    connection: qtm.QRTConnection = await qtm.connect(ip)
    if connection is None:
        print(' ---- Warning ---- Error during QTM connection @', ip, '***')
    else:
        print('QTM connected @', ip)
    return connection


async def scan_qtm_bodies(connection: qtm.qrt.QRTConnection):
    result = await connection.get_parameters(parameters=['6d'])
    root = ElementTree.fromstring(result)
    bodies: List[QTMBodyData] = []
    bodies_names = []
    for label in root.iter('Name'):
        if label.text is not None:
            new_body = QTMBodyData()
            new_body.body_name = label.text
            bodies.append(new_body)
            bodies_names.append(new_body.body_name)
    print('QTM scan results :', len(bodies), 'bodies tracked by QTM')
    return bodies, bodies_names


def extract_packet_data(packet: qtm.packet.QRTPacket,
                        previous_bodies: List[QTMBodyData],
                        bodies_names: List[str]):
    """ Extracts the QTM bodies positions from a QRTPacket, and wraps them in a list of QTMBodyData objects,
      each object corresponding to a Crazyflie """

    timestamp = packet.timestamp
    header, data = packet.get_6d_euler()
    bodies_number = header.body_count
    bodies = []
    if header.body_count != bodies_number:
        print(' ---- Warning ---- Inconsistent results :')
        print('                   ->', bodies_number, 'bodies found using scan_qtm_bodies()')
        print('                   ->', header.body_count, 'body descriptions received on qtm stream packet')
        return None
    if bodies_number == 1:
        position = QTMBodyData()
        if not math.isnan(data[0][0]):
            position.body_name = bodies_names[0]
            position.valid_6dof = True
            position.timestamp = timestamp / 10**6
            position.x = data[0][0] / 1000
            position.y = data[0][1] / 1000
            position.z = data[0][2] / 1000
            position.roll = data[1][2]
            position.pitch = data[1][1]
            position.yaw = data[1][0]

            phi = math.pi * position.roll/180
            theta = math.pi * position.pitch/180
            psi = math.pi * position.yaw/180

            qw = math.cos(phi/2) * math.cos(theta/2) * math.cos(psi/2) \
                 - math.sin(phi/2) * math.sin(theta/2) * math.sin(psi/2)
            qx = math.sin(phi/2) * math.cos(theta/2) * math.cos(psi/2) \
                 + math.cos(phi/2) * math.sin(theta/2) * math.sin(psi/2)
            qy = math.sin(phi/2) * math.cos(theta/2) * math.sin(psi/2) \
                 - math.cos(phi/2) * math.sin(theta/2) * math.cos(psi/2)
            qz = math.cos(phi/2) * math.cos(theta/2) * math.sin(psi/2) \
                 + math.sin(phi/2) * math.sin(theta/2) * math.cos(psi/2)

            qn = math.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
            position.qw = qw/qn
            position.qx = qx/qn
            position.qy = qy/qn
            position.qz = qz/qn
        else:
            # (1)
            # When a body is not found by QTM, its transmitted coordinates are a list of NaN (Not a Number)
            # To prevent errors and exceptions, these NaN are replaced by the last valid coordinates of the
            # corresponding body. However, the valid_6dof attribute indicates that these coordinates are obsolete
            # and should not be used to control the UAV
            position.body_name = bodies_names[0]
            position.valid_6dof = False
            position.timestamp = previous_bodies[0].timestamp
            position.x = previous_bodies[0].x
            position.y = previous_bodies[0].y
            position.z = previous_bodies[0].z
            position.roll = previous_bodies[0].roll
            position.pitch = previous_bodies[0].pitch
            position.yaw = previous_bodies[0].yaw
            position.qw = previous_bodies[0].qw
            position.qx = previous_bodies[0].qx
            position.qy = previous_bodies[0].qy
            position.qz = previous_bodies[0].qz
        bodies.append(position)
    elif bodies_number > 1:
        for index in range(bodies_number):
            position = QTMBodyData()
            if not math.isnan(data[index][0][0]):
                position.body_name = bodies_names[index]
                position.valid_6dof = True
                position.timestamp = timestamp / 10**6
                position.x = data[index][0][0] / 1000
                position.y = data[index][0][1] / 1000
                position.z = data[index][0][2] / 1000
                position.roll = data[index][1][2]
                position.pitch = data[index][1][1]
                position.yaw = data[index][1][0]

                phi = math.pi * position.roll / 180
                theta = math.pi * position.pitch / 180
                psi = math.pi * position.yaw / 180

                qw = math.cos(phi / 2) * math.cos(theta / 2) * math.cos(psi / 2) \
                     - math.sin(phi / 2) * math.sin(theta / 2) * math.sin(psi / 2)
                qx = math.sin(phi / 2) * math.cos(theta / 2) * math.cos(psi / 2) \
                     + math.cos(phi / 2) * math.sin(theta / 2) * math.sin(psi / 2)
                qy = math.sin(phi / 2) * math.cos(theta / 2) * math.sin(psi / 2) \
                     - math.cos(phi / 2) * math.sin(theta / 2) * math.cos(psi / 2)
                qz = math.cos(phi / 2) * math.cos(theta / 2) * math.sin(psi / 2) \
                     + math.sin(phi / 2) * math.sin(theta / 2) * math.cos(psi / 2)

                qn = math.sqrt(qw ** 2 + qx ** 2 + qy ** 2 + qz ** 2)
                position.qw = qw / qn
                position.qx = qx / qn
                position.qy = qy / qn
                position.qz = qz / qn
            else:
                # See (1), line 113
                position.body_name = bodies_names[index]
                position.valid_6dof = False
                position.timestamp = previous_bodies[index].timestamp
                position.x = previous_bodies[index].x
                position.y = previous_bodies[index].y
                position.z = previous_bodies[index].z
                position.roll = previous_bodies[index].roll
                position.pitch = previous_bodies[index].pitch
                position.yaw = previous_bodies[index].yaw
                position.qw = previous_bodies[index].qw
                position.qx = previous_bodies[index].qx
                position.qy = previous_bodies[index].qy
                position.qz = previous_bodies[index].qz
            bodies.append(position)
    return bodies


def display_results(bodies: List[QTMBodyData]):
    body: QTMBodyData
    for body in bodies:
        if body.valid_6dof:
            print(body.body_name, 'found @ (', body.x, ';', body.y, ';', body.z, ')')
        else:
            print(body.body_name, 'not found in QTM-covered flight volume')


async def disconnect_qtm(connection: qtm.QRTConnection):
    if connection is not None:
        await connection.stream_frames_stop()
        connection.disconnect()
        print('QTM disconnected')
    else:
        print('QTM connection already closed')


if __name__ == '__main__':
    qtm_ip_address = '192.168.0.1'
    qtm_connection = asyncio.get_event_loop().run_until_complete(connect_to_qtm(qtm_ip_address))
    bodies_found, names = asyncio.get_event_loop().run_until_complete(scan_qtm_bodies(qtm_connection))
    frame = asyncio.get_event_loop().run_until_complete(qtm_connection.get_current_frame(components=['6deuler']))
    bodies_data = extract_packet_data(frame, bodies_found, names)
    display_results(bodies_data)
    asyncio.get_event_loop().run_until_complete(disconnect_qtm(qtm_connection))
