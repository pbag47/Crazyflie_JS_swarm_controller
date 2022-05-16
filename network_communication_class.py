import socket
import threading
import time

from typing import Tuple, List


class NetworkCommunication:
    def __init__(self, this_pc: Tuple[str, int], other_pcs: List[Tuple[str, int]]):
        self.this_PC = this_pc
        self.other_PCs = other_pcs
        self.sender_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        self.receiver_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        self.receiver_socket.bind(self.this_PC)

    def send_packet(self, data: str):
        for pc in self.other_PCs:
            self.sender_socket.sendto(data.encode('utf-8'), pc)
        print('Packet sent :', data)

    def disconnect(self):
        data = 'Disconnecting...'
        for pc in self.other_PCs:
            self.sender_socket.sendto(data.encode('utf-8'), pc)
        self.sender_socket.close()
        self.receiver_socket.close()


if __name__ == '__main__':
    ip_address = '192.168.0.102'
    port_number = 4444
    pkh = NetworkCommunication((ip_address, port_number), [('192.168.0.100', 4444)])
    time.sleep(30)
    pkh.disconnect()
