from typing import List, Union
from qtm_tools import QTMBodyData
import cflib.crazyflie
import time


class Agent:
    def __init__(self, crazyflie, qtm_body_name):
        self.cf: cflib.crazyflie.Crazyflie = crazyflie
        self.name: str = qtm_body_name
        self.state: str = 'Not flying'
        self.is_flying: bool = False
        self.send_packet: bool = False
        self.waypoints: List[List[Union[None, float]]] = [[None, None, None, None],
                                                          [None, None, 0, None]]
        self.waypoint_number: int = 0
        self.on_waypoint: bool = False
        self.enabled: bool = False
        self.flight_completed: bool = False
        self.takeoff_position: List[float] = []
        self.takeoff_yaw: float = 0.0
        self.takeoff_height: float = 1.0
        self.land_position: List[float] = []
        self.land_yaw: float = 0.0
        self.standby_position: List[float] = []
        self.standby_yaw: float = 0.0
        self.z_consensus_xy_position: List[float] = []
        self.z_consensus_vz: float = 0.0
        self.z_consensus_connectivity: List[str] = []
        self.xy_consensus_height: float = 0.0
        self.xy_consensus_yaw: float = 0.0
        self.consensus_yaw: float = 0.0
        self.xy_auto_avoid_xyz_position: List[float] = []
        self.xy_auto_avoid_yaw: float = 0.0
        self.xy_auto_avoid_obstacles_list: List[str] = []
        self.wingman_z: float = 0
        self.wingman_yaw: float = 0

        self.stabilizer_estimator = 2
        self.flightmode_pos_set = 1
        self.quaternion_std_dev = 0.015
        self.pos_ctl_pid_thrust_base = 38000
        self.pos_ctl_pid_xkp = 1.2
        self.pos_ctl_pid_ykp = 1.2
        self.pos_ctl_pid_rp_limit = 10

        self.max_roll: float = 45  # (°)
        self.max_pitch: float = 45  # (°)
        self.x_boundaries: List[float] = [-2, 2]  # (m)
        self.y_boundaries: List[float] = [-2, 2]  # (m)
        self.z_boundaries: List[float] = [-0.1, 1.50]  # (m)

        self.extpos: QTMBodyData = QTMBodyData()
        self.invalid_6dof_count: int = 0

    def stop(self):
        self.state = 'Not flying'
        self.is_flying = False
        self.enabled = False
        self.flight_completed = True

    def setup_parameters(self):
        time.sleep(2)
        self.cf.param.set_value('stabilizer.estimator', str(self.stabilizer_estimator))
        self.cf.param.set_value('flightmode.posSet', str(self.flightmode_pos_set))
        self.cf.param.set_value('locSrv.extQuatStdDev', str(self.quaternion_std_dev))
        self.cf.param.set_value('posCtlPid.thrustBase', str(self.pos_ctl_pid_thrust_base))
        self.cf.param.set_value('posCtlPid.xKp', str(self.pos_ctl_pid_xkp))
        self.cf.param.set_value('posCtlPid.yKp', str(self.pos_ctl_pid_ykp))
        self.cf.param.set_value('posCtlPid.rpLimit', str(self.pos_ctl_pid_rp_limit))
        print(self.name, ': parameters updated')
        self.enabled = True

    def check_attitude(self):
        if self.enabled and (abs(self.extpos.roll) > self.max_roll or abs(self.extpos.pitch) > self.max_pitch
                             or self.extpos.x < self.x_boundaries[0] or self.extpos.x > self.x_boundaries[1]
                             or self.extpos.y < self.y_boundaries[0] or self.extpos.y > self.y_boundaries[1]
                             or self.extpos.z < self.z_boundaries[0] or self.extpos.z > self.z_boundaries[1]):
            print(self.name, '---- Warning ---- Abnormal attitude detected')
            print(self.name, 'attitude : {Roll =', round(self.extpos.roll), '° ; Pitch =', round(self.extpos.pitch),
                  '° ; x =', round(self.extpos.x, 2), 'm ; y =', round(self.extpos.y, 2), 'm ; z =',
                  round(self.extpos.z, 2), 'm}')
            print(self.name, 'nominal attitude boundaries : {|Roll| <', self.max_roll, '° ; |Pitch| <',
                  self.max_pitch,
                  '°, x in', self.x_boundaries, 'm, y in', self.y_boundaries, 'm, z in', self.z_boundaries, 'm}')
            self.stop()

    def send_external_position(self):
        self.cf.extpos.send_extpose(self.extpos.x, self.extpos.y, self.extpos.z,
                                    self.extpos.qx, self.extpos.qy, self.extpos.qz, self.extpos.qw)

    def set_takeoff_height(self, height: float):
        self.takeoff_height = height

    def set_waypoints(self, waypoints: List[List[Union[None, float]]]):
        self.waypoints = waypoints

    def set_z_consensus_connectivity(self, connections: List[str]):
        self.z_consensus_connectivity = connections

    def set_xy_auto_avoid_obstacles(self, obstacles: List[str]):
        self.xy_auto_avoid_obstacles_list = obstacles

    def wingman_behaviour(self):
        self.state = 'Wingman'
        self.wingman_z = self.extpos.z
        self.wingman_yaw = self.extpos.yaw

    def standby(self):
        self.state = 'Standby'
        self.standby_position = [self.extpos.x, self.extpos.y, self.extpos.z]
        self.standby_yaw = self.extpos.yaw

    def takeoff(self):
        self.state = 'Takeoff'
        self.is_flying = True
        self.takeoff_position = [self.extpos.x, self.extpos.y, self.takeoff_height]
        self.takeoff_yaw = self.extpos.yaw

    def land(self):
        self.state = 'Land'
        self.land_position = [self.extpos.x, self.extpos.y, 0.0]
        self.land_yaw = self.extpos.yaw

    def manual_flight(self):
        self.state = 'Manual'
        self.standby_position = [self.extpos.x, self.extpos.y, self.extpos.z]
        self.standby_yaw = self.extpos.yaw

    def consensus(self):
        self.state = 'Consensus'

    def xy_consensus(self, height: float):
        self.state = 'xy_consensus'
        self.xy_consensus_height = height
        self.xy_consensus_yaw = self.extpos.yaw

    def z_consensus(self):
        self.state = 'z_consensus'
        self.z_consensus_xy_position = [self.extpos.x, self.extpos.y]

    def xy_auto_avoid(self):
        self.state = 'xy_auto_avoid'
        self.xy_auto_avoid_xyz_position = [self.extpos.x, self.extpos.y, self.extpos.z]
        self.xy_auto_avoid_yaw = self.extpos.yaw

    def waypoint_sequence(self, init_waypoint_number: int = 0):
        self.state = 'Waypoints'
        self.waypoint_number = init_waypoint_number
