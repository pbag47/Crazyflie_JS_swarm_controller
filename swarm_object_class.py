from typing import List, Union
import numpy as np
from agent_class import Agent


class SwarmObject:
    def __init__(self):
        self.distance_to_waypoint_threshold: float = 0.1  # (m)
        self.yaw_error_tolerance: float = 10  # (°)

        self.manual_x: float = 0.0
        self.manual_y: float = 0.0
        self.manual_z: float = 0.0
        self.manual_yaw: float = 0.0
        self.manual_flight_agents_list: List[str] = []

        self.swarm_leader: Union[None, Agent] = None
        self.swarm_agent_list: List[Agent] = []
        self.agent_name_list: List[str] = []
        self.waypoint_number: Union[None, int] = None

        self.x_limits: List[float] = [-1.5, 1.5]
        self.y_limits: List[float] = [-1.5, 1.5]
        self.z_limits: List[float] = [0, 1.5]

        self.xy_auto_avoid_omega: float = np.pi / (2 * horizontal_distance([self.x_limits[0], self.y_limits[0]],
                                                                           [self.x_limits[1], self.y_limits[1]]))
        self.xy_auto_avoid_d0: float = 1.25  # Obstacle detection distance (m)

    def add_agent(self, agent: Agent):
        self.swarm_agent_list.append(agent)
        self.agent_name_list.append(agent.name)

    def assign_swarm_leader(self, cf_name: str):
        self.swarm_leader = cf_name

    def set_wingman_behaviour(self, wingmen_list: List[Agent]):
        if self.swarm_leader is not None:
            for agent in wingmen_list:
                agent.wingman_behaviour()
        else:
            print(' ---- Warning : No swarm leader assigned, unable to configure wingman behaviour')

    def flight_sequence(self):
        for agent in self.swarm_agent_list:
            if agent.enabled and agent.state != 'Not flying':
                if agent.state == 'Standby':
                    self.standby_control_law(agent)

                if agent.state == 'Takeoff':
                    self.takeoff_control_law(agent)

                if agent.state == 'Land':
                    self.landing_control_law(agent)

                if agent.state == 'Manual':
                    self.manual_control_law(agent)

                if agent.state == 'z_consensus':
                    self.z_consensus_control_law(agent)

                if agent.state == 'xy_auto_avoid':
                    self.xy_auto_avoid_control_law(agent)

                if agent.state == 'Wingman':
                    self.wingman_control_law(agent)
            else:
                agent.cf.commander.send_stop_setpoint()

    @staticmethod
    def standby_control_law(agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.standby_position[0], agent.standby_position[1],
                                                  agent.standby_position[2], agent.standby_yaw)

    def manual_control_law(self, agent: Agent):
        kp = 0.050
        if self.manual_x >= 0:
            agent.standby_position[0] = agent.standby_position[0] - kp * np.sqrt(self.manual_x)
        else:
            agent.standby_position[0] = agent.standby_position[0] + kp * np.sqrt(-self.manual_x)
        if self.manual_y >= 0:
            agent.standby_position[1] = agent.standby_position[1] - kp * np.sqrt(self.manual_y)
        else:
            agent.standby_position[1] = agent.standby_position[1] + kp * np.sqrt(-self.manual_y)
        agent.standby_position[2] = 1 - self.manual_z
        agent.standby_yaw = self.manual_yaw
        agent.cf.commander.send_position_setpoint(agent.standby_position[0], agent.standby_position[1],
                                                  agent.standby_position[2], agent.standby_yaw)

    def takeoff_control_law(self, agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.takeoff_position[0], agent.takeoff_position[1],
                                                  agent.takeoff_position[2], agent.takeoff_yaw)
        d = distance([agent.extpos.x, agent.extpos.y,
                      agent.extpos.z], agent.takeoff_position)
        if d <= self.distance_to_waypoint_threshold:
            print(agent.name, ': Takeoff completed')
            agent.is_flying = True
            agent.standby()

    def landing_control_law(self, agent: Agent):
        agent.cf.commander.send_position_setpoint(agent.land_position[0], agent.land_position[1],
                                                  agent.land_position[2], agent.land_yaw)
        d = vertical_distance(agent.extpos.z, agent.land_position[2])
        if d <= self.distance_to_waypoint_threshold:
            print(agent.name, ': Landing completed')
            agent.stop()

    def z_consensus_control_law(self, agent: Agent):
        in_flight_agents = [agt
                            for agt in self.swarm_agent_list if agt.state != 'Not flying' and agt != agent]
        kp = 0.8
        x_velocity = kp * (agent.z_consensus_xy_position[0] - agent.extpos.x)
        y_velocity = kp * (agent.z_consensus_xy_position[1] - agent.extpos.y)
        z_velocity = kp * (sum([agt.extpos.z - agent.extpos.z
                                for agt in in_flight_agents if agt.name in agent.z_consensus_connectivity]))
        agent.cf.commander.send_velocity_world_setpoint(x_velocity, y_velocity, z_velocity, 0)

    def xy_auto_avoid_control_law(self, agent: Agent):
        agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_obstacles_list]
        vx = 0.0
        vy = 0.0
        vz = agent.xy_auto_avoid_xyz_position[2] - agent.extpos.z

        # Borders
        if agent.extpos.x - self.x_limits[0] < 0:
            vx = vx + np.exp(-(agent.extpos.x - self.x_limits[0]))
        if agent.extpos.y - self.y_limits[0] < 0:
            vy = vy + np.exp(-(agent.extpos.y - self.y_limits[0]))
        if self.x_limits[1] - agent.extpos.x < 0:
            vx = vx - np.exp(-(self.x_limits[1] - agent.extpos.x))
        if self.y_limits[1] - agent.extpos.y < 0:
            vy = vy - np.exp(-(self.y_limits[1] - agent.extpos.y))

        # Obstacles
        for agt in agents_to_avoid:
            d = horizontal_distance([agent.extpos.x, agent.extpos.y], [agt.extpos.x, agt.extpos.y])
            if d <= self.xy_auto_avoid_d0:
                vx = vx - (((agt.extpos.x - agent.extpos.x) / (d+0.001)) * np.exp(-d))
                vy = vy - (((agt.extpos.y - agent.extpos.y) / (d+0.001)) * np.exp(-d))

        # Objective
        k = 1.5
        kv = 1.1
        distance_to_objective = horizontal_distance([agent.extpos.x, agent.extpos.y],
                                                    [agent.xy_auto_avoid_xyz_position[0],
                                                     agent.xy_auto_avoid_xyz_position[1]])

        d1 = np.pi / (2 * self.xy_auto_avoid_omega)
        vx = vx + ((agent.xy_auto_avoid_xyz_position[0] - agent.extpos.x) * k
                    / (2 * d1 * np.sqrt((distance_to_objective+0.001) / d1)))
        vy = vy + ((agent.xy_auto_avoid_xyz_position[1] - agent.extpos.y) * k
                   / (2 * d1 * np.sqrt((distance_to_objective + 0.001) / d1)))

        agent.cf.commander.send_velocity_world_setpoint(kv * vx, kv * vy, vz, 0)

    def wingman_control_law(self, agent):
        agents_to_avoid = [agt for agt in self.swarm_agent_list if agt.name in agent.xy_auto_avoid_obstacles_list]
        vx = 0.0
        vy = 0.0
        vz = agent.wingman_z - agent.extpos.z

        # Borders
        if agent.extpos.x - self.x_limits[0] < 0:
            vx = vx + np.exp(-(agent.extpos.x - self.x_limits[0]))
        if agent.extpos.y - self.y_limits[0] < 0:
            vy = vy + np.exp(-(agent.extpos.y - self.y_limits[0]))
        if self.x_limits[1] - agent.extpos.x < 0:
            vx = vx - np.exp(-(self.x_limits[1] - agent.extpos.x))
        if self.y_limits[1] - agent.extpos.y < 0:
            vy = vy - np.exp(-(self.y_limits[1] - agent.extpos.y))

        # Objective
        k = 1.5
        kv = 1
        leader_agent = [agt for agt in self.swarm_agent_list if agt.name == self.swarm_leader]
        distance_to_objective = horizontal_distance([agent.extpos.x, agent.extpos.y],
                                                    [leader_agent[0].extpos.x,
                                                     leader_agent[0].extpos.y])

        d1 = np.pi / (2 * self.xy_auto_avoid_omega)
        vx = vx + ((leader_agent[0].extpos.x - agent.extpos.x) * k
                   / (2 * d1 * np.sqrt((distance_to_objective+0.001) / d1)))
        vy = vy + ((leader_agent[0].extpos.y - agent.extpos.y) * k
                   / (2 * d1 * np.sqrt((distance_to_objective+0.001) / d1)))

        # Obstacles
        for agt in agents_to_avoid:
            d = horizontal_distance([agent.extpos.x, agent.extpos.y], [agt.extpos.x, agt.extpos.y])
            if d <= self.xy_auto_avoid_d0:
                vx = vx - (((agt.extpos.x - agent.extpos.x) / (d+0.001)) * np.exp(-d))
                vy = vy - (((agt.extpos.y - agent.extpos.y) / (d+0.001)) * np.exp(-d))
        agent.cf.commander.send_velocity_world_setpoint(kv * vx, kv * vy, vz, 0)


def distance(position_1_xyz_list: List[float], position_2_xyz_list: List[float]):
    d = np.sqrt((position_1_xyz_list[0] - position_2_xyz_list[0]) ** 2
                + (position_1_xyz_list[1] - position_2_xyz_list[1]) ** 2
                + (position_1_xyz_list[2] - position_2_xyz_list[2]) ** 2)
    return d


def vertical_distance(position_1_z: float, position_2_z: float):
    vd = np.sqrt((position_1_z - position_2_z) ** 2)
    return vd


def horizontal_distance(position_1_xy: List[float], position_2_xy: List[float]):
    hd = np.sqrt((position_1_xy[0] - position_2_xy[0]) ** 2
                 + (position_1_xy[1] - position_2_xy[1]) ** 2)
    return hd
