from __future__ import annotations
import json
import flocking
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityNedYaw
import pymap3d as pm
from communication import DroneCommunication
from data_structures import AgentTelemetry
import math
import numpy as np
from string import digits


def index_checker(input_index, length) -> int:
    if input_index >= length:
        return int(input_index % length)
    return input_index


def only_numeric(input_string):
    output_string = "".join(c for c in input_string if c in digits)
    return output_string


class Experiment:
    def __init__(self, id, swarm_telem, experiment_file_path) -> None:
        self.ready_flag = False
        self.id = id
        self.least_distance = 2  # minimum allowed distance between two agents
        # Set up corridor variables
        self.points = []
        self.directions = []
        self.current_index = 0
        self.target_point = np.array([0, 0, 0])
        self.target_direction = np.array([1, 1, 1])
        self.load(experiment_file_path, swarm_telem)

    def load(self, experiment_file_path, swarm_telem):

        with open(experiment_file_path, "r") as f:
            experiment_parameters = json.load(f)

        self.k_migration = experiment_parameters["k_migration"]
        self.k_lane_cohesion = experiment_parameters["k_lane_cohesion"]
        self.k_rotation = experiment_parameters["k_rotation"]
        self.k_separation = experiment_parameters["k_seperation"]
        self.r_conflict = experiment_parameters["r_conflict"]
        self.r_collision = experiment_parameters["r_collision"]
        self.pre_start_positions = experiment_parameters["pre_start_positions"]
        self.lane_radius = experiment_parameters["corridor_radius"]
        self.points = experiment_parameters["corridor_points"]
        self.length = len(self.points)
        self.create_directions()
        self.initial_nearest_point(swarm_telem)
        self.ready_flag = True
        print("ready")

    def get_pre_start_positions(self, swarm_telem):
        numeric_ids = {}
        assigned_pre_start_positions = {}
        # numeric_id = int(only_numeric(self.id))
        for agent in swarm_telem.keys():
            numeric_ids[agent] = int(only_numeric(agent))

        swarm_priorities = sorted(numeric_ids, key=numeric_ids.get)

        for i, agent in enumerate(swarm_priorities):
            if len(self.pre_start_positions) > i:
                assigned_pre_start_positions[agent] = self.pre_start_positions[i]
            else:
                # if there isnt enough pre start positions, start from current position
                assigned_pre_start_positions[agent] = swarm_telem[self.id].position_ned

        return assigned_pre_start_positions

    def create_directions(self):
        for i in range(len(self.points)):
            # All points must be converted to np arrays
            self.points[i] = np.array(self.points[i])

            if i == len(self.points) - 1:
                self.directions.append(
                    (self.points[0] - self.points[i])
                    / np.linalg.norm(self.points[0] - self.points[i])
                )
            else:
                self.directions.append(
                    (self.points[i + 1] - self.points[i])
                    / np.linalg.norm(self.points[i + 1] - self.points[i])
                )

    def initial_nearest_point(self, swarm_telem) -> None:
        lnitial_least_distance = math.inf
        for i in range(len(self.points)):
            range_to_point_i = np.linalg.norm(
                np.array(swarm_telem[self.id].position_ned) - self.points[i]
            )
            if range_to_point_i <= lnitial_least_distance:
                lnitial_least_distance = range_to_point_i
                self.current_index = i

    def path_following(self, swarm_telem, max_speed, time_step, max_accel):
        self.target_point = self.points[self.current_index]
        self.target_direction = self.directions[self.current_index]
        iterator = 0
        # Finding the next bigger Index ----------
        range_to_next = (
            np.array(swarm_telem[self.id].position_ned)
            - self.points[index_checker(self.current_index + 1, self.length)]
        )

        if (
            np.dot(range_to_next, self.directions[self.current_index]) > 0
        ):  # drone has passed the point next to current one
            self.current_index = index_checker(self.current_index + 1, len(self.points))
            self.target_point = self.points[self.current_index]
            self.target_direction = self.directions[self.current_index]
            iterator = 0
            dot_fartherpoints = 0
            while dot_fartherpoints >= 0:  # Searching for farther points
                iterator += 1
                farther_point = index_checker(
                    self.current_index + iterator, len(self.points)
                )
                range_to_farther_point = (
                    np.array(swarm_telem[self.id].position_ned)
                    - self.points[farther_point]
                )
                dot_fartherpoints = np.dot(
                    range_to_farther_point, self.directions[farther_point - 1]
                )

            self.current_index = (
                farther_point - 1
            )  # farther_point here has negative dot product
            self.target_point = self.points[self.current_index]
            self.target_direction = self.directions[self.current_index]

        # Calculating migration velocity (normalized)---------------------
        limit_v_migration = 1
        v_migration = self.target_direction / np.linalg.norm(self.target_direction)
        if np.linalg.norm(v_migration) > limit_v_migration:
            v_migration = v_migration * limit_v_migration / np.linalg.norm(v_migration)

        # Calculating lane Cohesion Velocity ---------------
        limit_v_lane_cohesion = 1
        lane_cohesion_position_error = self.target_point - np.array(
            swarm_telem[self.id].position_ned
        )
        lane_cohesion_position_error -= (
            np.dot(lane_cohesion_position_error, self.target_direction)
            * self.target_direction
        )
        lane_cohesion_position_error_magnitude = np.linalg.norm(
            lane_cohesion_position_error
        )

        if np.linalg.norm(lane_cohesion_position_error) != 0:
            v_lane_cohesion = (
                (lane_cohesion_position_error_magnitude - self.lane_radius)
                * lane_cohesion_position_error
                / np.linalg.norm(lane_cohesion_position_error)
            )
        else:
            v_lane_cohesion = np.array([0.01, 0.01, 0.01])

        if np.linalg.norm(v_lane_cohesion) > limit_v_lane_cohesion:
            v_lane_cohesion = (
                v_lane_cohesion
                * limit_v_lane_cohesion
                / np.linalg.norm(v_lane_cohesion)
            )

        # Calculating v_rotation (normalized)---------------------
        limit_v_rotation = 1
        if lane_cohesion_position_error_magnitude < self.lane_radius:
            v_rotation_magnitude = (
                lane_cohesion_position_error_magnitude / self.lane_radius
            )
        else:
            v_rotation_magnitude = (
                self.lane_radius / lane_cohesion_position_error_magnitude
            )
        cross_prod = np.cross(lane_cohesion_position_error, self.target_direction)
        if np.linalg.norm(cross_prod) != 0:
            v_rotation = v_rotation_magnitude * cross_prod / np.linalg.norm(cross_prod)
        else:
            v_rotation = np.array[0, 0, 0]

        if np.linalg.norm(v_rotation) > limit_v_rotation:
            v_rotation = v_rotation * limit_v_rotation / np.linalg.norm(v_rotation)

        # Calculating v_separation (normalized) -----------------------------
        limit_v_separation = 5
        r_conflict = 5
        r_collision = 2.5
        v_separation = np.array([0, 0, 0])
        for key in swarm_telem:
            if key == self.id:
                continue
            p = np.array(swarm_telem[key].position_ned)
            x = np.array(swarm_telem[self.id].position_ned) - p
            d = np.linalg.norm(x)
            if self.least_distance > d:
                self.least_distance = d
            if d <= r_conflict and d > r_collision and d != 0:
                v_separation = v_separation + (
                    (x / d) * (r_conflict - d / r_conflict - r_collision)
                )
            if d <= r_collision and d != 0:
                v_separation = v_separation + 1 * (x / d)
            if np.linalg.norm(v_separation) > limit_v_separation:
                v_separation = (
                    v_separation * limit_v_separation / np.linalg.norm(v_separation)
                )

        desired_vel = (
            self.k_lane_cohesion * v_lane_cohesion
            + self.k_migration * v_migration
            + self.k_rotation * v_rotation
            + self.k_separation * v_separation
        )

        # NOTE maybe add lane cohesion as well so we point the right way when coming from far away
        yaw = flocking.get_desired_yaw(v_migration[0], v_migration[1])

        output_vel = flocking.check_velocity(
            desired_vel, swarm_telem[self.id], max_speed, yaw, time_step, max_accel
        )
        return output_vel
