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
        self.points = [[]]
        self.current_path = 0
        self.rotation_factor = 1
        self.directions = []
        self.current_index = 0
        self.pass_permission = False
        self.target_point = np.array([0, 0, 0], dtype="float64")
        self.target_direction = np.array([1, 1, 1], dtype="float64")
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
        self.pass_permission_list = experiment_parameters[
            "pass_permission"
        ]  # the permission to go to another path
        self.repeat = experiment_parameters[
            "repeat"
        ]  # the permission to go to another path
        self.pre_start_positions = experiment_parameters["pre_start_positions"]
        self.initial_paths = experiment_parameters["initial_paths"]
        self.lane_radius = experiment_parameters["corridor_radius"]
        self.points = experiment_parameters["corridor_points"]
        self.rotation_dir = experiment_parameters["path_rotation_dir"]
        self.length = [
            len(self.points[j]) for j in range(len(self.points))
        ]  # j is the number of a path
        self.create_directions()
        # self.initial_nearest_point(swarm_telem)
        self.create_adjacent_points()
        self.ready_flag = True

    def get_pre_start_positions(self, swarm_telem, swarm_priorities):

        assigned_pre_start_positions = {}

        for i, agent in enumerate(swarm_priorities):
            if len(self.pre_start_positions) > i:
                assigned_pre_start_positions[agent] = self.pre_start_positions[i]
            else:
                # if there isnt enough pre start positions, start from current position
                assigned_pre_start_positions[agent] = swarm_telem[self.id].position_ned

        return assigned_pre_start_positions

    def get_path_and_permission(self, swarm_priorities):
        if self.id in swarm_priorities:
            self.current_path = self.initial_paths[swarm_priorities.index(self.id)]
            self.pass_permission = self.pass_permission_list[
                swarm_priorities.index(self.id)
            ]

    def get_swarm_priorities(self, swarm_telem):
        numeric_ids = {}
        # assigned_pre_start_positions = {}
        # numeric_id = int(only_numeric(self.id))
        for agent in swarm_telem.keys():
            numeric_ids[agent] = int(only_numeric(agent))

        swarm_priorities = sorted(numeric_ids, key=numeric_ids.get)
        return swarm_priorities

    def create_directions(self):
        for j in range(len(self.points)):
            self.directions.append([])
            for i in range(len(self.points[j])):
                # All points must be converted to np arrays
                self.points[j][i] = np.array(self.points[j][i], dtype="float64")
                if i == len(self.points[j]) - 1:
                    self.directions[j].append(
                        (self.points[j][0] - self.points[j][i])
                        / np.linalg.norm(self.points[j][0] - self.points[j][i])
                    )
                else:
                    self.directions[j].append(
                        (self.points[j][i + 1] - self.points[j][i])
                        / np.linalg.norm(self.points[j][i + 1] - self.points[j][i])
                    )
        print("done creating directions")

    def create_adjacent_points(self) -> None:
        self.adjacent_points = []
        for j in range(len(self.points)):  # j is the number of path
            self.adjacent_points.append({})
            for i in range(len(self.points[j])):
                for k in range(
                    len(self.points[index_checker(j + 1, len(self.points))])
                ):
                    distance = np.linalg.norm(
                        self.points[j][i]
                        - self.points[index_checker(j + 1, len(self.points))][k]
                    )
                    if (
                        distance
                        <= self.lane_radius[j]
                        + self.lane_radius[index_checker(j + 1, len(self.points))]
                        and np.dot(
                            self.directions[j][i],
                            self.directions[index_checker(j + 1, len(self.points))][k],
                        )
                        == 1
                    ):
                        pass_vector = (
                            self.points[j][i]
                            - self.points[index_checker(j + 1, len(self.points))][k]
                        )
                        pass_vector = pass_vector / np.linalg.norm(pass_vector)
                        self.adjacent_points[j].update(
                            {i: [k, pass_vector]}
                        )  # jth dictionary is {adj. point of path j: [adj. point of j+1, vector from adj. point of path j to adj. point of j+1]}

    def initial_nearest_point(self, swarm_telem) -> None:
        lnitial_least_distance = math.inf
        for i in range(len(self.points[self.current_path])):
            range_to_point_i = np.linalg.norm(
                np.array(swarm_telem[self.id].position_ned, dtype="float64")
                - self.points[self.current_path][i]
            )
            if range_to_point_i <= lnitial_least_distance:
                lnitial_least_distance = range_to_point_i
                self.current_index = i

    def switch(self):
        self.pass_permission = (
            False  # the agent is not allowed to get back to previous path anymore
        )

        self.current_index = self.adjacent_points[self.current_path][
            self.current_index
        ][
            0
        ]  # now current index is a point of the next path
        self.current_path = index_checker(self.current_path + 1, len(self.points))
        self.target_point = self.points[self.current_path][self.current_index]
        self.target_direction = self.directions[self.current_path][self.current_index]

    def path_following(self, swarm_telem, max_speed, time_step, max_accel):
        self.target_point = self.points[self.current_path][self.current_index]
        self.target_direction = self.directions[self.current_path][self.current_index]
        if (
            self.current_index in self.adjacent_points[self.current_path]
            and self.pass_permission == True
        ):
            pass_vector = self.adjacent_points[self.current_path][self.current_index][1]
            lane_cohesion_position_error = self.target_point - np.array(
                swarm_telem[self.id].position_ned, dtype="float64"
            )
            lane_cohesion_position_error -= (
                np.dot(lane_cohesion_position_error, self.target_direction)
                * self.target_direction
            )
            cos_of_angle = np.dot(pass_vector, lane_cohesion_position_error) / (
                np.linalg.norm(pass_vector)
                * np.linalg.norm(lane_cohesion_position_error)
            )
            if cos_of_angle >= 0.9:
                self.switch()
        # Finding the next bigger Index ----------
        range_to_next = (
            np.array(swarm_telem[self.id].position_ned, dtype="float64")
            - self.points[self.current_path][
                index_checker(self.current_index + 1, self.length[self.current_path])
            ]
        )
        if (
            np.dot(
                range_to_next, self.directions[self.current_path][self.current_index]
            )
            > 0
        ):  # drone has passed the point next to current one
            self.current_index = index_checker(
                self.current_index + 1, self.length[self.current_path]
            )
            self.target_point = self.points[self.current_path][self.current_index]
            self.target_direction = self.directions[self.current_path][
                self.current_index
            ]
            iterator = 0
            dot_fartherpoints = 0
            while dot_fartherpoints >= 0:  # Searching for farther points
                iterator += 1
                farther_point = index_checker(
                    self.current_index + iterator, self.length[self.current_path]
                )
                range_to_farther_point = (
                    np.array(swarm_telem[self.id].position_ned, dtype="float64")
                    - self.points[self.current_path][farther_point]
                )
                dot_fartherpoints = np.dot(
                    range_to_farther_point,
                    self.directions[self.current_path][farther_point - 1],
                )
            # Now farther_point has negative dot product
            if farther_point != 0:
                self.current_index = farther_point - 1
            else:
                self.current_index = (
                    self.length[self.current_path] - 1
                )  # the index of the last point of the current path
                if not self.repeat:
                    self.k_lane_cohesion = 0
                    self.k_migration = 0
                    self.k_rotation = 0  # passed last point of path

            self.target_point = self.points[self.current_path][self.current_index]
            self.target_direction = self.directions[self.current_path][
                self.current_index
            ]
        # Calculating migration velocity (normalized)---------------------
        limit_v_migration = 1
        v_migration = self.target_direction / np.linalg.norm(self.target_direction)
        if np.linalg.norm(v_migration) > limit_v_migration:
            v_migration = v_migration * limit_v_migration / np.linalg.norm(v_migration)
        # Calculating lane Cohesion Velocity ---------------
        limit_v_lane_cohesion = 1
        lane_cohesion_position_error = self.target_point - np.array(
            swarm_telem[self.id].position_ned, dtype="float64"
        )
        lane_cohesion_position_error = lane_cohesion_position_error - (
            np.dot(lane_cohesion_position_error, self.target_direction)
            * self.target_direction
        )
        lane_cohesion_position_error_magnitude = np.linalg.norm(
            lane_cohesion_position_error
        )
        if np.linalg.norm(lane_cohesion_position_error) != 0:
            v_lane_cohesion = (
                (
                    lane_cohesion_position_error_magnitude
                    - self.lane_radius[self.current_path]
                )
                * lane_cohesion_position_error
                / np.linalg.norm(lane_cohesion_position_error)
            )
        else:
            v_lane_cohesion = np.array([0.01, 0.01, 0.01], dtype="float64")

        if np.linalg.norm(v_lane_cohesion) > limit_v_lane_cohesion:
            v_lane_cohesion = (
                v_lane_cohesion
                * limit_v_lane_cohesion
                / np.linalg.norm(v_lane_cohesion)
            )
        # Calculating v_rotation (normalized)---------------------
        limit_v_rotation = 1
        if lane_cohesion_position_error_magnitude < self.lane_radius[self.current_path]:
            v_rotation_magnitude = (
                lane_cohesion_position_error_magnitude
                / self.lane_radius[self.current_path]
            )
        else:
            v_rotation_magnitude = (
                self.lane_radius[self.current_path]
                / lane_cohesion_position_error_magnitude
            )
        cross_prod = np.cross(lane_cohesion_position_error, self.target_direction)
        if np.linalg.norm(cross_prod) != 0:
            v_rotation = (
                self.rotation_dir[self.current_path]
                * v_rotation_magnitude
                * cross_prod
                / np.linalg.norm(cross_prod)
            )
        else:
            v_rotation = np.array([0, 0, 0], dtype="float64")

        if np.linalg.norm(v_rotation) > limit_v_rotation:
            v_rotation = v_rotation * limit_v_rotation / np.linalg.norm(v_rotation)
        # Calculating v_separation (normalized) -----------------------------
        limit_v_separation = 5
        r_conflict = 5
        r_collision = 2.5
        v_separation = np.array([0, 0, 0], dtype="float64")
        for key in swarm_telem:
            if key == self.id:
                continue
            p = np.array(swarm_telem[key].position_ned, dtype="float64")
            x = np.array(swarm_telem[self.id].position_ned, dtype="float64") - p
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
        v_separation = np.array([0, 0, 0])
        # NOTE maybe add lane cohesion as well so we point the right way when coming from far away
        # yaw = flocking.get_desired_yaw(v_migration[0], v_migration[1])
        yaw_vel = (
            self.k_lane_cohesion * v_lane_cohesion
            + self.k_migration * v_migration
            + self.k_rotation * v_rotation
        )
        yaw = flocking.get_desired_yaw(yaw_vel[0], yaw_vel[1])
        output_vel = flocking.check_velocity(
            desired_vel, swarm_telem[self.id], max_speed, yaw, time_step, max_accel
        )
        return output_vel
