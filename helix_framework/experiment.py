from __future__ import annotations

import json
import math
import time
from string import digits

import flocking
import numpy as np
import pymap3d as pm
from communication import DroneCommunication
from data_structures import AgentTelemetry
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityNedYaw


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
        self.v_migration = np.array([0, 0, 0], dtype="float64")
        self.v_lane_cohesion = np.array([0, 0, 0], dtype="float64")
        self.v_separation = np.array([0, 0, 0], dtype="float64")
        self.v_rotation = np.array([0, 0, 0], dtype="float64")
        self.v_force_field = np.array([0, 0, 0], dtype="float64")

        # Set up mission variables
        self.points = [[]]
        self.start_time = 0
        self.start_delay = 0  # delay in micro seconds
        self.current_path = 0
        self.rotation_factor = 1
        self.directions = []
        self.current_index = 0
        self.pass_permission = (
            {}
        )  # self.pass_permission[j]: shows the path wchich path j can switch to
        self.target_point = np.array([0, 0, 0], dtype="float64")
        self.target_direction = np.array([1, 1, 1], dtype="float64")
        self.min_distance = [
            math.inf,
            0,
            0,
            np.array([0, 0, 0], dtype="float64"),
            np.array([0, 0, 0], dtype="float64"),
        ]  # [distance, self.id (id of the current drone), id of the other drone, position of current drone, position of the other drone]
        self.switched_positions = []  # to save the positions where the drone switches
        self.force_field_mode = False
        self.load(experiment_file_path, swarm_telem)

    def load(self, experiment_file_path, swarm_telem):

        with open(experiment_file_path, "r") as f:
            experiment_parameters = json.load(f)

        self.k_migration = experiment_parameters["k_migration"]
        self.k_lane_cohesion = experiment_parameters["k_lane_cohesion"]
        self.k_rotation = experiment_parameters["k_rotation"]
        self.k_separation = experiment_parameters["k_seperation"]
        self.k_force_field = experiment_parameters["k_force_field"]
        self.r_conflict = experiment_parameters["r_conflict"]
        self.r_collision = experiment_parameters["r_collision"]
        self.pass_permission_list = experiment_parameters[
            "pass_permission_list"
        ]  # self.pass_permission_list[n]: is a dictionary to show switching paths for drone n, it should be empty if there is no switching
        self.switching_points = experiment_parameters[
            "switching_points"
        ]  # self.pass_switching_points[j]: contains all of the points which can switch from path j
        self.repeat = experiment_parameters[
            "repeat"
        ]  # the permission to go to another path
        self.pre_start_positions = experiment_parameters["pre_start_positions"]
        self.initial_paths = experiment_parameters["initial_paths"]
        self.lane_radius = experiment_parameters["corridor_radius"]
        self.points = experiment_parameters["corridor_points"]
        self.ribbons = experiment_parameters["ribbons"]
        self.cylinders = experiment_parameters["cylinders"]
        self.tubes = experiment_parameters["tubes"]
        self.rotation_dir = experiment_parameters["path_rotation_dir"]
        self.start_delay_list = experiment_parameters["start_delay_list"]
        self.vortices = experiment_parameters["vortices"]
        if (
            len(self.vortices) != 0
        ):  # if a vortex is defined, vortex-centre should be defined as well
            self.force_field_mode = True
            self.vortex_centre = np.array(
                experiment_parameters["vortex_centre"], dtype="float64"
            )
            for i in range(len(self.vortices)):
                self.vortices[i] = np.array(self.vortices[i], dtype="float64")

        if (
            len(self.ribbons) != 0
        ):  # if a ribbon is defined, ribbon_norm_vect should be defined as well
            ribbons_norm_vect_str = experiment_parameters["ribbons_norm_vect"]
            self.ribbons_norm_vect = {}
            for path, norm_vect_list in ribbons_norm_vect_str.items():
                normalized_norm_vect_list = []
                for norm_vect in norm_vect_list:
                    normalized_norm_vect = np.array(norm_vect)
                    if (
                        np.linalg.norm(normalized_norm_vect) != 0
                    ):  # normalizinf the norm vector
                        normalized_norm_vect = normalized_norm_vect / np.linalg.norm(
                            normalized_norm_vect
                        )
                    normalized_norm_vect_list.append(normalized_norm_vect)

                self.ribbons_norm_vect.update({int(path): normalized_norm_vect_list})

        self.length = [
            len(self.points[j]) for j in range(len(self.points))
        ]  # j is the number of a path
        self.passed_last_point = [
            False for j in range(len(self.points))
        ]  # to see if the drone has passed the last point of path j or not
        self.determine_path_type()
        self.create_directions()
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
            self.start_delay = self.start_delay_list[swarm_priorities.index(self.id)]
            for j, next_path in self.pass_permission_list[
                swarm_priorities.index(self.id)
            ].items():
                self.pass_permission.update({int(j): next_path})  # j here is a string

        self.create_adjacent_points()

    def determine_path_type(self):
        # the path type for each path is tube unless another type has been set for a path
        self.path_type = ["tube" for i in range(len(self.points))]
        for i in range(len(self.points)):
            if i in self.ribbons:
                self.path_type[i] = "ribbon"
            elif i in self.cylinders:
                self.path_type[i] = "cylinder"

    def get_swarm_priorities(
        self, swarm_telem
    ):  # swarm_telem is an object of AgentTelemetry
        numeric_ids = {}
        # assigned_pre_startawait asynciopositions = {}
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
                    if (
                        self.repeat[j] == "STAY" or self.repeat[j] == "STOP"
                    ):  # if we want to stay at the last point
                        self.directions[j].append(
                            (self.points[j][i] - self.points[j][i - 1])
                            / np.linalg.norm(self.points[j][i] - self.points[j][i - 1])
                        )
                    else:
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
        self.adjacent_points = [
            {} for j in range(len(self.points))
        ]  # jth dictionary is for jth path
        for (
            path
        ) in (
            self.pass_permission
        ):  # path is the number of path in dictionary self.pass_permission
            for switching_point in self.switching_points[path]:
                shortest_dist_switch = (
                    math.inf
                )  # the shortest distance for switching point to switch
                for next_path in self.pass_permission[path]:
                    for k in range(
                        len(self.points[next_path])
                    ):  # self.pass_permission[j] shows the path we can switch from path j
                        switching_points_distance = np.linalg.norm(
                            self.points[path][switching_point]
                            - self.points[next_path][k]
                        )

                        if (
                            switching_points_distance
                            <= (
                                self.lane_radius[path][switching_point]
                                + self.lane_radius[next_path][k]
                            )
                            * 1.01
                            and switching_points_distance
                            < shortest_dist_switch  # this 1 percent is to compensate numerical calculation inaccuracies
                        ):
                            shortest_dist_switch = switching_points_distance
                            pass_vector = (
                                self.points[path][switching_point]
                                - self.points[next_path][k]
                            )
                            if np.linalg.norm(pass_vector) != 0:
                                pass_vector = pass_vector / np.linalg.norm(pass_vector)

                            self.adjacent_points[path].update(
                                {
                                    switching_point: [k, next_path, pass_vector]
                                }  # each switching point in path j can just switch to one path, if we add the same key with different value, the last key with the last value would be considered
                            )  # jth dictionary is {adj. point of path j: [point of next_path, next_path, vector from adj. point of path j to adj. point of next_path]}

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

    def switch(self, switching_point):
        print(
            self.id, "switched from", self.current_path, "at index", self.current_index
        )
        next_path = self.adjacent_points[self.current_path][switching_point][1]
        self.pass_permission[
            self.current_path
        ] = []  # the agent is not allowed to get back to previous path anymore

        self.current_index = self.adjacent_points[self.current_path][switching_point][
            0
        ]  # now current index is a point of the next path
        self.current_path = next_path
        self.target_point = self.points[self.current_path][self.current_index]
        self.target_direction = self.directions[self.current_path][self.current_index]

    def path_following(self, swarm_telem, max_speed, time_step, max_accel, mqtt_client):
        self.target_point = self.points[self.current_path][self.current_index]
        self.target_direction = self.directions[self.current_path][self.current_index]
        if (
            self.current_index
            in self.adjacent_points[
                self.current_path
            ]  # the index is elgible for switching
        ):

            pass_vector = self.adjacent_points[self.current_path][self.current_index][2]
            lane_cohesion_position_error = self.target_point - np.array(
                swarm_telem[self.id].position_ned, dtype="float64"
            )
            lane_cohesion_position_error -= (
                np.dot(lane_cohesion_position_error, self.target_direction)
                * self.target_direction
            )
            cos_of_angle = 0
            if (
                abs(np.linalg.norm(pass_vector)) <= 0.05
                or abs(np.linalg.norm(lane_cohesion_position_error)) <= 0.05
            ):
                cos_of_angle = 1
            else:
                cos_of_angle = np.dot(pass_vector, lane_cohesion_position_error) / (
                    np.linalg.norm(pass_vector)
                    * np.linalg.norm(lane_cohesion_position_error)
                )

            if cos_of_angle >= 0.9:
                self.switched_positions.append(
                    np.array(swarm_telem[self.id].position_ned, dtype="float64")
                )
                self.switch(self.current_index)
        # Finding the next bigger Index ----------
        dot_next_point = 0

        while dot_next_point >= 0 and not (
            self.passed_last_point[self.current_path] == True
            and (
                self.repeat[self.current_path] == "STAY"
                or self.repeat[self.current_path] == "STOP"
            )
        ):  # Searching for farther points, if the drone reach the last point and it has STAY for its path, searching for more points should be stopped
            next_point = index_checker(
                self.current_index + 1, self.length[self.current_path]
            )
            range_to_next = (
                np.array(swarm_telem[self.id].position_ned, dtype="float64")
                - self.points[self.current_path][next_point]
            )
            dot_next_point = np.dot(
                range_to_next,
                self.directions[self.current_path][next_point],
            )
            if dot_next_point >= 0:  # the drone has passed the next point
                self.current_index = next_point
                if (
                    next_point == self.length[self.current_path] - 1
                ):  # it shows the drone has passed the last point of the current path
                    self.passed_last_point[self.current_path] = True

        self.target_point = self.points[self.current_path][self.current_index]
        self.target_direction = self.directions[self.current_path][self.current_index]
        # Calculating migration velocity (normalized)---------------------
        limit_v_migration = 1
        self.v_migration = self.target_direction / np.linalg.norm(self.target_direction)
        if np.linalg.norm(self.v_migration) > limit_v_migration:
            self.v_migration = (
                self.v_migration * limit_v_migration / np.linalg.norm(self.v_migration)
            )
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
            self.v_lane_cohesion = (
                (
                    lane_cohesion_position_error_magnitude
                    - self.lane_radius[self.current_path][self.current_index]
                )
                * lane_cohesion_position_error
                / np.linalg.norm(lane_cohesion_position_error)
            )
        elif self.lane_radius[self.current_path][self.current_index] == 0:
            self.v_lane_cohesion = np.array([0.0, 0.0, 0.0], dtype="float64")
        else:
            self.v_lane_cohesion = np.array([0.01, 0.01, 0.01], dtype="float64")

        # if we have a cylinder
        if (
            self.path_type[self.current_path] == "cylinder"
            and lane_cohesion_position_error_magnitude
            < self.lane_radius[self.current_path][self.current_index]
        ):
            self.v_lane_cohesion = np.array([0.0, 0.0, 0.0], dtype="float64")

        if np.linalg.norm(self.v_lane_cohesion) > limit_v_lane_cohesion:
            self.v_lane_cohesion = (
                self.v_lane_cohesion
                * limit_v_lane_cohesion
                / np.linalg.norm(self.v_lane_cohesion)
            )
        # Calculating v_rotation (normalized)---------------------
        limit_v_rotation = 1
        if (
            lane_cohesion_position_error_magnitude == 0
            or self.lane_radius[self.current_path][self.current_index] == 0
        ):
            v_rotation_magnitude = 0
        else:
            if (
                lane_cohesion_position_error_magnitude
                < self.lane_radius[self.current_path][self.current_index]
            ):
                v_rotation_magnitude = (
                    lane_cohesion_position_error_magnitude
                    / self.lane_radius[self.current_path][self.current_index]
                )
            else:
                v_rotation_magnitude = (
                    self.lane_radius[self.current_path][self.current_index]
                    / lane_cohesion_position_error_magnitude
                )
        cross_prod = np.cross(lane_cohesion_position_error, self.target_direction)
        if np.linalg.norm(cross_prod) != 0:
            self.v_rotation = (
                self.rotation_dir[self.current_path]
                * v_rotation_magnitude
                * cross_prod
                / np.linalg.norm(cross_prod)
            )
        else:
            self.v_rotation = np.array([0, 0, 0], dtype="float64")

        if np.linalg.norm(self.v_rotation) > limit_v_rotation:
            self.v_rotation = (
                self.v_rotation * limit_v_rotation / np.linalg.norm(self.v_rotation)
            )
        # Calculating v_separation (normalized) -----------------------------
        limit_v_separation = 5
        self.v_separation = np.array([0, 0, 0], dtype="float64")
        for key in swarm_telem:
            if key == self.id:
                continue
            p = np.array(swarm_telem[key].position_ned, dtype="float64")
            x = np.array(swarm_telem[self.id].position_ned, dtype="float64") - p
            d = np.linalg.norm(x)

            # finding the minimum distance
            if d <= self.min_distance[0]:
                self.min_distance[0] = d
                self.min_distance[1] = self.id
                self.min_distance[2] = key
                self.min_distance[3] = np.array(
                    swarm_telem[self.id].position_ned, dtype="float64"
                )
                self.min_distance[4] = np.array(
                    swarm_telem[key].position_ned, dtype="float64"
                )

            if d <= self.r_conflict and d > self.r_collision and d != 0:
                self.v_separation = self.v_separation + (
                    (x / d)
                    * ((self.r_conflict - d) / (self.r_conflict - self.r_collision))
                )
            if d <= self.r_collision and d != 0:
                self.v_separation = self.v_separation + 1 * (x / d)

        if np.linalg.norm(self.v_separation) > limit_v_separation:
            self.v_separation = (
                self.v_separation
                * limit_v_separation
                / np.linalg.norm(self.v_separation)
            )

        # checking for start delay time
        if (
            swarm_telem[self.id].current_time - self.start_time <= self.start_delay
            and swarm_telem[self.id].current_time != 0
        ):  # if it is true, it is not the time to start the mission
            self.v_lane_cohesion = np.array([0, 0, 0], dtype="float64")
            self.v_migration = np.array([0, 0, 0], dtype="float64")
            self.v_rotation = np.array([0, 0, 0], dtype="float64")
            self.v_separation = np.array([0, 0, 0], dtype="float64")

        # checking the last point of the current path
        if (
            self.passed_last_point[self.current_path] == True
            and self.repeat[self.current_path] == "STOP"
        ):
            # stop the drone at the current position with seperation rule still running
            self.v_lane_cohesion = np.array([0, 0, 0], dtype="float64")
            self.v_migration = np.array([0, 0, 0], dtype="float64")
            self.v_rotation = np.array([0, 0, 0], dtype="float64")

            # update status to for automated experiments
            # mqtt_client.publish(self.id + "/status", "DONE")

        elif (
            self.passed_last_point[self.current_path] == True
            and self.repeat[self.current_path] == "STAY"
        ):
            self.v_migration = np.array([0, 0, 0], dtype="float64")

        elif (
            self.passed_last_point[self.current_path] == True
            and self.repeat[self.current_path] == "REPEAT"
        ):
            pass

        # if we have ribbons
        if self.path_type[self.current_path] == "ribbon":
            # calculating v_lane_cohesion normal to the ribbon
            lane_cohesion_position_error = self.target_point - np.array(
                swarm_telem[self.id].position_ned, dtype="float64"
            )  # target point is a point on the current ribbon
            lane_cohesion_position_error_normal = (
                np.dot(
                    lane_cohesion_position_error,
                    self.ribbons_norm_vect[self.current_path][self.current_index],
                )
                * self.ribbons_norm_vect[self.current_path][self.current_index]
            )

            # calculating v_lane_cohesion along width of the ribbon
            lane_cohesion_position_error_lane_width_vector = (
                lane_cohesion_position_error
                - (
                    np.dot(lane_cohesion_position_error, self.target_direction)
                    * self.target_direction
                )
            )
            lane_cohesion_position_error_lane_width_vector = (
                lane_cohesion_position_error_lane_width_vector
                - np.dot(
                    lane_cohesion_position_error_lane_width_vector,
                    self.ribbons_norm_vect[self.current_path][self.current_index],
                )
                * self.ribbons_norm_vect[self.current_path][self.current_index]
            )

            lane_cohesion_position_error_lane_width_vector_magnitude = np.linalg.norm(
                lane_cohesion_position_error_lane_width_vector
            )

            if (
                lane_cohesion_position_error_lane_width_vector_magnitude
                <= self.lane_radius[self.current_path][self.current_index]
            ):  # when the drone distance in plane of ribbon is less than ribbon width
                lane_cohesion_position_error_lane_width_vector = np.array(
                    [0.0, 0.0, 0.0], dtype="float64"
                )
            else:
                lane_cohesion_position_error_lane_width_vector = (
                    (
                        lane_cohesion_position_error_lane_width_vector_magnitude
                        - self.lane_radius[self.current_path][self.current_index]
                    )
                    * lane_cohesion_position_error_lane_width_vector
                    / lane_cohesion_position_error_lane_width_vector_magnitude
                )

            lane_cohesion_position_error = (
                lane_cohesion_position_error_normal
                + lane_cohesion_position_error_lane_width_vector
            )
            self.v_lane_cohesion = lane_cohesion_position_error

            if np.linalg.norm(self.v_lane_cohesion) > limit_v_lane_cohesion:
                self.v_lane_cohesion = (
                    self.v_lane_cohesion
                    * limit_v_lane_cohesion
                    / np.linalg.norm(self.v_lane_cohesion)
                )

            self.v_rotation = np.array([0, 0, 0], dtype="float64")

        # if we have a cylinder
        if self.path_type[self.current_path] == "cylinder":
            if (
                lane_cohesion_position_error_magnitude
                < self.lane_radius[self.current_path][self.current_index]
            ):
                self.v_lane_cohesion = np.array([0.0, 0.0, 0.0], dtype="float64")
            self.v_rotation = np.array([0, 0, 0], dtype="float64")

        if np.linalg.norm(self.v_lane_cohesion) > limit_v_lane_cohesion:
            self.v_lane_cohesion = (
                self.v_lane_cohesion
                * limit_v_lane_cohesion
                / np.linalg.norm(self.v_lane_cohesion)
            )

        # if we have vortices
        if self.force_field_mode == True:
            self.v_force_field = np.array([0, 0, 0], dtype="float64")
            R_to_centre = np.linalg.norm(
                np.array(swarm_telem[self.id].position_ned, dtype="float64")
                - self.vortex_centre
            )  # for all of the fields

            # Adding effect of the source field at the vortex centre
            force_field = (
                np.array(swarm_telem[self.id].position_ned, dtype="float64")
                - self.vortex_centre
            )
            force_field_magnitude = np.linalg.norm(force_field)

            if force_field_magnitude != 0:
                self.v_force_field = (
                    self.v_force_field
                    + (1 / R_to_centre) * force_field / force_field_magnitude
                )

            for vortex in self.vortices:
                force_field = np.cross(
                    vortex,
                    np.array(swarm_telem[self.id].position_ned, dtype="float64")
                    - self.vortex_centre,
                )
                force_field_magnitude = np.linalg.norm(force_field)
                r_to_vortex = force_field_magnitude / np.linalg.norm(
                    vortex
                )  # direct distance between the vortex axis and the drone
                if force_field_magnitude != 0 and r_to_vortex != 0:
                    self.v_force_field = (
                        self.v_force_field
                        + np.linalg.norm(vortex)
                        * (1 / R_to_centre)
                        * (1 / r_to_vortex)
                        * force_field
                        / force_field_magnitude
                    )

            self.v_migration = self.points[self.current_path][-1] - np.array(
                swarm_telem[self.id].position_ned
            )  # v_migration here is like a Portional controller to get the drone to the last point
            if np.linalg.norm(self.v_migration) > limit_v_migration:
                self.v_migration = (
                    self.v_migration
                    * limit_v_migration
                    / np.linalg.norm(self.v_migration)
                )

            self.v_lane_cohesion = np.array([0, 0, 0], dtype="float64")
            self.v_rotation = np.array([0, 0, 0], dtype="float64")

        desired_vel = (
            self.k_lane_cohesion * self.v_lane_cohesion
            + self.k_migration * self.v_migration
            + self.k_rotation * self.v_rotation
            + self.k_separation * self.v_separation
            + self.k_force_field * self.v_force_field
        )
        # NOTE maybe add lane cohesion as well so we point the right way when coming from far away
        # yaw = flocking.get_desired_yaw(v_migration[0], v_migration[1])
        yaw_vel = (
            self.k_lane_cohesion * self.v_lane_cohesion
            + self.k_migration * self.v_migration
            + self.k_rotation * self.v_rotation
        )
        # yaw = flocking.get_desired_yaw(yaw_vel[0], yaw_vel[1])
        yaw = 0.0
        output_vel = flocking.check_velocity(
            desired_vel,
            max_speed,
            yaw,
        )
        return output_vel
