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
        self.pass_permission = {} # self.pass_permission[j]: shows the path wchich path j can switch to
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
        self.pass_permission_list = experiment_parameters["pass_permission"]  # self.pass_permission_list[n]: is a dictionary to show switching paths for drone n, it should be empty if there is no switching 
        self.switching_points=experiment_parameters["switching_points"] # self.pass_switching_points[j]: contains all of the points which can switch from path j
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
        self.passed_last_point=[False for j in range(len(self.points))] # to see if the drone has passed the last point of path j or not
        self.create_directions()
        # self.initial_nearest_point(swarm_telem)
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
            for j , next_path in self.pass_permission_list[swarm_priorities.index(self.id)].items():
                self.pass_permission.update({int(j): next_path}) # j here is a string
        
        self.create_adjacent_points()

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
                    if self.repeat[j]=="STAY": # if we want to stay at the last point
                        self.directions[j].append((self.points[j][i] - self.points[j][i-1])/ np.linalg.norm(self.points[j][i] - self.points[j][i-1]))
                    else:    
                        self.directions[j].append((self.points[j][0] - self.points[j][i])/ np.linalg.norm(self.points[j][0] - self.points[j][i]))
                else:
                    self.directions[j].append(
                        (self.points[j][i + 1] - self.points[j][i])
                        / np.linalg.norm(self.points[j][i + 1] - self.points[j][i])
                    )
        print("done creating directions")

    def create_adjacent_points(self) -> None:
        self.adjacent_points = [{} for j in range(len(self.points))]
        for j in self.pass_permission:  # j is the number of path
            if self.pass_permission[j] != None: # if there is a path that path j can switch to
                next_path=self.pass_permission[j]
                for switching_point in self.switching_points[j]:
                    for k in range(
                        len(self.points[next_path]) # self.pass_permission[j] shows the path we can switch from path j
                    ):
                        current_distance = np.linalg.norm(
                            self.points[j][switching_point]
                            - self.points[next_path][k])
                            
                        if (
                            current_distance
                            <= (self.lane_radius[j][switching_point]
                            + self.lane_radius[self.pass_permission[j]][k])*1.001  # this 1 percent is to compensate numerical calculation inaccuracies
    
                        ):
                            pass_vector = (
                                self.points[j][switching_point]
                                - self.points[self.pass_permission[j]][k]
                            )
                            if np.linalg.norm(pass_vector)!=0:
                                pass_vector = pass_vector / np.linalg.norm(pass_vector)
                            
                            self.adjacent_points[j].update(
                                {switching_point: [k, pass_vector]}
                            )  # jth dictionary is {adj. point of path j: [adj. point of next_path, vector from adj. point of path j to adj. point of next_path]}

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
        print(self.id, 'switched from',self.current_path, "at index", self.current_index)
        next_path=self.pass_permission [self.current_path]
        self.pass_permission [self.current_path]= None  # the agent is not allowed to get back to previous path anymore

        self.current_index = self.adjacent_points[self.current_path][
            self.current_index
        ][
            0
        ]  # now current index is a point of the next path
        self.current_path = next_path
        self.target_point = self.points[self.current_path][self.current_index]
        self.target_direction = self.directions[self.current_path][self.current_index]

    def path_following(self, swarm_telem, max_speed, time_step, max_accel):
        self.target_point = self.points[self.current_path][self.current_index]
        self.target_direction = self.directions[self.current_path][self.current_index]
        if (
            self.current_index in self.adjacent_points[self.current_path] # the index is elgible for switching
        ):
            
            pass_vector = self.adjacent_points[self.current_path][self.current_index][1]
            lane_cohesion_position_error = self.target_point - np.array(
                swarm_telem[self.id].position_ned, dtype="float64"
            )
            lane_cohesion_position_error -= (
                np.dot(lane_cohesion_position_error, self.target_direction)
                * self.target_direction
            )
            cos_of_angle = 0
            if abs (np.linalg.norm(pass_vector)) <=0.05 or abs(np.linalg.norm(lane_cohesion_position_error))<=0.05:
                cos_of_angle=1
            else:
                cos_of_angle = np.dot(pass_vector, lane_cohesion_position_error) / (np.linalg.norm(pass_vector)* np.linalg.norm(lane_cohesion_position_error))
            
            if cos_of_angle >= 0.9:
                self.switch()
        # Finding the next bigger Index ----------
        dot_next_point = 0
            
        while dot_next_point >= 0 and not (self.passed_last_point[self.current_path]==True and self.repeat[self.current_path]=="STAY"):  # Searching for farther points, if the drone reach the last point and it has STAY for its path, searching for more points should be stopped
            next_point = index_checker(
                self.current_index +1, self.length[self.current_path]
            )
            range_to_next = (
                np.array(swarm_telem[self.id].position_ned, dtype="float64")
                - self.points[self.current_path][next_point]
            )
            dot_next_point = np.dot(
                range_to_next,
                self.directions[self.current_path][next_point - 1],
            )
            if (dot_next_point>=0):
                self.current_index=next_point
                if (next_point==self.length[self.current_path]-1): # it shows the drone has passed the last point of the current path
                    self.passed_last_point[self.current_path]=True

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
                    - self.lane_radius[self.current_path][self.current_index]
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
        if lane_cohesion_position_error_magnitude==0 or self.lane_radius[self.current_path][self.current_index]==0:
            v_rotation_magnitude=0
        else:
            if lane_cohesion_position_error_magnitude < self.lane_radius[self.current_path][self.current_index]:
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

        # checking the last point of the current path
        if self.passed_last_point[self.current_path]==True and self.repeat[self.current_path]=="STOP":
            self.k_lane_cohesion=0
            self.k_migration =0
            self.k_rotation=0

        elif self.passed_last_point[self.current_path]==True and self.repeat[self.current_path]=="STAY":
            v_migration=0
        
        elif self.passed_last_point[self.current_path]==True and self.repeat[self.current_path]=="REPEAT":
            pass

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
        #yaw = flocking.get_desired_yaw(yaw_vel[0], yaw_vel[1])
        yaw=0.0
        output_vel = flocking.check_velocity(
            desired_vel, swarm_telem[self.id], max_speed, yaw, time_step, max_accel
        )
        return output_vel
