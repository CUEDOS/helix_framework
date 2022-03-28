# from asyncio.windows_events import NULL
import sys
import time
import json
import logging
import asyncio

from tables import Description

# from tokenize import String
# from typing import List
import flocking
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityNedYaw
import pymap3d as pm
from communication import DroneCommunication
from data_structures import AgentTelemetry
import math
import numpy as np


def index_checker(input_index, length) -> int:
    if input_index >= length:
        return int(input_index % length)
    return input_index


class Experiment:
    def __init__(self, drone) -> None:
        self.ready_flag = False
        self.drone = drone
        self.least_distance = 2  # minimum allowed distance between two agents
        # Set up corridor variables
        self.points = []
        self.lane_radius = 0

        # Sensible defaults for gains
        self.k_migration = 1
        self.k_lane_cohesion = 2
        # self.k_rotation = 0.1
        self.k_rotation = 2
        self.k_separation = 2

        self.directions = []
        self.current_index = 0
        self.target_point = np.array([0, 0, 0])
        self.target_direction = np.array([1, 1, 1])

    def set_corridor(self, corridor_json):
        corridor = json.loads(corridor_json)
        self.lane_radius = corridor["corridor_radius"]
        self.points = corridor["corridor_points"]
        print(self.points)
        self.length = len(self.points)
        self.create_directions()
        self.initial_nearest_point()
        self.ready_flag = True
        print("ready")

    def create_directions(self) -> list:
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

    def initial_nearest_point(self) -> int:
        lnitial_least_distance = math.inf
        for i in range(len(self.points)):
            range_to_point_i = np.linalg.norm(
                np.array(agent.my_telem.position_ned) - self.points[i]
            )
            if range_to_point_i <= lnitial_least_distance:
                lnitial_least_distance = range_to_point_i
                self.current_index = i

    # def limit_accelleration(self, desired_vel, current_vel, time_step, max_accel):
    #     delta_v = np.linalg.norm(desired_vel - current_vel)

    #     accelleration = delta_v / time_step

    #     # impose accelleration limit
    #     if accelleration > max_accel:
    #         desired_vel = (
    #             desired_vel
    #             / np.linalg.norm(desired_vel)
    #             * (max_accel * time_step + np.linalg.norm(current_vel))
    #         )

    #     return desired_vel

    def path_following(
        self, drone_id, swarm_telem, my_telem, max_speed, time_step, max_accel
    ):
        print(self.length)
        self.target_point = self.points[self.current_index]
        self.target_direction = self.directions[self.current_index]
        iterator = 0
        # Finding the next bigger Index ----------
        print(self.current_index)
        print(self.length)
        range_to_next = (
            np.array(my_telem.position_ned)
            - self.points[index_checker(self.current_index + 1, self.length)]
        )

        print("current index")
        print(self.current_index)

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
                    np.array(my_telem.position_ned) - self.points[farther_point]
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

        print("v_migration")
        print(v_migration)

        # Calculating lane Cohesion Velocity ---------------
        limit_v_lane_cohesion = 1
        lane_cohesion_position_error = self.target_point - np.array(
            agent.my_telem.position_ned
        )
        lane_cohesion_position_error -= (
            np.dot(lane_cohesion_position_error, self.target_direction)
            * self.target_direction
        )
        lane_cohesion_position_error_magnitude = np.linalg.norm(
            lane_cohesion_position_error
        )
        print("lane cohesion error magnitude")
        print(lane_cohesion_position_error_magnitude)

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

        print("v_lane_cohesion")
        print(v_lane_cohesion)

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

        if (
            np.linalg.norm(
                np.cross(lane_cohesion_position_error, self.target_direction)
            )
            != 0
        ):
            v_rotation = (
                v_rotation_magnitude
                * np.cross(lane_cohesion_position_error, self.target_direction)
                / np.linalg.norm(
                    np.cross(lane_cohesion_position_error, self.target_direction)
                )
            )
        else:
            v_rotation = 0

        if np.linalg.norm(v_rotation) > limit_v_rotation:
            v_rotation = v_rotation * limit_v_rotation / np.linalg.norm(v_rotation)

        print("v_rotation")
        print(v_rotation)

        # Calculating v_separation (normalized) -----------------------------
        limit_v_separation = 1
        r_0 = 2
        v_separation = np.array([0, 0, 0])
        for key in swarm_telem:
            if key == drone_id:
                continue
            p = np.array(swarm_telem[key].position_ned)
            x = np.array(my_telem.position_ned) - p
            d = np.linalg.norm(x)
            if self.least_distance > d:
                self.least_distance = d
            if d <= r_0 and d != 0:
                v_separation = v_separation + ((x / d) * (r_0 - d / r_0))
            if np.linalg.norm(v_separation) > limit_v_separation:
                v_separation = (
                    v_separation * limit_v_separation / np.linalg.norm(v_separation)
                )

        print("v_separation")
        print(v_separation)

        desired_vel = (
            self.k_lane_cohesion * v_lane_cohesion
            + self.k_migration * v_migration
            + self.k_rotation * v_rotation
            + self.k_separation * v_separation
        )
        # output_vel = self.limit_accelleration(
        #     output_vel, np.array(my_telem.velocity_ned), time_step, max_accel
        # )

        print("my position")
        print(my_telem.position_ned)
        print(agent.my_telem.position_ned)

        yaw = flocking.get_desired_yaw(v_migration[0], v_migration[1])

        output_vel = flocking.check_velocity(
            desired_vel, my_telem, max_speed, yaw, time_step, max_accel
        )
        # print("output vel")
        print(output_vel)
        return output_vel


# Class containing all methods for the drones.
class Agent:
    def __init__(self):
        self.my_telem = AgentTelemetry()
        self.return_alt = 10
        if CONST_LOGGING == True:
            self.logger = setup_logger()

    async def run(self):
        self.drone = System(mavsdk_server_address="localhost", port=CONST_PORT)
        await self.drone.connect()
        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"Drone discovered!")
                break

        self.experiment = Experiment(self.drone)

        self.comms = DroneCommunication(
            CONST_REAL_SWARM_SIZE,
            CONST_SITL_SWARM_SIZE,
            CONST_DRONE_ID,
            self.experiment,
        )
        # self.comms = DroneCommunication(
        #     CONST_REAL_SWARM_SIZE, CONST_SITL_SWARM_SIZE, CONST_DRONE_ID
        # )
        asyncio.ensure_future(self.comms.run_comms())

        await asyncio.sleep(1)
        asyncio.ensure_future(self.get_position(self.drone))
        asyncio.ensure_future(self.get_heading(self.drone))
        asyncio.ensure_future(self.get_velocity(self.drone))
        asyncio.ensure_future(self.get_arm_status(self.drone))
        asyncio.ensure_future(self.get_battery_level(self.drone))
        asyncio.ensure_future(self.get_flight_mode(self.drone))

        # Put command callback functions in a dict with command as key
        command_functions = {
            "arm": self.arm,
            "takeoff": self.takeoff,
            "Simple Flocking": self.simple_flocking,
            "Experiment": self.run_experiment,
            "Migration Test": self.migration_test,
            "hold": self.hold,
            "return": self.return_to_home,
            "land": self.land,
            "disconnect": self.on_disconnect,
        }

        # Bind the callbacks
        self.comms.bind_command_functions(command_functions, event_loop)

    async def on_disconnect(self):
        print("connection lost, timeout in 5s")
        await asyncio.sleep(5)
        if self.comms.connected == False:
            # await self.catch_action_error(self.drone.action.hold())
            print("connection lost: logging")
            self.logger.warning("connection lost")

    async def arm(self):
        print("ARMING")
        self.logger.info("arming")

        try:
            await self.drone.action.arm()
            self.home_lat = self.my_telem.geodetic[0]
            self.home_long = self.my_telem.geodetic[1]
        except ActionError as error:
            self.report_error(error._result.result_str)

    async def takeoff(self):
        print("Taking Off")
        self.logger.info("taking-off")
        try:
            await self.drone.action.set_takeoff_altitude(20)
            await self.drone.action.takeoff()
        except ActionError as error:
            self.report_error(error._result.result_str)

    async def hold(self):
        print("Hold")
        self.logger.info("holding")
        try:
            await self.drone.action.hold()
        except ActionError as error:
            self.report_error(error._result.result_str)

    async def land(self):
        print("Landing")
        self.logger.info("landing")
        try:
            await self.drone.action.land()
        except ActionError as error:
            self.report_error(error._result.result_str)

    # async def catch_action_error(self, command):
    #     # Attempts to perform the action and if the command fails the error is reported through MQTT
    #     try:
    #         await command
    #         return True
    #     except ActionError as error:
    #         print("Action Failed: ", error._result.result_str)
    #         self.report_error(error._result.result_str)
    #         self.logger.error("Action Failed: ", error._result.result_str)
    #         return False

    async def start_offboard(self, drone):
        print("-- Setting initial setpoint")
        await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        print("-- Starting offboard")
        try:
            await drone.offboard.start()
        except OffboardError as error:
            print(
                f"Starting offboard mode failed with error code: \
                {error._result.result}"
            )
            print("-- Disarming")
            self.report_error(error._result.result_str)
            self.logger.error("Offboard failed to start: ", error._result.result_str)
            await drone.action.hold()
            print("could not start offboard")
            return

    async def run_experiment(self):
        print("running experiment")
        await self.start_offboard(self.drone)

        # End of Init the drone
        offboard_loop_duration = 0.1  # duration of each loop

        # Loop in which the velocity command outputs are generated
        while (
            self.comms.current_command == "Experiment"
            and self.experiment.ready_flag == True
        ):
            print("generating velocity")
            offboard_loop_start_time = time.time()

            # print(
            #     self.experiment.path_following(
            #         CONST_DRONE_ID,
            #         self.comms.swarm_telemetry,
            #         self.my_telem,
            #         CONST_MAX_SPEED,
            #         offboard_loop_duration,
            #         5,
            #     )
            # )

            await self.drone.offboard.set_velocity_ned(
                self.experiment.path_following(
                    CONST_DRONE_ID,
                    self.comms.swarm_telemetry,
                    self.my_telem,
                    CONST_MAX_SPEED,
                    offboard_loop_duration,
                    5,
                )
            )

            # Checking frequency of the loop
            await asyncio.sleep(
                offboard_loop_duration - (time.time() - offboard_loop_start_time)
            )

    async def simple_flocking(self):
        # pre-swarming process
        swarming_start_lat = self.my_telem.geodetic[0]
        swarming_start_long = self.my_telem.geodetic[1]

        print("Preparing Swarming")
        await self.drone.action.hold()
        await asyncio.sleep(1)

        print("START ALTITUDE:")
        print(self.comms.return_alt)

        try:
            await self.drone.action.goto_location(
                swarming_start_lat, swarming_start_long, self.comms.return_alt, 0
            )
        except ActionError as error:
            self.report_error(error._result.result_str)

        while abs(self.my_telem.geodetic[2] - self.comms.return_alt) > 0.5:
            await asyncio.sleep(1)

        try:
            await self.drone.action.goto_location(
                self.mission_lat, self.mission_long, self.comms.return_alt, 0
            )
        except ActionError as error:
            self.report_error(error._result.result_str)

        await self.start_offboard(self.drone)

        # End of Init the drone
        offboard_loop_duration = 0.1  # duration of each loop

        exp = Experiment(self.drone)

        # Catch points, direction

        # Then calculate nearest point

        # Loop in which the velocity command outputs are generated
        while self.comms.current_command == "Simple Flocking":
            offboard_loop_start_time = time.time()

            output_vel = flocking.simple_flocking(
                CONST_DRONE_ID,
                self.comms.swarm_telemetry,
                self.my_telem,
                offboard_loop_duration,
                5,
            )

            # Sending the target velocities to the quadrotor
            await self.drone.offboard.set_velocity_ned(
                flocking.check_velocity(
                    output_vel,
                    self.my_telem,
                    CONST_MAX_SPEED,
                    0.0,
                    offboard_loop_duration,
                    5,
                )
            )

            # logging the position of each drone in the swarm that this drone has
            for key in self.comms.swarm_telemetry.keys():
                self.logger.info(
                    key + ": " + str(self.comms.swarm_telemetry[key].position_ned)
                )

            # logging the velocity commands sent to the pixhawk
            self.logger.info(
                str(
                    flocking.check_velocity(
                        output_vel,
                        self.my_telem,
                        CONST_MAX_SPEED,
                        0.0,
                        offboard_loop_duration,
                        5,
                    )
                )
            )
            # Checking frequency of the loop
            await asyncio.sleep(
                offboard_loop_duration - (time.time() - offboard_loop_start_time)
            )

    async def migration_test(self):
        await self.start_offboard(self.drone)

        # End of Init the drone
        offboard_loop_duration = 0.1  # duration of each loop

        await asyncio.sleep(2)
        # Endless loop (Mission)
        Migrated = False
        while self.comms.current_command == "Migration Test":
            print("getting new point to migrate to")
            desired_pos = flocking.migration_test(Migrated)
            print(desired_pos)
            while self.comms.current_command == "Migration Test" and (
                abs(self.my_telem.position_ned[0] - desired_pos[0]) > 1
                or abs(self.my_telem.position_ned[1] - desired_pos[1]) > 1
                or abs(self.my_telem.position_ned[2] - desired_pos[2]) > 1
            ):
                offboard_loop_start_time = time.time()

                flocking_vel = flocking.simple_flocking(
                    CONST_DRONE_ID,
                    self.comms.swarm_telemetry,
                    self.my_telem,
                    offboard_loop_duration,
                    1,
                )

                migration_vel, yaw = flocking.velocity_to_point(
                    self.my_telem, desired_pos
                )

                output_vel = flocking_vel + migration_vel

                # Sending the target velocities to the quadrotor
                await self.drone.offboard.set_velocity_ned(
                    flocking.check_velocity(
                        output_vel,
                        self.my_telem,
                        CONST_MAX_SPEED,
                        yaw,
                        offboard_loop_duration,
                        2,
                    )
                )

                # Checking frequency of the loop
                await asyncio.sleep(
                    offboard_loop_duration - (time.time() - offboard_loop_start_time)
                )
            Migrated = not Migrated

    async def return_to_home(self):
        rtl_start_lat = self.my_telem.geodetic[0]
        rtl_start_long = self.my_telem.geodetic[1]

        print("Returning to home")
        await self.drone.action.hold()
        await asyncio.sleep(1)

        print("RETURN ALTITUDE:")
        print(self.comms.return_alt)

        try:
            await self.drone.action.goto_location(
                rtl_start_lat, rtl_start_long, self.comms.return_alt, 0
            )
        except ActionError as error:
            self.report_error(error._result.result_str)

        while abs(self.my_telem.geodetic[2] - self.comms.return_alt) > 0.5:
            await asyncio.sleep(1)

        try:
            await self.drone.action.goto_location(
                self.home_lat, self.home_long, self.comms.return_alt, 0
            )
        except ActionError as error:
            self.report_error(error._result.result_str)

    # runs in background and upates state class with latest telemetry
    async def get_position(self, drone):
        # set the rate of telemetry updates to 10Hz
        await drone.telemetry.set_rate_position(10)
        async for position in drone.telemetry.position():

            self.my_telem.geodetic = (
                position.latitude_deg,
                position.longitude_deg,
                position.absolute_altitude_m,
            )

            self.my_telem.position_ned = pm.geodetic2ned(
                position.latitude_deg,
                position.longitude_deg,
                position.absolute_altitude_m,
                CONST_REF_LAT,
                CONST_REF_LON,
                CONST_REF_ALT,
            )

            self.comms.client.publish(
                CONST_DRONE_ID + "/telemetry/position_ned",
                str(self.my_telem.position_ned).strip("()"),
            )

            self.comms.client.publish(
                CONST_DRONE_ID + "/telemetry/geodetic",
                str(self.my_telem.geodetic).strip("()"),
            )

    async def get_heading(self, drone):
        # set the rate of telemetry updates to 10Hz
        # await drone.telemetry.set_rate_heading(10)
        async for heading in drone.telemetry.heading():

            self.my_telem.heading = heading

            self.comms.client.publish(
                CONST_DRONE_ID + "/telemetry/heading",
                str(self.my_telem.heading.heading_deg).strip("()"),
            )

    async def get_velocity(self, drone):
        # set the rate of telemetry updates to 10Hz
        await drone.telemetry.set_rate_position_velocity_ned(10)
        async for position_velocity_ned in drone.telemetry.position_velocity_ned():
            # changed from list to tuple so formatting for all messages is the same
            self.my_telem.velocity_ned = (
                position_velocity_ned.velocity.north_m_s,
                position_velocity_ned.velocity.east_m_s,
                position_velocity_ned.velocity.down_m_s,
            )
            self.comms.client.publish(
                CONST_DRONE_ID + "/telemetry/velocity_ned",
                str(self.my_telem.velocity_ned).strip("()"),
            )

    async def get_arm_status(self, drone):
        async for arm_status in drone.telemetry.armed():

            if arm_status != self.my_telem.arm_status:
                self.my_telem.arm_status = arm_status
                self.comms.client.publish(
                    CONST_DRONE_ID + "/telemetry/arm_status",
                    str(self.my_telem.arm_status),
                )

    async def get_battery_level(self, drone):
        await drone.telemetry.set_rate_battery(0.1)
        async for battery_level in drone.telemetry.battery():
            self.comms.client.publish(
                CONST_DRONE_ID + "/battery_level",
                str(round(battery_level.remaining_percent * 100)),
            )

    async def get_flight_mode(self, drone):
        previous_flight_mode = "NONE"
        async for flight_mode in drone.telemetry.flight_mode():
            if flight_mode != previous_flight_mode:
                previous_flight_mode = flight_mode
                print(flight_mode)
                self.comms.client.publish(
                    CONST_DRONE_ID + "/flight_mode", str(flight_mode), qos=2
                )

    def report_error(self, error):
        print("Action Failed: ", error)
        self.logger.error("Action Failed: ", error)
        self.comms.client.publish("errors", CONST_DRONE_ID + ": " + error)


def setup_logger():
    log_format = "%(levelname)s %(asctime)s - %(message)s"
    log_date = time.strftime("%d-%m-%y_%H-%M")

    logging.basicConfig(
        filename="logs/" + CONST_DRONE_ID + "_" + log_date + ".log",
        filemode="w",
        format=log_format,
        level=logging.INFO,
    )

    logger = logging.getLogger()
    return logger


if __name__ == "__main__":

    # Takes command line arguments
    CONST_DRONE_ID = str(sys.argv[1])
    CONST_REAL_SWARM_SIZE = int(sys.argv[2])
    CONST_SITL_SWARM_SIZE = int(sys.argv[3])
    CONST_SWARM_SIZE = CONST_REAL_SWARM_SIZE + CONST_SITL_SWARM_SIZE
    CONST_PORT = int(sys.argv[4])
    CONST_LOGGING = bool(
        sys.argv[5]
    )  # 5th argument should be empty for no logging and 'L' for logging enabled
    CONST_MAX_SPEED = 5

    # below are reference GPS coordinates used as the origin of the NED coordinate system

    # For Zurich
    # CONST_REF_LAT = 47.39796
    # CONST_REF_LON = 8.5443076
    # CONST_REF_ALT = 488

    # For Baylands
    # CONST_REF_LAT = 37.413534
    # CONST_REF_LON = -121.996561
    # CONST_REF_ALT = 1.3

    # For Hough End
    CONST_REF_LAT = 53.43578053111544
    CONST_REF_LON = -2.250343561172483
    CONST_REF_ALT = 31

    # Start the main function
    agent = Agent()
    asyncio.ensure_future(agent.run())
    # Runs the event loop until the program is canceled with e.g. CTRL-C
    event_loop = asyncio.get_event_loop()
    event_loop.run_forever()
