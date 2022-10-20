from __future__ import annotations  # compatibility with older python versions than 3.9
import sys
import os
import time
import json
import logging
import asyncio
import typing
import flocking
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityNedYaw
import pymap3d as pm
from communication import DroneCommunication
from data_structures import AgentTelemetry
from experiment import Experiment
from csv_logger import CsvFormatter
import math
import gtools
import numpy as np
from telemetry import SwarmManager, TelemetryUpdater


# Class containing all methods for the drones.
class Agent:
    def __init__(self):
        # Open the json file where the config parameters are stored and read them
        print("opening json file")
        with open(CONST_JSON_PATH, "r") as f:
            parameters = json.load(f)
        self.load_parameters(parameters)
        self.swarm_manager = SwarmManager()
        self.swarm_manager.telemetry[self.id] = AgentTelemetry()
        self.current_experiment = "Octobout"
        self.return_alt: float = 10
        if self.logging == True:
            self.logger = setup_logger(self.id)
            self.logger.info("ref lat: " + str(self.ref_lat))
            self.logger.info("ref lon: " + str(self.ref_lon))
            self.logger.info("ref alt: " + str(self.ref_alt))
        print("setup done")

    async def run(self):
        self.drone: type[System] = System(
            mavsdk_server_address="localhost", port=self.port
        )
        await self.drone.connect()

        # self.drone: type[System] = System()
        # await self.drone.connect(system_address=self.serial_address)

        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"Drone discovered!")
                break

        self.comms = DroneCommunication(
            self,
            self.swarm_manager,
        )
        asyncio.ensure_future(self.comms.run_comms())
        await asyncio.sleep(2)
        # Put command callback functions in a dict with command as key
        command_functions = {
            "arm": self.arm,
            "takeoff": self.takeoff,
            "Experiment": self.run_experiment,
            "pre_start": self.pre_start,
            "hold": self.hold,
            "return": self.return_to_home,
            "land": self.land,
            "disconnect": self.on_disconnect,
        }

        # Bind the callbacks
        self.comms.bind_command_functions(command_functions, event_loop)
        self.telemetry_updater = TelemetryUpdater(
            self.id,
            self.drone,
            self.comms.client,
            self.swarm_manager.telemetry,
            event_loop,
            [self.ref_lat, self.ref_lon, self.ref_alt],
            self.download_ulog,
        )

    async def on_disconnect(self):
        print("connection lost, timeout in 5s")
        await asyncio.sleep(5)
        if self.comms.connected == False:
            # await self.catch_action_error(self.drone.action.hold())
            print("connection lost: logging")
            if self.logging == True:
                self.logger.warning("connection lost")

    def load_parameters(self, parameters):
        # takes dict of parameters and loads them into variables
        self.id: str = parameters["id"]
        self.broker_ip: str = parameters["broker_ip"]
        self.port: int = parameters["port"]
        self.serial_address: str = parameters["serial_address"]
        self.logging: bool = parameters["logging"]
        self.max_speed: int = parameters["max_speed"]
        self.ref_lat: float = parameters["ref_lat"]
        self.ref_lon: float = parameters["ref_lon"]
        self.ref_alt: float = parameters["ref_alt"]

    def update_parameter(self, new_parameters_json):

        new_parameters = json.loads(new_parameters_json)

        # load old parameters and insert new ones
        with open(CONST_JSON_PATH, "r") as f:
            parameters = json.load(f)

        for key in new_parameters.keys():
            parameters[key] = new_parameters[key]
        # write new parameters to file
        with open(CONST_JSON_PATH, "w") as f:
            json.dump(parameters, f)

        self.load_parameters(parameters)

    async def arm(self):
        print("ARMING")
        if self.logging == True:
            self.logger.info("arming")
        try:
            await self.drone.action.arm()
            self.home_lat = self.swarm_manager.telemetry[self.id].geodetic[0]
            self.home_long = self.swarm_manager.telemetry[self.id].geodetic[1]
        except ActionError as error:
            self.report_error(error._result.result_str)

    async def takeoff(self):
        print("Taking Off")
        if self.logging == True:
            self.logger.info("taking-off")
        try:
            await self.drone.action.set_takeoff_altitude(20)
            await self.drone.action.takeoff()
        except ActionError as error:
            self.report_error(error._result.result_str)

    async def hold(self):
        print("Hold")
        if self.logging == True:
            self.logger.info("holding")
        try:
            await self.drone.action.hold()
        except ActionError as error:
            self.report_error(error._result.result_str)

    async def land(self):
        print("Landing")
        if self.logging == True:
            self.logger.info("landing")
        try:
            await self.drone.action.land()
        except ActionError as error:
            self.report_error(error._result.result_str)

    async def deconflicted_goto(self, desired_positions_ned, deconflicted_alt_dict):
        # TODO make sure calling hold during process works
        start_lat = self.swarm_manager.telemetry[self.id].geodetic[0]
        start_lon = self.swarm_manager.telemetry[self.id].geodetic[1]
        travel_alt = deconflicted_alt_dict[self.id]

        await self.drone.action.hold()
        await asyncio.sleep(1)

        (desired_lat, desired_lon, desired_alt) = pm.ned2geodetic(
            desired_positions_ned[self.id][0],
            desired_positions_ned[self.id][1],
            desired_positions_ned[self.id][2],
            self.ref_lat,
            self.ref_lon,
            self.ref_alt,
        )

        # Go to the deconflicted travel altitude
        try:
            await self.drone.action.goto_location(start_lat, start_lon, travel_alt, 0)
        except ActionError as error:
            self.report_error(error._result.result_str)

        # wait until altitude is reached by all agents
        # while not self.swarm_manager.check_swarm_altitudes(deconflicted_alt_dict):
        #     await asyncio.sleep(0.1)
        while abs(self.swarm_manager.telemetry[self.id].geodetic[2] - travel_alt) > 0.5:
            await asyncio.sleep(1)
        await asyncio.sleep(10)

        # Go to the desired position at the travel alt
        try:
            await self.drone.action.goto_location(
                desired_lat, desired_lon, travel_alt, 0
            )
        except ActionError as error:
            self.report_error(error._result.result_str)

        while (
            abs(
                self.swarm_manager.telemetry[self.id].position_ned[0]
                - desired_positions_ned[self.id][0]
            )
            > 1
            or abs(
                self.swarm_manager.telemetry[self.id].position_ned[1]
                - desired_positions_ned[self.id][1]
            )
            > 1
        ):
            await asyncio.sleep(1)

        # Waits until position is reached by all agents
        # while not self.swarm_manager.check_swarm_positions(
        #     desired_positions_ned, check_alt=False
        # ):
        #     await asyncio.sleep(0.1)
        await asyncio.sleep(10)

        # finally go to the desired altitude
        try:
            await self.drone.action.goto_location(
                desired_lat, desired_lon, desired_alt, 0
            )
        except ActionError as error:
            self.report_error(error._result.result_str)

        # Waits until position is reached by all agents
        # while not self.swarm_manager.check_swarm_positions(desired_positions_ned):
        #     await asyncio.sleep(0.1)
        # await asyncio.sleep(1)

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
            if self.logging == True:
                self.logger.error(
                    "Offboard failed to start: ", error._result.result_str
                )
            await drone.action.hold()
            print("could not start offboard")
            return

    async def pre_start(self):
        # temp
        experiment_file_path: str = "experiments/" + self.current_experiment + ".json"

        print("experiments/" + self.current_experiment + ".json")
        self.experiment = Experiment(
            self.id, self.swarm_manager.telemetry, experiment_file_path
        )

        await asyncio.sleep(1)

        alt_dict = {}
        # get the intiial point and the intiial path
        swarm_priorities = self.experiment.get_swarm_priorities(
            self.swarm_manager.telemetry
        )
        self.experiment.get_path_and_permission(swarm_priorities)
        pre_start_positions = self.experiment.get_pre_start_positions(
            self.swarm_manager.telemetry, swarm_priorities
        )

        for key in self.swarm_manager.telemetry.keys():
            alt_dict[key] = self.swarm_manager.telemetry[key].geodetic[2]

        # TODO add check that all agents calculate the same alts
        deconflicted_alt_dict = gtools.alt_calc(alt_dict, self.ref_alt)

        # TODO add check if pre start position is current position
        await self.deconflicted_goto(pre_start_positions, deconflicted_alt_dict)

        # once in pre start position find the intiial nearest point
        self.experiment.initial_nearest_point(self.swarm_manager.telemetry)

    async def run_experiment(self):
        print("Starting experiment")
        await self.start_offboard(self.drone)

        # End of Init the drone
        offboard_loop_duration = 0.1  # duration of each loop

        # Loop in which the velocity command outputs are generated
        self.experiment.start_time = self.swarm_manager.telemetry[self.id].current_time
        # Calling method path_following
        while (
            self.comms.current_command == "Experiment"
            and self.experiment.ready_flag == True
        ):
            offboard_loop_start_time = time.time()

            await self.drone.offboard.set_velocity_ned(
                self.experiment.path_following(
                    self.swarm_manager.telemetry,
                    self.max_speed,
                    offboard_loop_duration,
                    10,
                )
            )

            await self.check_altitude()

            # Checking frequency of the loop
            await asyncio.sleep(
                offboard_loop_duration - (time.time() - offboard_loop_start_time)
            )

    async def return_to_home(self):
        rtl_start_lat = self.swarm_manager.telemetry[self.id].geodetic[0]
        rtl_start_long = self.swarm_manager.telemetry[self.id].geodetic[1]

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

        while (
            abs(
                self.swarm_manager.telemetry[self.id].geodetic[2]
                - self.comms.return_alt
            )
            > 0.5
        ):
            await asyncio.sleep(1)

        try:
            await self.drone.action.goto_location(
                self.home_lat, self.home_long, self.comms.return_alt, 0
            )
        except ActionError as error:
            self.report_error(error._result.result_str)

    async def check_altitude(self):
        top_alt_limit = 120.0
        bottom_alt_limit = 5.0
        if self.swarm_manager.telemetry[self.id].flight_mode == "OFFBOARD" and (
            -self.swarm_manager.telemetry[self.id].position_ned[2] <= bottom_alt_limit
            or -self.swarm_manager.telemetry[self.id].position_ned[2] >= top_alt_limit
        ):
            print("OUTSIDE ALTITUDE LIMITS")
            await self.hold()
            self.comms.current_command = "hold"
            for agent in self.swarm_manager.telemetry.keys():
                self.comms.client.publish("commands/" + agent, "hold")

    def report_error(self, error):
        print(error)
        self.logger.error(error)

    async def download_ulog(self):
        entries = await self.drone.log_files.get_entries()
        entry = entries[-1]
        date_without_colon = entry.date.replace(":", "-")
        filename = f"{os.getcwd()}/px4_logs/log-{date_without_colon}.ulog"
        print(f"Downloading: log {entry.id} from {entry.date} to {filename}")
        previous_progress = -1
        async for progress in self.drone.log_files.download_log_file(entry, filename):
            new_progress = round(progress.progress * 100)
            if new_progress != previous_progress:
                sys.stdout.write(f"\r{new_progress} %")
                sys.stdout.flush()
                previous_progress = new_progress
        print()


def setup_logger(id):
    log_date = time.strftime("%d-%m-%y_%H-%M")

    logging.basicConfig(
        filename="logs/" + id + "_" + log_date + ".log",
        filemode="w",
        level=logging.INFO,
    )

    logger = logging.getLogger()
    logging.root.handlers[0].setFormatter(CsvFormatter())
    return logger


if __name__ == "__main__":

    # Takes command line arguments
    # CONST_DRONE_ID = str(sys.argv[1])
    # CONST_REAL_SWARM_SIZE = int(sys.argv[2])
    # CONST_SITL_SWARM_SIZE = int(sys.argv[3])
    # CONST_SWARM_SIZE = CONST_REAL_SWARM_SIZE + CONST_SITL_SWARM_SIZE
    # CONST_PORT = int(sys.argv[4])
    # CONST_LOGGING = bool(
    #    sys.argv[5]
    # )  # 5th argument should be empty for no logging and 'L' for logging enabled
    # CONST_MAX_SPEED = 5

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
    # CONST_REF_LAT = 53.43578053111544
    # CONST_REF_LON = -2.250343561172483
    # CONST_REF_ALT = 31

    CONST_JSON_PATH = str(sys.argv[1])
    # CONST_JSON_PATH = "parameters.json"
    # Start the main function
    agent = Agent()
    asyncio.ensure_future(agent.run())
    # Runs the event loop until the program is canceled with e.g. CTRL-C
    event_loop = asyncio.get_event_loop()
    event_loop.run_forever()
