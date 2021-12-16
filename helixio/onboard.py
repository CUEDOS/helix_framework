import sys
import time
import logging
import numpy as np
import asyncio
import flocking
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityNedYaw
import pymap3d as pm
import paho.mqtt.client as mqtt
from communication import AgentTelemetry, DroneCommunication


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

        self.comms = DroneCommunication(
            CONST_REAL_SWARM_SIZE, CONST_SITL_SWARM_SIZE, CONST_DRONE_ID
        )
        asyncio.ensure_future(self.comms.run_comms())
        await asyncio.sleep(1)
        asyncio.ensure_future(self.get_position(self.drone))
        asyncio.ensure_future(self.get_velocity(self.drone))
        asyncio.ensure_future(self.get_arm_status(self.drone))

        # Put command callback functions in a dict with command as key
        command_functions = {
            "arm": self.arm,
            "takeoff": self.takeoff,
            "Simple Flocking": self.simple_flocking,
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
            await self.catch_action_error(self.drone.action.hold())
            print("connection lost: holding")

    async def arm(self):
        print("ARMING")
        self.logger.info("arming")
        # if await self.catch_action_error(self.drone.action.arm()):
        if await self.catch_action_error(self.drone.action.arm()):
            self.home_lat = self.my_telem.geodetic[0]
            self.home_long = self.my_telem.geodetic[1]

    async def takeoff(self):
        print("Taking Off")
        await self.catch_action_error(self.drone.action.set_takeoff_altitude(20))
        self.logger.info("taking-off")
        await self.catch_action_error(self.drone.action.takeoff())

    async def hold(self):
        print("Hold")
        self.logger.info("holding")
        await self.catch_action_error(self.drone.action.hold())

    async def land(self):
        print("Landing")
        self.logger.info("landing")
        await self.catch_action_error(self.drone.action.land())

    async def catch_action_error(self, command):
        # Attempts to perform the action and if the command fails the error is reported through MQTT
        try:
            await command
            return True
        except ActionError as error:
            print("Action Failed: ", error._result.result_str)
            self.report_error(error._result.result_str)
            self.logger.error("Action Failed: ", error._result.result_str)
            return False

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

    async def simple_flocking(self):
        await self.start_offboard(self.drone)

        # End of Init the drone
        offboard_loop_duration = 0.1  # duration of each loop

        # commandting out to see result
        # await asyncio.sleep(2)
        # Endless loop (Mission)
        while self.comms.current_command == "Simple Flocking":
            offboard_loop_start_time = time.time()

            output_vel = flocking.simple_flocking(
                CONST_DRONE_ID,
                self.comms.swarm_telemetry,
                self.my_telem,
                offboard_loop_duration,
                1,
            )

            # Sending the target velocities to the quadrotor
            await self.drone.offboard.set_velocity_ned(
                flocking.check_velocity(
                    output_vel,
                    self.my_telem,
                    CONST_MAX_SPEED,
                    0.0,
                    offboard_loop_duration,
                    1,
                )
            )
            self.logger.info(
                str(
                    flocking.check_velocity(
                        output_vel,
                        self.my_telem,
                        CONST_MAX_SPEED,
                        0.0,
                        offboard_loop_duration,
                        1,
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

        print(self.comms.return_alt)

        await self.catch_action_error(
            self.drone.action.goto_location(
                rtl_start_lat, rtl_start_long, self.comms.return_alt, 0
            )
        )

        while abs(self.my_telem.geodetic[2] - self.comms.return_alt) > 0.5:
            await asyncio.sleep(1)

        await self.catch_action_error(
            self.drone.action.goto_location(
                self.home_lat, self.home_long, self.comms.return_alt, 0
            )
        )

    # runs in background and upates state class with latest telemetry
    async def get_position(self, drone):
        # set the rate of telemetry updates to 50Hz
        await drone.telemetry.set_rate_position(50)
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
                str(self.my_telem.position_ned),
            )

            self.comms.client.publish(
                CONST_DRONE_ID + "/telemetry/geodetic",
                str(self.my_telem.geodetic),
            )

    async def get_velocity(self, drone):
        # set the rate of telemetry updates to 10Hz
        # await drone.telemetry.set_rate_position_velocity_ned(10)
        async for position_velocity_ned in drone.telemetry.position_velocity_ned():
            self.my_telem.velocity_ned = [
                position_velocity_ned.velocity.north_m_s,
                position_velocity_ned.velocity.east_m_s,
                position_velocity_ned.velocity.down_m_s,
            ]
            self.comms.client.publish(
                CONST_DRONE_ID + "/telemetry/velocity_ned",
                str(self.my_telem.velocity_ned),
            )

    async def get_arm_status(self, drone):
        async for arm_status in drone.telemetry.armed():

            if arm_status != self.my_telem.arm_status:
                self.my_telem.arm_status = arm_status
                self.comms.client.publish(
                    CONST_DRONE_ID + "/telemetry/arm_status",
                    str(self.my_telem.arm_status),
                )

    def report_error(self, error):
        self.comms.client.publish("errors", CONST_DRONE_ID + ": " + error)


def setup_logger():
    log_format = "%(levelname)s %(asctime)s - %(message)s"

    logging.basicConfig(
        filename="logfile.log", filemode="w", format=log_format, level=logging.INFO
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
