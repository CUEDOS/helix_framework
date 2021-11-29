import sys
import time
import numpy as np
import asyncio
import flocking
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
import pymap3d as pm
import paho.mqtt.client as mqtt
from communication import PosVelNED, DroneCommunication


# Class containing all methods for the drones.
class Agent:
    def __init__(self):
        self.my_pos_vel = PosVelNED()
        self.return_alt = 10

    async def run(self):
        self.drone = System(mavsdk_server_address="localhost", port=CONST_PORT)
        await self.drone.connect()
        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"Drone discovered!")
                break

        self.comms = DroneCommunication(CONST_SWARM_SIZE, CONST_DRONE_ID)
        asyncio.ensure_future(self.comms.run_comms())
        await asyncio.sleep(1)
        asyncio.ensure_future(self.get_position(self.drone))
        asyncio.ensure_future(self.get_velocity(self.drone))

        cmd_loop_duration = 1

        while True:
            command_loop_start_time = time.time()

            if self.comms.current_command == "arm":
                print("ARMING")
                self.home_lat = self.my_pos_vel.geodetic[0]
                self.home_long = self.my_pos_vel.geodetic[1]
                await self.drone.action.arm()
                self.comms.current_command = "none"

            elif self.comms.current_command == "takeoff":
                print("Taking Off")
                await self.takeoff(self.drone)
                self.comms.current_command = "none"

            elif self.comms.current_command == "Simple Flocking":
                await self.offboard(self.drone)

            elif self.comms.current_command == "hold":
                print("Stopping Flocking")
                await self.drone.action.hold()
                self.comms.current_command = "none"

            elif self.comms.current_command == "return":
                print("Returning to home")
                await self.drone.action.hold()
                await self.return_to_home(
                    self.my_pos_vel.geodetic[0], self.my_pos_vel.geodetic[1]
                )
                self.comms.current_command = "none"

            elif self.comms.current_command == "land":
                print("Landing")
                await self.drone.action.land()
                self.comms.current_command = "none"

            # Checking frequency of the loop
            await asyncio.sleep(
                cmd_loop_duration - (time.time() - command_loop_start_time)
            )

    async def takeoff(self, drone):
        await drone.action.set_takeoff_altitude(20)
        await drone.action.takeoff()

    async def offboard(self, drone):
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
            await drone.action.land()
            await drone.action.disarm()
            return
        # End of Init the drone
        offboard_loop_duration = 0.1  # duration of each loop

        await asyncio.sleep(2)
        # Endless loop (Mission)
        while self.comms.current_command == "Simple Flocking":
            offboard_loop_start_time = time.time()

            output_vel = flocking.simple_flocking(
                CONST_DRONE_ID,
                self.comms.swarm_pos_vel,
                self.my_pos_vel,
                CONST_MAX_SPEED,
            )

            # Sending the target velocities to the quadrotor
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(output_vel[0], output_vel[1], output_vel[2], 0.0)
            )

            # Checking frequency of the loop
            await asyncio.sleep(
                offboard_loop_duration - (time.time() - offboard_loop_start_time)
            )

    async def return_to_home(self, rtl_start_lat, rtl_start_long):
        print("send goto")
        print(self.comms.return_alt)
        await self.drone.action.goto_location(
            rtl_start_lat, rtl_start_long, self.comms.return_alt, 0
        )

        while abs(self.my_pos_vel.geodetic[2] - self.comms.return_alt) > 0.5:
            await asyncio.sleep(1)

        await self.drone.action.goto_location(
            self.home_lat, self.home_long, self.comms.return_alt, 0
        )

    # runs in background and upates state class with latest telemetry
    async def get_position(self, drone):
        # set the rate of telemetry updates to 10Hz
        await drone.telemetry.set_rate_position(50)
        async for position in drone.telemetry.position():

            self.my_pos_vel.geodetic = (
                position.latitude_deg,
                position.longitude_deg,
                position.absolute_altitude_m,
            )

            self.my_pos_vel.position_ned = pm.geodetic2ned(
                position.latitude_deg,
                position.longitude_deg,
                position.absolute_altitude_m,
                CONST_REF_LAT,
                CONST_REF_LON,
                CONST_REF_ALT,
            )

            self.comms.client.publish(
                CONST_DRONE_ID + "/telemetry/position_ned",
                str(self.my_pos_vel.position_ned),
            )

            self.comms.client.publish(
                CONST_DRONE_ID + "/telemetry/geodetic",
                str(self.my_pos_vel.geodetic),
            )

    async def get_velocity(self, drone):
        # set the rate of telemetry updates to 10Hz
        # await drone.telemetry.set_rate_position_velocity_ned(10)
        async for position_velocity_ned in drone.telemetry.position_velocity_ned():
            self.my_pos_vel.velocity_ned = [
                position_velocity_ned.velocity.north_m_s,
                position_velocity_ned.velocity.east_m_s,
                position_velocity_ned.velocity.down_m_s,
            ]
            self.comms.client.publish(
                CONST_DRONE_ID + "/telemetry/velocity_ned",
                str(self.my_pos_vel.velocity_ned),
            )


if __name__ == "__main__":

    # Takes command line arguments
    CONST_DRONE_ID = "P" + str(sys.argv[1])
    CONST_SWARM_SIZE = int(sys.argv[2])
    CONST_PORT = int(sys.argv[3])
    CONST_MAX_SPEED = 5

    # below are reference GPS coordinates used as the origin of the NED coordinate system
    CONST_REF_LAT = 53.435053670574646
    CONST_REF_LON = -2.248619912055967
    CONST_REF_ALT = 39

    # Start the main function
    agent = Agent()
    asyncio.ensure_future(agent.run())
    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
