import sys
import time
import numpy as np
import asyncio
import flocking
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
import pymap3d as pm
import paho.mqtt.client as mqtt

# takes command line arguments
CONST_DRONE_ID = "P" + str(sys.argv[1])
CONST_SWARM_SIZE = int(sys.argv[2])
CONST_PORT = int(sys.argv[3])
CONST_MAX_SPEED = 5

# below are reference GPS coordinates used as the origin of the NED coordinate system
CONST_REF_LAT = 53.473489655102014
CONST_REF_LON = -2.2354534026550343
CONST_REF_ALT = 0


class PosVelNED:
    position = [0, 0, 0]
    velocity = [0, 0, 0]


class Agent:
    current_command = "none"
    drone_ids = range(101, 101 + CONST_SWARM_SIZE)
    swarm_pos_vel = {}
    for i in drone_ids:
        swarm_pos_vel["P" + str(i)] = PosVelNED()

    def __init__(self):
        self.my_pos_vel = PosVelNED()

        # Create dict of PosVelNED objects with drone identifier e.g. P101 as the keys
        drone_ids = range(101, 101 + CONST_SWARM_SIZE)

    async def run(self):
        self.drone = System(mavsdk_server_address="localhost", port=CONST_PORT)
        await self.drone.connect()
        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"Drone discovered!")
                break

        comms = Communication()
        asyncio.ensure_future(comms.run_comms())
        asyncio.ensure_future(self.get_position(self.drone))
        asyncio.ensure_future(self.get_velocity(self.drone))

        cmd_loop_duration = 1

        while True:
            command_loop_start_time = time.time()
            if Agent.current_command == "takeoff":
                print("Taking Off")
                await self.takeoff(self.drone)
                Agent.current_command = "none"
            elif self.current_command == "Simple Flocking":
                await self.offboard(self.drone)
            elif Agent.current_command == "hold":
                print("Stopping Flocking")
                await self.drone.action.hold()
                Agent.current_command = "none"
            elif Agent.current_command == "land":
                print("Landing")
                await self.drone.action.land()
                Agent.current_command = "none"

            # Checking frequency of the loop
            await asyncio.sleep(
                cmd_loop_duration - (time.time() - command_loop_start_time)
            )

    async def takeoff(self, drone):
        await drone.action.set_takeoff_altitude(20)
        await drone.action.arm()
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
        # take off the quadrotor
        await asyncio.sleep(2)
        # Endless loop (Mission)
        while Agent.current_command == "Simple Flocking":
            offboard_loop_start_time = time.time()

            # send the position of the drone to the other drones
            Communication.client.publish(
                CONST_DRONE_ID + "/telemetry/position", str(self.my_pos_vel.position)
            )
            Communication.client.publish(
                CONST_DRONE_ID + "/telemetry/velocity", str(self.my_pos_vel.velocity)
            )

            output_vel = flocking.simple_flocking(
                CONST_DRONE_ID, Agent.swarm_pos_vel, self.my_pos_vel, CONST_MAX_SPEED
            )

            # Sending the target velocities to the quadrotor
            await drone.offboard.set_velocity_ned(
                VelocityNedYaw(output_vel[0], output_vel[1], output_vel[2], 0.0)
            )

            # Checking frequency of the loop
            await asyncio.sleep(
                offboard_loop_duration - (time.time() - offboard_loop_start_time)
            )

    # runs in background and upates state class with latest telemetry
    async def get_position(self, drone):
        async for position in drone.telemetry.position():
            self.my_pos_vel.position = pm.geodetic2ned(
                position.latitude_deg,
                position.longitude_deg,
                position.absolute_altitude_m,
                CONST_REF_LAT,
                CONST_REF_LON,
                CONST_REF_ALT,
            )

    async def get_velocity(self, drone):
        async for position_velocity_ned in drone.telemetry.position_velocity_ned():
            self.my_pos_vel.velocity = [
                position_velocity_ned.velocity.north_m_s,
                position_velocity_ned.velocity.east_m_s,
                position_velocity_ned.velocity.down_m_s,
            ]


class Communication:
    client = mqtt.Client()

    # def __init__(self):

    async def run_comms(self):
        Communication.client.message_callback_add(
            "+/telemetry/position", self.on_message_position
        )
        Communication.client.message_callback_add(
            "+/telemetry/velocity", self.on_message_velocity
        )
        Communication.client.message_callback_add("commands", self.on_message_command)
        Communication.client.connect_async(
            "localhost", 1883, 60
        )  # change localhost to IP of broker
        Communication.client.on_connect = self.on_connect
        Communication.client.loop_start()

    def on_message_command(self, mosq, obj, msg):
        print("received command")
        Agent.current_command = msg.payload.decode()

    # callback triggeed on connection to MQTT
    def on_connect(self, client, userdata, flags, rc):
        print("MQTT connected to broker with result code " + str(rc))
        client.subscribe("+/telemetry/+")
        client.subscribe("commands")

    def on_message_position(self, mosq, obj, msg):
        # Remove none numeric parts of string and then split into north east and down
        received_string = msg.payload.decode().strip("()")
        string_list = received_string.split(", ")
        position = [float(i) for i in string_list]
        # time.sleep(1)  # simulating comm latency
        Agent.swarm_pos_vel[msg.topic[0:4]].position = position

    def on_message_velocity(self, mosq, obj, msg):
        # Remove none numeric parts of string and then split into north east and down
        received_string = msg.payload.decode().strip("[]")
        string_list = received_string.split(", ")
        velocity = [float(i) for i in string_list]
        # time.sleep(1)  # simulating comm latency
        Agent.swarm_pos_vel[msg.topic[0:4]].velocity = velocity


if __name__ == "__main__":
    # Start the main function
    agent = Agent()
    asyncio.ensure_future(agent.run())

    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
