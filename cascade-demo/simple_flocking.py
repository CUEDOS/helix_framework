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


my_pos_vel = PosVelNED()

current_command = "none"

# Create dict of PosVelNED objects with drone identifier e.g. P101 as the keys
drone_ids = range(101, 101 + CONST_SWARM_SIZE)
swarm_pos_vel = {}
for i in drone_ids:
    swarm_pos_vel["P" + str(i)] = PosVelNED()

client = mqtt.Client()
# run function (main code)
async def run():
    global current_command
    drone = System(mavsdk_server_address="localhost", port=CONST_PORT)
    await drone.connect()
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered!")
            break

    asyncio.ensure_future(communication())
    asyncio.ensure_future(get_position(drone))
    asyncio.ensure_future(get_velocity(drone))

    cmd_loop_duration = 1

    while True:
        command_loop_start_time = time.time()
        if current_command == "takeoff":
            print("Taking Off")
            await takeoff(drone)
            current_command = "none"
        elif current_command == "Simple Flocking":
            await offboard(drone)
        elif current_command == "stop":
            print("Stopping Flocking")
            await drone.action.hold()
            current_command = "none"
        elif current_command == "land":
            print("Landing")
            await drone.action.land()
            current_command = "none"

        # Checking frequency of the loop
        await asyncio.sleep(cmd_loop_duration - (time.time() - command_loop_start_time))


# End of run function (main code)


async def takeoff(drone):
    await drone.action.set_takeoff_altitude(30)
    await drone.action.arm()
    await drone.action.takeoff()


# puts drone into offboard mode and sends velocities to px4
async def offboard(drone):
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
    while current_command == "start":
        offboard_loop_start_time = time.time()

        # send the position of the drone to the other drones
        client.publish(CONST_DRONE_ID + "/telemetry/position", str(my_pos_vel.position))
        client.publish(CONST_DRONE_ID + "/telemetry/velocity", str(my_pos_vel.velocity))

        output_vel = flocking.simple_flocking(
            CONST_DRONE_ID, swarm_pos_vel, my_pos_vel, CONST_MAX_SPEED
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
async def get_position(drone):
    async for position in drone.telemetry.position():
        my_pos_vel.position = pm.geodetic2ned(
            position.latitude_deg,
            position.longitude_deg,
            position.absolute_altitude_m,
            CONST_REF_LAT,
            CONST_REF_LON,
            CONST_REF_ALT,
        )


async def get_velocity(drone):
    async for position_velocity_ned in drone.telemetry.position_velocity_ned():
        my_pos_vel.velocity = [
            position_velocity_ned.velocity.north_m_s,
            position_velocity_ned.velocity.east_m_s,
            position_velocity_ned.velocity.down_m_s,
        ]


async def communication():
    client.message_callback_add("+/telemetry/position", on_message_position)
    client.message_callback_add("+/telemetry/velocity", on_message_velocity)
    client.message_callback_add("commands", on_message_command)
    client.connect_async("localhost", 1883, 60)  # change localhost to IP of broker
    client.on_connect = on_connect
    client.loop_start()


def on_message_command(mosq, obj, msg):
    print("received command")
    global current_command
    current_command = msg.payload.decode()


# callback triggeed on connection to MQTT
def on_connect(client, userdata, flags, rc):
    print("MQTT connected to broker with result code " + str(rc))
    client.subscribe("+/telemetry/+")
    client.subscribe("commands")


def on_message_position(mosq, obj, msg):
    # Remove none numeric parts of string and then split into north east and down
    received_string = msg.payload.decode().strip("()")
    string_list = received_string.split(", ")
    position = [float(i) for i in string_list]
    swarm_pos_vel[msg.topic[0:4]].position = position


def on_message_velocity(mosq, obj, msg):
    # Remove none numeric parts of string and then split into north east and down
    received_string = msg.payload.decode().strip("[]")
    string_list = received_string.split(", ")
    velocity = [float(i) for i in string_list]
    swarm_pos_vel[msg.topic[0:4]].velocity = velocity


if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())

    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
