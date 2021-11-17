import sys
import time
import numpy as np
import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
import pymap3d as pm
import paho.mqtt.client as mqtt

# CONST_DRONE_ID = "P" + str(sys.argv[1])
# CONST_SWARM_SIZE = int(sys.argv[2])
# CONST_PORT = int(sys.argv[3])
CONST_DRONE_ID = "P101"
CONST_SWARM_SIZE = 1
CONST_PORT = 50041
CONST_MAX_SPEED = 5

# below are reference GPS coordinates used as the origin of the NED coordinate system
CONST_REF_LAT = 53.473489655102014
CONST_REF_LON = -2.2354534026550343
CONST_REF_ALT = 0


class PosVelNED:
    position = [0, 0, 0]
    velocity = [0, 0, 0]


my_pos_vel = PosVelNED()

# Create dict of PosVelNED objects with drone identifier e.g. P101 as the keys
drone_ids = range(101, 101 + CONST_SWARM_SIZE)
swarm_pos_vel = {}
for i in drone_ids:
    swarm_pos_vel["P" + str(i)] = PosVelNED()

client = mqtt.Client()
drone = System(mavsdk_server_address="localhost", port=CONST_PORT)
# run function (main code)
async def run():

    # drone = System(mavsdk_server_address="localhost", port=CONST_PORT)
    await drone.connect()
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered!")
            break

    asyncio.ensure_future(communication(drone))
    asyncio.ensure_future(get_position(drone))
    asyncio.ensure_future(get_velocity(drone))


# End of run function (main code)

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


async def takeoff(drone):
    await drone.action.arm()
    await drone.action.takeoff()


async def land(drone):
    await drone.action.land()
    await drone.action.disarm()


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
        await drone.action.disarm()
        return
    # End of Init the drone
    loop_duration = 0.1  # duration of each loop
    # take off the quadrotor
    await asyncio.sleep(2)
    # Endless loop (Mission)
    while True:
        loop_start_time = time.time()
        # print(my_telemetry.position)
        client.publish(CONST_DRONE_ID + "/telemetry/position", str(my_pos_vel.position))
        client.publish(CONST_DRONE_ID + "/telemetry/velocity", str(my_pos_vel.velocity))

        output_vel = simple_flocking()

        # Sending the target velocities to the quadrotor
        await drone.offboard.set_velocity_ned(
            VelocityNedYaw(output_vel[0], output_vel[1], output_vel[2], 0.0)
        )

        # Checking frequency of the loop
        await asyncio.sleep(loop_duration - (time.time() - loop_start_time))


# class CmdHandler:
#     def __init__(self, drone):
#         self.drone = drone
#         self.running_task = 0

#     def on_message_command(self, mosq, obj, msg):
#         print("got here")
#         if msg.payload.decode() == "takeoff":
#             print("Taking Off")
#             self.running_task = asyncio.run(takeoff(self.drone))
#         elif msg.payload.decode() == "start":
#             print("Starting Flocking")
#             self.running_task = asyncio.run(offboard(self.drone))
#         elif msg.payload.decode() == "stop":
#             print("Stopping Flocking")
#             self.running_task.cancel()
#         elif msg.payload.decode() == "land":
#             print("Landing")
#             self.running_task = asyncio.run(land(self.drone))


async def communication(drone):
    # handler = CmdHandler(drone)
    client.message_callback_add("+/telemetry/position", on_message_position)
    client.message_callback_add("+/telemetry/velocity", on_message_velocity)
    client.message_callback_add("commands", on_message_command)
    # client.message_callback_add("commands", on_message_command)
    client.connect_async("localhost", 1883, 60)  # change localhost to IP of broker
    client.on_connect = on_connect
    client.loop_start()


# running_task = None
def on_message_command(mosq, obj, msg):
    print("received message")
    print(msg.payload.decode())

    async def run_command():
        if msg.payload.decode() == "takeoff":
            print("Taking Off")
            # running_task = asyncio.run(takeoff(drone))
            await takeoff(drone)
        elif msg.payload.decode() == "start":
            print("Starting Flocking")
            # running_task = asyncio.run(offboard(drone))
            asyncio.get_event_loop().run(offboard(drone))
        elif msg.payload.decode() == "stop":
            print("Stopping Flocking")
            # running_task.cancel()
            asyncio.get_event_loop().run(land(drone))
        elif msg.payload.decode() == "land":
            print("Landing")
            # running_task = asyncio.run(land(drone))
            asyncio.get_event_loop().run(land(drone))

    asyncio.run(run_command())


def simple_flocking():
    com = np.array([0, 0, 0])
    k_cohesion = 1
    for key in swarm_pos_vel:
        p = np.array(swarm_pos_vel[key].position)
        com = com + p
    com = com / len(swarm_pos_vel)
    v_cohesion = (
        k_cohesion
        * (com - np.array(my_pos_vel.position))
        / np.linalg.norm(com - np.array(my_pos_vel.position))
    )

    r_0 = 10
    v_separation = np.array([0, 0, 0])
    for key in swarm_pos_vel:
        if key == CONST_DRONE_ID:
            continue
        p = np.array(swarm_pos_vel[key].position)
        x = np.array(my_pos_vel.position) - p
        d = np.linalg.norm(x)
        v_separation = v_separation + ((x / d) * (r_0 - d) / r_0)

    output_vel = v_cohesion + v_separation
    if np.linalg.norm(output_vel) > CONST_MAX_SPEED:
        output_vel = output_vel / np.linalg.norm(output_vel) * CONST_MAX_SPEED

    return output_vel.tolist()


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
