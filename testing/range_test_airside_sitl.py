import time
import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityNedYaw
import paho.mqtt.client as mqtt


class drone_telemetry:
    qx = 0  # position of the quadrotor in x direction with respect to initial point
    qy = 0  # position of the quadrotor in y direction with respect to initial point
    qz = 0  # position of the quadrotor in z direction with respect to initial point


async def run():
    mqtt_topic = "range_test"
    broker_IP = "localhost"
    client = mqtt.Client()
    client.connect(broker_IP, 1883, 60)
    # Init the drone
    drone = System()
    await drone.connect()
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered!")
            break

    # Start background task
    asyncio.ensure_future(get_position(drone))
    # End of Init the drone
    # Endless loop
    while 1:
        start_time = time.time()
        client.publish(
            mqtt_topic,
            str(time.time())
            + ","
            + str(drone_telemetry.qx)
            + ","
            + str(drone_telemetry.qy)
            + ","
            + str(drone_telemetry.qz),
        )

        # Checking frequency of the loop
        await asyncio.sleep(
            0.1 - (time.time() - start_time)
        )  # to make while 1 work at 10 Hz
        print("loop duration=", (time.time() - start_time))
    # End of Endless loop


# runs in background and upates state class with latest telemetry
async def get_position(drone):
    async for position_velocity_ned in drone.telemetry.position_velocity_ned():
        drone_telemetry.qx = position_velocity_ned.position.north_m
        drone_telemetry.qy = position_velocity_ned.position.east_m
        drone_telemetry.qz = position_velocity_ned.position.down_m


if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())

    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
