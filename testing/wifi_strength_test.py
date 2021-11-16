import rssi
import csv
import time
import asyncio
from mavsdk import System
from datetime import datetime


class drone_telemetry:
    qx = 0  # position of the quadrotor in x direction with respect to initial point
    qy = 0  # position of the quadrotor in y direction with respect to initial point
    qz = 0  # position of the quadrotor in z direction with respect to initial point


async def run():
    # run ifconfig to find interface
    csv_file = "wifi_strength_data.csv"
    interface = "wlp0s20f3"
    rssi_scanner = rssi.RSSI_Scan(interface)
    ssids = ["skynet"]

    now = datetime.now()
    date_time_list = ["Test on: ", now.strftime("%d/%m/%Y %H:%M:%S")]

    ap_info = rssi_scanner.getAPinfo(networks=ssids, sudo=True)

    with open(csv_file, "a", encoding="UTF8") as f:
        writer = csv.writer(f)
        writer.writerow(date_time_list)

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

        signal_strength = []
        data_line = []
        for i in range(0, len(ap_info)):
            signal_strength.append(int(ap_info[i]["signal"]))

        # take the channel with best signal strength and convert to a percentage
        rssi_level = (max(signal_strength) + 110) * 10 / 7
        rssi_level = [round(rssi_level, 2)]

        drone_position = [drone_telemetry.qx, drone_telemetry.qy, drone_telemetry.qz]
        data_line = drone_position + rssi_level + signal_strength
        data_line = [str(x) for x in data_line]
        print(data_line)

        with open(csv_file, "a", encoding="UTF8") as f:
            writer = csv.writer(f)
            writer.writerow(data_line)

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
