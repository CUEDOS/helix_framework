#!/usr/bin/env python3

import asyncio

from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone discovered!")
            break

    print_mission_progress_task = asyncio.ensure_future(
        print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks))

    mission_items = []
    # 1st Waypoint
    mission_items.append(MissionItem(53.480, # Latitude of the 1st waypoint
                                     -2.240, #Longitude of the 1st waypoint
                                     50, #Altitude of the 1st waypoint
                                     5, #Speed to the 1st waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 2nd Waypoint
    mission_items.append(MissionItem(53.480, #Latitude of the 2nd waypoint
                                     -2.239, #Longitude of the 2nd  waypoint
                                     50, #Altitude of the 2nd waypoint
                                     5, #Speed to the 2nd waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 3rd Waypoint
    mission_items.append(MissionItem(53.4795,#Latitude of the 3rd waypoint
                                     -2.239,#Longitude of the 3rd waypoint
                                     50, #Altitude of the 3rd waypoint
                                     5, #Speed to the 3rd waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 4th Waypoint
    mission_items.append(MissionItem(53.4795,#Latitude of the 4th waypoint
                                     -2.240,#Longitude of the 4th waypoint
                                     50, #Altitude of the 4th waypoint
                                     5, #Speed to the 4th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 5th Waypoint
    mission_items.append(MissionItem(53.480,#Latitude of the 5th waypoint
                                     -2.240,#Longitude of the 5th waypoint
                                     50, #Altitude of the 5th waypoint
                                     5, #Speed to the 5th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 6th Waypoint
    mission_items.append(MissionItem(53.479,#Latitude of the 6th waypoint
                                     -2.240,#Longitude of the 6th waypoint
                                     50, #Altitude of the 6th waypoint
                                     5, #Speed to the 6th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 7th Waypoint
    mission_items.append(MissionItem(53.4790,  # Latitude of the 7th waypoint
                                     -2.2375,  # Longitude of the 7th waypoint
                                     50,  # Altitude of the 7th waypoint
                                     5,  # Speed to the 7th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 8th Waypoint
    mission_items.append(MissionItem(53.4790,  # Latitude of the 8th waypoint
                                     -2.2385,  # Longitude of the 8th waypoint
                                     50,  # Altitude of the 8th waypoint
                                     5,  # Speed to the 8th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 9th Waypoint
    mission_items.append(MissionItem(53.4795,  # Latitude of the 9th waypoint
                                     -2.2385,  # Longitude of the 9th waypoint
                                     50,  # Altitude of the 9th waypoint
                                     5,  # Speed to the 9th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 10th Waypoint
    mission_items.append(MissionItem(53.4795,  # Latitude of the 10th waypoint
                                     -2.2375,  # Longitude of the 10th waypoint
                                     50,  # Altitude of the 10th waypoint
                                     5,  # Speed to the 10th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 11th Waypoint
    mission_items.append(MissionItem(53.4795,  # Latitude of the 11th waypoint
                                     -2.2385,  # Longitude of the 11th waypoint
                                     50,  # Altitude of the 11th waypoint
                                     5,  # Speed to the 11th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 12th Waypoint
    mission_items.append(MissionItem(53.480,  # Latitude of the 12th waypoint
                                     -2.2385,  # Longitude of the 12th waypoint
                                     50,  # Altitude of the 12th waypoint
                                     5,  # Speed to the 12th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 13th Waypoint
    mission_items.append(MissionItem(53.480,  # Latitude of the 13th waypoint
                                     -2.2370,  # Longitude of the 13th waypoint
                                     50,  # Altitude of the 13th waypoint
                                     5,  # Speed to the 13th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 14th Waypoint
    mission_items.append(MissionItem(53.4795,  # Latitude of the 14th waypoint
                                     -2.2365,  # Longitude of the 14th waypoint
                                     50,  # Altitude of the 14th waypoint
                                     5,  # Speed to the 14th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 15th Waypoint
    mission_items.append(MissionItem(53.480,  # Latitude of the 15th waypoint
                                     -2.2360,  # Longitude of the 15th waypoint
                                     50,  # Altitude of the 15th waypoint
                                     5,  # Speed to the 15th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 16th Waypoint
    mission_items.append(MissionItem(53.4795,  # Latitude of the 16th waypoint
                                     -2.2365,  # Longitude of the 16th waypoint
                                     50,  # Altitude of the 16th waypoint
                                     5,  # Speed to the 16th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 17th Waypoint
    mission_items.append(MissionItem(53.4790,  # Latitude of the 17th waypoint
                                     -2.2365,  # Longitude of the 17th waypoint
                                     50,  # Altitude of the 17th waypoint
                                     5,  # Speed to the 17th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 18th Waypoint
    mission_items.append(MissionItem(53.4790,  # Latitude of the 18th waypoint
                                     -2.2355,  # Longitude of the 18th waypoint
                                     60,  # Altitude of the 18th waypoint
                                     5,  # Speed to the 18th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 19th Waypoint
    mission_items.append(MissionItem(53.480,  # Latitude of the 19th waypoint
                                     -2.2355,  # Longitude of the 19th waypoint
                                     60,  # Altitude of the 19th waypoint
                                     5,  # Speed to the 19th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 20th Waypoint
    mission_items.append(MissionItem(53.4795,  # Latitude of the 20th waypoint
                                     -2.2350,  # Longitude of the 20th waypoint
                                     60,  # Altitude of the 20th waypoint
                                     5,  # Speed to the 20th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 21st Waypoint
    mission_items.append(MissionItem(53.480,  # Latitude of the 21st waypoint
                                     -2.2345,  # Longitude of the 21st waypoint
                                     60,  # Altitude of the 21st waypoint
                                     5,  # Speed to the 21st waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 22nd Waypoint
    mission_items.append(MissionItem(53.479,  # Latitude of the 22nd waypoint
                                     -2.2345,  # Longitude of the 22nd waypoint
                                     60,  # Altitude of the 22nd waypoint
                                     5,  # Speed to the 22nd waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 23rd Waypoint
    mission_items.append(MissionItem(53.479,  # Latitude of the 23rd waypoint
                                     -2.2340,  # Longitude of the 23rd waypoint
                                     60,  # Altitude of the 23rd waypoint
                                     5,  # Speed to the 23rd waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 24th Waypoint
    mission_items.append(MissionItem(53.480,  # Latitude of the 24th waypoint
                                     -2.2335,  # Longitude of the 24th waypoint
                                     60,  # Altitude of the 24th waypoint
                                     5,  # Speed to the 24th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 25th Waypoint
    mission_items.append(MissionItem(53.479,  # Latitude of the 25th waypoint
                                     -2.2330,  # Longitude of the 25th waypoint
                                     60,  # Altitude of the 25th waypoint
                                     5,  # Speed to the 25th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 26th Waypoint
    mission_items.append(MissionItem(53.4795,  # Latitude of the 26th waypoint
                                     -2.23325,  # Longitude of the 26th waypoint
                                     60,  # Altitude of the 26th waypoint
                                     5,  # Speed to the 26th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 27th Waypoint
    mission_items.append(MissionItem(53.4795,  # Latitude of the 27th waypoint
                                     -2.23375,  # Longitude of the 27th waypoint
                                     60,  # Altitude of the 27th waypoint
                                     5,  # Speed to the 27th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 28th Waypoint
    mission_items.append(MissionItem(53.4795,  # Latitude of the 28th waypoint
                                     -2.2325,  # Longitude of the 28th waypoint
                                     60,  # Altitude of the 28th waypoint
                                     5,  # Speed to the 28th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 29th Waypoint
    mission_items.append(MissionItem(53.4790,  # Latitude of the 29th waypoint
                                     -2.2325,  # Longitude of the 29th waypoint
                                     60,  # Altitude of the 29th waypoint
                                     5,  # Speed to the 29th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 30th Waypoint
    mission_items.append(MissionItem(53.480,  # Latitude of the 30th waypoint
                                     -2.2325,  # Longitude of the 30th waypoint
                                     60,  # Altitude of the 30th waypoint
                                     5,  # Speed to the 30th waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 31st Waypoint
    mission_items.append(MissionItem(53.4790,  # Latitude of the 31st waypoint
                                     -2.2315,  # Longitude of the 31st waypoint
                                     60,  # Altitude of the 31st waypoint
                                     5,  # Speed to the 31st waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))
    # 32nd Waypoint
    mission_items.append(MissionItem(53.480,  # Latitude of the 32nd waypoint
                                     -2.2315,  # Longitude of the 32nd waypoint
                                     60,  # Altitude of the 32nd waypoint
                                     5,  # Speed to the 32nd waypoint
                                     True,
                                     float('nan'),
                                     float('nan'),
                                     MissionItem.CameraAction.NONE,
                                     float('nan'),
                                     float('nan'),
                                     float('nan'),
                                     float('nan')))

    mission_plan = MissionPlan(mission_items)

    await drone.mission.set_return_to_launch_after_mission(True)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("-- Arming")
    await drone.action.arm()

    print("-- Starting mission")
    await drone.mission.start_mission()

    await termination_task


async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
