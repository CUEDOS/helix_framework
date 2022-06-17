from __future__ import annotations  # compatibility with older python versions than 3.9
import asyncio
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, VelocityNedYaw
import pymap3d as pm
from data_structures import AgentTelemetry
import numpy as np


class SwarmManager:
    def __init__(self):
        self.telemetry: dict[str, type[AgentTelemetry]] = {}

    def check_swarm_positions(self, required_positions, check_alt=True):
        # takes required positions as NED and checks positions of swarm
        if check_alt == True:
            for agent in self.telemetry.keys():
                if (
                    np.sqrt(
                        (
                            required_positions[agent][0]
                            - self.telemetry[agent].position_ned[0]
                        )
                        ** 2
                        + (
                            required_positions[agent][1]
                            - self.telemetry[agent].position_ned[1]
                        )
                        ** 2
                        + (
                            required_positions[agent][2]
                            - self.telemetry[agent].position_ned[2]
                        )
                        ** 2
                    )
                ) > 0.3:
                    return False
        else:
            for agent in self.telemetry.keys():
                if (
                    np.sqrt(
                        (
                            required_positions[agent][0]
                            - self.telemetry[agent].position_ned[0]
                        )
                        ** 2
                        + (
                            required_positions[agent][1]
                            - self.telemetry[agent].position_ned[1]
                        )
                        ** 2
                    )
                ) > 0.3:
                    return False
        return True

    def check_swarm_altitudes(self, required_altitudes):
        # takes required altitudes and checks positions of swarm
        for agent in self.telemetry.keys():
            if abs(self.telemetry[agent].geodetic[2] - required_altitudes[agent]) > 0.5:
                return False
        return True


class TelemetryUpdater:
    def __init__(
        self,
        id,
        drone,
        client,
        swarm_telem,
        event_loop,
        geodetic_ref,
        ulog_callback,
    ):
        self.id = id
        self.drone = drone
        self.client = client

        asyncio.ensure_future(
            self.get_position(swarm_telem, geodetic_ref),
            loop=event_loop,
        )
        asyncio.ensure_future(self.get_heading(swarm_telem), loop=event_loop)
        asyncio.ensure_future(self.get_velocity(swarm_telem), loop=event_loop)
        asyncio.ensure_future(
            self.get_arm_status(swarm_telem, ulog_callback), loop=event_loop
        )
        asyncio.ensure_future(self.get_battery_level(), loop=event_loop)
        asyncio.ensure_future(self.get_flight_mode(swarm_telem), loop=event_loop)

    async def get_position(self, swarm_telem, geodetic_ref):
        # set the rate of telemetry updates to 10Hz
        await self.drone.telemetry.set_rate_position(10)
        async for position in self.drone.telemetry.position():

            swarm_telem[self.id].geodetic = (
                position.latitude_deg,
                position.longitude_deg,
                position.absolute_altitude_m,
            )

            swarm_telem[self.id].position_ned = pm.geodetic2ned(
                position.latitude_deg,
                position.longitude_deg,
                position.absolute_altitude_m,
                geodetic_ref[0],
                geodetic_ref[1],
                geodetic_ref[2],
            )

            self.client.publish(
                self.id + "/telemetry/geodetic",
                str(swarm_telem[self.id].geodetic).strip("()"),
            )

            self.client.publish(
                self.id + "/telemetry/position_ned",
                str(swarm_telem[self.id].position_ned).strip("()"),
            )

            # if (
            #     -swarm_telem.position_ned[2] <= bottom_alt_limit
            #     or -swarm_telem.position_ned[2] >= top_alt_limit
            # ):
            #     self.client.publish("emergency_stop", "reached an altitude limit")

    async def get_heading(self, swarm_telem):
        # set the rate of telemetry updates to 10Hz
        # await drone.telemetry.set_rate_heading(10)
        async for heading in self.drone.telemetry.heading():

            swarm_telem[self.id].heading = heading

            self.client.publish(
                self.id + "/telemetry/heading",
                str(swarm_telem[self.id].heading.heading_deg).strip("()"),
            )

    async def get_velocity(self, swarm_telem):
        # set the rate of telemetry updates to 10Hz
        await self.drone.telemetry.set_rate_position_velocity_ned(10)
        async for position_velocity_ned in self.drone.telemetry.position_velocity_ned():
            # changed from list to tuple so formatting for all messages is the same
            swarm_telem[self.id].velocity_ned = (
                position_velocity_ned.velocity.north_m_s,
                position_velocity_ned.velocity.east_m_s,
                position_velocity_ned.velocity.down_m_s,
            )
            self.client.publish(
                self.id + "/telemetry/velocity_ned",
                str(swarm_telem[self.id].velocity_ned).strip("()"),
            )

    async def get_arm_status(self, swarm_telem, ulog_callback):
        async for is_armed in self.drone.telemetry.armed():
            if is_armed != swarm_telem[self.id].arm_status:
                swarm_telem[self.id].arm_status = is_armed
                self.client.publish(
                    self.id + "/telemetry/arm_status",
                    str(swarm_telem[self.id].arm_status),
                )
                # if swarm_telem[self.id].arm_status == False:
                #     await ulog_callback()

    async def get_battery_level(self):
        await self.drone.telemetry.set_rate_battery(0.1)
        async for battery_level in self.drone.telemetry.battery():
            self.client.publish(
                self.id + "/battery_level",
                str(round(battery_level.remaining_percent * 100)),
            )

    async def get_flight_mode(self, swarm_telem):
        async for flight_mode in self.drone.telemetry.flight_mode():
            if str(flight_mode) != swarm_telem[self.id].flight_mode:
                swarm_telem[self.id].flight_mode = str(flight_mode)
                print(swarm_telem[self.id].flight_mode)
                self.client.publish(self.id + "/flight_mode", str(flight_mode), qos=2)
