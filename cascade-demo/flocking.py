from time import time
import numpy as np
import random
import math
from mavsdk.offboard import VelocityNedYaw


def check_velocity(
    desired_vel, my_pos_vel, max_speed, yaw, time_step, max_accelleration
):
    current_vel = np.array(my_pos_vel.velocity_ned)

    # impose velocity limit
    if np.linalg.norm(desired_vel) > max_speed:
        desired_vel = desired_vel / np.linalg.norm(desired_vel) * max_speed

    # impose accelleration limit
    limit_accelleration(desired_vel, current_vel, time_step, max_accelleration)

    # yaw = 0.0
    output_vel = desired_vel
    return VelocityNedYaw(output_vel[0], output_vel[1], output_vel[2], yaw)


def limit_accelleration(desired_vel, current_vel, time_step, max_accel):
    delta_v = np.linalg.norm(desired_vel - current_vel)

    accelleration = delta_v / time_step

    # impose accelleration limit
    if accelleration > max_accel:
        desired_vel = (
            desired_vel
            / np.linalg.norm(desired_vel)
            * (max_accel * time_step + np.linalg.norm(current_vel))
        )

    return desired_vel


def simple_flocking(drone_id, swarm_pos_vel, my_pos_vel, time_step, max_accel):
    com = np.array([0, 0, 0])
    k_cohesion = 1
    for key in swarm_pos_vel:
        p = np.array(swarm_pos_vel[key].position_ned)
        com = com + p
    com = com / len(swarm_pos_vel)
    v_cohesion = (
        k_cohesion
        * (com - np.array(my_pos_vel.position_ned))
        / np.linalg.norm(com - np.array(my_pos_vel.position_ned))
    )

    r_0 = 10
    v_separation = np.array([0, 0, 0])
    for key in swarm_pos_vel:
        if key == drone_id:
            continue
        p = np.array(swarm_pos_vel[key].position_ned)
        x = np.array(my_pos_vel.position_ned) - p
        d = np.linalg.norm(x)
        v_separation = v_separation + ((x / d) * (r_0 - d) / r_0)

    output_vel = v_cohesion + v_separation

    output_vel = limit_accelleration(
        output_vel, np.array(my_pos_vel.velocity_ned), time_step, max_accel
    )

    return output_vel


def get_desired_yaw(north, east):
    # rho = np.sqrt(x**2 + y**2)
    yaw = np.arctan2(east, north)
    yaw = yaw * 180 / np.pi
    return yaw


def migration_test(migrated):
    if migrated == True:
        north = 0
    else:
        north = 200

    east = random.randint(0, 50)
    down = -20

    return [north, east, down]


def velocity_to_point(my_pos_vel, desired_pos):
    k = 5
    desired_pos = np.array(desired_pos)
    current_pos = np.array(my_pos_vel.position_ned)
    unit_vector = (desired_pos - current_pos) / np.linalg.norm(
        desired_pos - current_pos
    )

    output_vel = unit_vector * k
    yaw = get_desired_yaw(output_vel[0], output_vel[1])

    return output_vel, yaw
