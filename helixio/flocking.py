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
        if d<= r_0:
            v_separation = v_separation + ((x / d) * (r_0 - d) / r_0)

    output_vel = v_cohesion + v_separation

    output_vel = limit_accelleration(
        output_vel, np.array(my_pos_vel.velocity_ned), time_step, max_accel
    )

    return output_vel

def single_torus_swarming(drone_id, swarm_pos_vel, my_pos_vel, time_step, max_accel):
    laneRadius=1
    Torus_points_num=2000
    Torus_circle_rad=10
    a_ellipse=8
    b_ellipse=15
    targetPoint=np.array([0,0,0])
    targetDirection=np.array([0,0,0])
    nearestRange=math.inf
    nearestIndex=0
    redTorusPoints=[]
    redTorusDirections=[]
    
    #Creating Torus points -----------------------------------------------
    for i in range(Torus_points_num):
        redTorusPoints.append(np.array([a_ellipse*math.sin(i*6.28/Torus_points_num), b_ellipse*math.cos(i*6.28/Torus_points_num),5]))
    
    #Creating Torus directions -----------------------------------------
    for i in range(Torus_points_num):
    if i==Torus_points_num-1:
        redTorusDirections.append(redTorusPoints[0]-redTorusPoints[i])
    else:
        redTorusDirections.append(redTorusPoints[i+1]-redTorusPoints[i])
    
    #Finding nearest Torus point ---------------------------------
    for i in range(Torus_points_num):
        range_to_point= np.linalg.norm(np.array(my_pos_vel.position_ned)-np.array(redTorusPoints[i]))
	if range_to_point<=nearestRange:
	    nearestRange=range_to_point
            nearestIndex=i
	    targetPoint=redTorusPoints[nearestIndex]
            targetDirection =redTorusDirections[nearestIndex]
    
    #Calculating migration velocity (normalized)---------------------
    k_migration=1
    limit_v_migration=1
    v_migration = targetDirection/np.linalg.norm(targetDirection)
    if np.linalg.norm(v_migration)> limit_v_migration:
        v_migration=v_migration*limit_v_migration/np.linalg.norm(v_migration)
    
    #Calculating lane Cohesion Velocity ---------------
    k_laneCohesion=3
    limit_v_laneCohesion=1
    laneCohesionPositionError=targetPoint-np.array(my_pos_vel.position_ned)
    laneCohesionPositionError_magnitude=np.linalg.norm(laneCohesionPositionError)
		
    v_laneCohesion=(laneCohesionPositionError_magnitude-laneRadius)*laneCohesionPositionError/np.linalg.norm(laneCohesionPositionError)
        
    if np.linalg.norm(v_laneCohesion)> limit_v_laneCohesion:
	v_laneCohesion=v_laneCohesion*limit_v_laneCohesion/np.linalg.norm(v_laneCohesion)
        
    #Calculating v_rotation (normalized)---------------------
    k_rotation=2
    limit_v_rotation=1
    if (laneCohesionPositionError_magnitude<laneRadius):
        v_rotation_magnitude=laneCohesionPositionError_magnitude/laneRadius
    else:
	v_rotation_magnitude=laneRadius/laneCohesionPositionError_magnitude
		
    v_rotation=v_rotation_magnitude*np.cross(v_laneCohesion, targetDirection)/np.linalg.norm(np.cross(v_laneCohesion, targetDirection))
    if np.linalg.norm(v_rotation)> limit_v_rotation:
        v_rotation=v_rotation*limit_v_rotation/np.linalg.norm(v_rotation)
		
    #Calculating v_separation (normalized) -----------------------------
    k_separation=2
    limit_v_separation=1
    r_0 = 2
    v_separation = np.array([0, 0, 0])
    for key in swarm_pos_vel:
	if key==drone_id:
	    continue
	p = np.array(swarm_pos_vel[key].position_ned)
	x = np.array(my_pos_vel.position_ned) - p
	d = np.linalg.norm(x)
	if d<=r_0:
	    v_separation = v_separation + ((x / d) * (r_0 - d/ r_0))
			
        if np.linalg.norm(v_separation)>limit_v_separation:
	    v_separation=v_separation*limit_v_separation/np.linalg.norm(v_separation)	
    
    #Calculating net velocity ---------
    output_vel =k_laneCohesion*v_laneCohesion + k_migration*v_migration+ k_rotation* v_rotation +k_separation*v_separation
    
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
    k = 40
    desired_pos = np.array(desired_pos)
    current_pos = np.array(my_pos_vel.position_ned)
    unit_vector = (desired_pos - current_pos) / np.linalg.norm(
        desired_pos - current_pos
    )

    output_vel = unit_vector * k
    yaw = get_desired_yaw(output_vel[0], output_vel[1])

    return output_vel, yaw
