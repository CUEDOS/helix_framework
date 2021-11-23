import time
from math import (cos,sin,sqrt)
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)
import pymap3d as pm
import numpy as np

# defining calss of drone telemetry------------------------------------------------------------------------------------
class drone1_telemetry:
    qx1 = 0.00          # position of the quadrotor in x direction with respect to initial point
    qy1 = 0.00          # position of the quadrotor in y direction with respect to initial point
    qz1 = 0.00          # position of the quadrotor in z direction with respect to initial point

class drone2_telemetry:
    qx2 = 0.00          # position of the quadrotor in x direction with respect to initial point
    qy2 = 0.00          # position of the quadrotor in y direction with respect to initial point
    qz2 = 0.00          # position of the quadrotor in z direction with respect to initial point

# run function (main code) --------------------------------------------------------------------------------------------
async def run():
    # Init the drone1 -------------------------------------------------------------------------------------------------
    drone1 = System(mavsdk_server_address="localhost", port=50041)
    await drone1.connect()
    print("Waiting for drone1 to connect...")
    async for state in drone1.core.connection_state():
        if state.is_connected:
            print(f"Drone1 discovered!")
            break
    
    print("-- Arming drone1")
    await drone1.action.arm()

    print("-- Setting initial setpoint")
    await drone1.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone1.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone1.action.disarm()
        return
    # Init the drone2 --------------------------------------------------------------------------------------------------
    drone2 = System(mavsdk_server_address="localhost", port=50042)
    await drone2.connect()
    print("Waiting for drone2 to connect...")
    async for state in drone2.core.connection_state():
        if state.is_connected:
            print(f"Drone2 discovered!")
            break
    
    print("-- Arming drone2")
    await drone2.action.arm()

    print("-- Setting initial setpoint")
    await drone2.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone2.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone2.action.disarm()
        return
    #Start background task
    #below are reference GPS coordinates used as the origin of the NED coordinate system
    ref_lat = 53.4800000000000000
    ref_long = -2.2400000000000000
    ref_alt = 0.00
    asyncio.ensure_future(get_position_1(drone1, ref_lat, ref_long, ref_alt))
    asyncio.ensure_future(get_position_2(drone2, ref_lat, ref_long, ref_alt))
    # End of Init the drone  ------------------------------------------------------------------------------------------  
    kp=2.00
    ki=1.00
    dt=0.10                                              # duration of each loop
    ex=np.array([0.00,0.00], dtype='f')                  # error of the drones in x direction
    Integ_x=np.array([0.00,0.0], dtype='f')             # Integral of error of the drones in x direction
    ey=np.array([0.00,0.00], dtype='f')                  # error of the drones in y direction
    Integ_y=np.array([0.00,0.00], dtype='f')             # Integral of error of the drones in y direction
    ez=np.array([0.00,0.00], dtype='f')                  # error of the drones in z direction
    Integ_z=np.array([0.00,0.00], dtype='f')             # Integral of error of the drones in z direction
    normFactor=np.array([1.00,1.00], dtype='f')          # speed normalizing factor
    maxSpeed = 5.00                                      # maximum magnitude of the speed vector
    
    # take off the drones ------------------------------------------------------------------------------------------
    await drone1.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -2.0, 0.0))
    await drone2.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(5)
    
    Mission_start_time=time.time()  # start time of the mission
    # Endless loop (Mission)--------------------------------------------------------------------------------------------
    while 1:
        loop_start_time=time.time()
        
        #Printing the poition of the drones in NED coordinates ------------------------------------------------------
        print("--qx1=",drone1_telemetry.qx1) # drone1_telemetry.qx1 is the most updated position of drone1 in x direction 
        print("--qy1=",drone1_telemetry.qy1) # drone1_telemetry.qy1 is the most updated position of drone1 in y direction
        print("--qz1=",drone1_telemetry.qz1) # drone1_telemetry.qz1 is the most updated position of drone1 in z direction
        print("--qx2=",drone2_telemetry.qx2) # drone2_telemetry.qx2 is the most updated position of drone2 in x direction 
        print("--qy2=",drone2_telemetry.qy2) # drone2_telemetry.qy2 is the most updated position of drone2 in y direction
        print("--qz2=",drone2_telemetry.qz2) # drone2_telemetry.qz2 is the most updated position of drone2 in z direction
        
        #Creating desired path ----------------------------------------------------------------------------------------
        t=time.time()-Mission_start_time                                           # current time of the mission
        x_desired=np.array([-5+5*sin(0.5*t), 5+5*sin(0.5*t)], dtype='f')           # desired positions in x direction
        Vx_desired=np.array([0.5*5*cos(0.5*t), 0.5*5*cos(0.5*t)], dtype='f')       # velocities of the desired positions in x direction
        y_desired=np.array([5*cos(0.5*t), 5*cos(0.5*t)], dtype='f')                # desired positions in y direction
        Vy_desired=np.array([0.5*-5*sin(0.5*t), 0.5*-5*sin(0.5*t)], dtype='f')     # velocities of the desired positions in y direction
        z_desired=np.array([-20.00, -20.00], dtype='f')                            # desired positions in z direction
        Vz_desired=np.array([0.00, 0.00], dtype='f')                               # velocities of the desired positions in z direction      
        
        #PID controller ------------------------------------------------------------------------------------------------
        
        # x direction --------------------------------------------------------------------------------------------------
        previous_ex=ex
        ex=x_desired-np.array([drone1_telemetry.qx1, drone2_telemetry.qx2])
        Integ_x=((ex+previous_ex)/2.00)*dt +Integ_x # Trapezoidal Integration
        
        if Integ_x[0] >= 1:  # limiting the accumulator of integral term of drone1 in x direction
            Integ_x[0]=1
        if Integ_x[0] <= -1:
            Integ_x[0]=-1
            
        if Integ_x[1] >= 1:  # limiting the accumulator of integral term of drone2 in x direction
            Integ_x[1]=1
        if Integ_x[1] <= -1:
            Integ_x[1]=-1
            
        target_vx=kp*(ex)+ki*Integ_x+Vx_desired
        
        print("--ex=",ex)
        print("--Integ_x=",Integ_x)
        
        # y direction ---------------------------------------------------------------------------------------------------
        previous_ey=ey
        ey=y_desired-np.array([drone1_telemetry.qy1, drone2_telemetry.qy2])
        Integ_y=((ey+previous_ey)/2.00)*dt +Integ_y # Trapezoidal Integration
        
        if Integ_y[0] >= 1:  # limiting the accumulator of integral term of drone1 in y direction
            Integ_y[0]=1
        if Integ_y[0] <= -1:
            Integ_y[0]=-1
            
        if Integ_y[1] >= 1:  # limiting the accumulator of integral term of drone2 in y direction
            Integ_y[1]=1
        if Integ_y[1] <= -1:
            Integ_y[1]=-1
            
        target_vy=kp*(ey)+ki*Integ_y+Vy_desired
        
        print("--ey=",ey)
        print("--Integ_y=",Integ_y)
        
        #z direction -----------------------------------------------------------------------------------------------------
        previous_ez=ez
        ez=z_desired-np.array([drone1_telemetry.qz1, drone2_telemetry.qz2]) 
        Integ_z=((ez+previous_ez)/2.00)*dt +Integ_z # Trapezoidal Integration
        
        if Integ_z[0] >= 1:  # limiting the accumulator of integral term of drone1 in z direction
            Integ_z[0]=1
        if Integ_z[0] <= -1:
            Integ_z[0]=-1
            
        if Integ_z[1] >= 1:  # limiting the accumulator of integral term of drone2 in z direction
            Integ_z[1]=1
        if Integ_z[1] <= -1:
            Integ_z[1]=-1
            
        target_vz=kp*(ez)+ki*Integ_z+Vz_desired
        
        print("--ez=",ez)
        print("--Integ_z=",Integ_z)
        
        #limiting and normalizing the speed of the drones -------------------------------------------------------------------
        v = np.array([sqrt(target_vx[0]**2 + target_vy[0]**2+ target_vz[0]**2), sqrt(target_vx[1]**2 + target_vy[1]**2 + target_vz[1]**2)], dtype='f')
        if v[0] > maxSpeed:
            normFactor[0] = maxSpeed / v[0]
            target_vx[0] = target_vx[0] * normFactor[0]
            target_vy[0] = target_vy[0] * normFactor[0]
            target_vz[0] = target_vz[0] * normFactor[0]
        
        if v[1] > maxSpeed:
            normFactor[1] = maxSpeed / v[1]
            target_vx[1] = target_vx[1] * normFactor[1]
            target_vy[1] = target_vy[1] * normFactor[1]
            target_vz[1] = target_vz[1] * normFactor[1]

        print("--target_vx1=",target_vx[0])
        print("--target_vy1=",target_vy[0])
        print("--target_vz1=",target_vz[0])
        print("--target_vx2=",target_vx[1])
        print("--target_vy2=",target_vy[1])
        print("--target_vz2=",target_vz[1])
        
        #Sending the target velocities to the quadrotor ---------------------------------------------------------------------
        await drone1.offboard.set_velocity_ned(VelocityNedYaw(target_vx[0], target_vy[0], target_vz[0], 0.0))
        await drone2.offboard.set_velocity_ned(VelocityNedYaw(target_vx[1], target_vy[1], target_vz[1], 0.0))
        
        #Checking frequency of the loop
        await asyncio.sleep(0.10-(time.time()-loop_start_time)) # to make while 1 work at 10 Hz
        print("loop duration=",(time.time()-loop_start_time))
    # End of Endless loop ---------------------------------------------------------------------------------------------------
# End of run function (main code) -------------------------------------------------------------------------------------------

#runs in background and upates state class with latest telemetry -----------------------------------------------------------
async def get_position_1(drone1,ref_lat,ref_long,ref_alt):
    async for position in drone1.telemetry.position():
            drone1_telemetry.qx1, drone1_telemetry.qy1, drone1_telemetry.qz1 = pm.geodetic2ned\
                (position.latitude_deg, position.longitude_deg, position.absolute_altitude_m, ref_lat, ref_long, ref_alt)

async def get_position_2(drone2,ref_lat,ref_long,ref_alt):         
    async for position in drone2.telemetry.position():
            drone2_telemetry.qx2, drone2_telemetry.qy2, drone2_telemetry.qz2 = pm.geodetic2ned\
                (position.latitude_deg, position.longitude_deg, position.absolute_altitude_m, ref_lat, ref_long, ref_alt)


if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())

    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()
    
