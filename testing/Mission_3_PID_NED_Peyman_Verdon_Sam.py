import time
from math import (cos,sin,sqrt)
import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityNedYaw)
import pymap3d as pm

# defining calss of drone telemetry------------------------------------------------------------------------------------
class drone_telemetry:
    qx = 0          # position of the quadrotor in x direction with respect to initial point
    qy = 0          # position of the quadrotor in y direction with respect to initial point
    qz = 0          # position of the quadrotor in z direction with respect to initial point

# run function (main code) --------------------------------------------------------------------------------------------
async def run():
    # Init the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered!")
            break
    
    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    #Start background task
    #below are reference GPS coordinates used as the origin of the NED coordinate system
    ref_lat = 53.48
    ref_long = -2.24
    ref_alt = 0
    asyncio.ensure_future(get_position(drone, ref_lat, ref_long, ref_alt))
    # End of Init the drone  ------------------------------------------------------------------------------------------  
    kp=2
    ki=1
    dt=0.1                          # duration of each loop
    ex=0                            # error in x direction
    Integ_x=0                       # Integral of error in x direction
    ey=0                            # error in y direction
    Integ_y=0                       # Integral of error in y direction
    ez=0                            # error in z direction
    Integ_z=0                       # Integral of error in z direction
    maxSpeed = 5                    # maximum magnitude of the speed vector
    # take off the quadrotor ------------------------------------------------------------------------------------------
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, -2.0, 0.0))
    await asyncio.sleep(5)
    
    Mission_start_time=time.time()  # start time of the mission
    # Endless loop (Mission)--------------------------------------------------------------------------------------------
    while 1:
        loop_start_time=time.time()
        
        #Printing the poition of the quadrotor in NED coordinates ------------------------------------------------------
        print("--qx=",drone_telemetry.qx) # drone_telemetry.qx is the most updated position of the quadrotor in x direction 
        print("--qy=",drone_telemetry.qy) # drone_telemetry.qy is the most updated position of the quadrotor in y direction
        print("--qz=",drone_telemetry.qz) # drone_telemetry.qz is the most updated position of the quadrotor in z direction
        
        #Creating desired path ----------------------------------------------------------------------------------------
        t=time.time()-Mission_start_time   # current time of the mission
        x_desired=5*sin(0.5*t)             # position of the desired position in x direction
        Vx_desired=0.5*5*cos(0.5*t)        # velocity of the desired position in x direction
        y_desired=5*cos(0.5*t)             # position of the desired position in y direction
        Vy_desired=0.5*-5*sin(0.5*t)       # velocity of the desired position in y direction
        z_desired=-20                      # position of the desired position in z direction
        Vz_desired=0                       # velocity of the desired position in z direction      
        
        #PID controller ------------------------------------------------------------------------------------------------
        
        # x direction --------------------------------------------------------------------------------------------------
        previous_ex=ex
        ex=x_desired-drone_telemetry.qx
        Integ_x=((ex+previous_ex)/2)*dt +Integ_x # Trapezoidal Integration
        if Integ_x >= 1:  # limiting the accumulator of integral term
            Integ_x=1
        if Integ_x <= -1:
            Integ_x=-1
            
        target_vx=kp*(ex)+ki*Integ_x+Vx_desired
        
        print("--ex=",ex)
        print("--Integ_x=",Integ_x)
        
        # y direction ---------------------------------------------------------------------------------------------------
        previous_ey=ey
        ey=y_desired-drone_telemetry.qy
        Integ_y=((ey+previous_ey)/2)*dt +Integ_y # Trapezoidal Integration
        
        if Integ_y >= 1:  # limiting the accumulator of integral term
            Integ_y=1
        if Integ_y <= -1:
            Integ_y=-1
            
        target_vy=kp*(ey)+ki*Integ_y+Vy_desired
        
        print("--ey=",ey)
        print("--Integ_y=",Integ_y)
        
        #z direction -----------------------------------------------------------------------------------------------------
        previous_ez=ez
        ez=z_desired-drone_telemetry.qz
        Integ_z=((ez+previous_ez)/2)*dt +Integ_z # Trapezoidal Integration
        
        if Integ_z >= 1:  # limiting the accumulator of integral term
            Integ_z=1
        if Integ_z <= -1:
            Integ_z=-1
            
        target_vz=kp*(ez)+ki*Integ_z+Vz_desired
        
        print("--ez=",ez)
        print("--Integ_z=",Integ_z)
        
        #limiting and normalizing the speed of the drone -------------------------------------------------------------------
        v = sqrt(target_vx**2 + target_vy**2 + target_vz**2)
        normFactor = 1
        if v > maxSpeed:
            normFactor = maxSpeed / v 
            target_vx *= normFactor
            target_vy *= normFactor
            target_vz *= normFactor

        print("--target_vx=",target_vx)
        print("--target_vy=",target_vy)
        print("--target_vz=",target_vz)
        
        #Sending the target velocities to the quadrotor ---------------------------------------------------------------------
        await drone.offboard.set_velocity_ned(VelocityNedYaw(target_vx, target_vy, target_vz, 0.0))
        
        #Checking frequency of the loop
        await asyncio.sleep(0.1-(time.time()-loop_start_time)) # to make while 1 work at 10 Hz
        print("loop duration=",(time.time()-loop_start_time))
    # End of Endless loop ---------------------------------------------------------------------------------------------------
# End of run function (main code) -------------------------------------------------------------------------------------------

#runs in background and upates state class with latest telemetry -----------------------------------------------------------
async def get_position(drone,ref_lat,ref_long,ref_alt):
    async for position in drone.telemetry.position():
            drone_telemetry.qx, drone_telemetry.qy, drone_telemetry.qz = pm.geodetic2ned(position.latitude_deg, position.longitude_deg, position.absolute_altitude_m, ref_lat, ref_long, ref_alt)

if __name__ == "__main__":
    # Start the main function
    asyncio.ensure_future(run())

    # Runs the event loop until the program is canceled with e.g. CTRL-C
    asyncio.get_event_loop().run_forever()

