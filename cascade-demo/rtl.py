def return_to_home(
    drone, home_lat, home_long, rtl_altitude, rtl_start_lat, rtl_start_lon
):
    await drone.action.goto_location(rtl_start_lat, rtl_start_lon, rtl_altitude, 0)
    if abs(current_lat-home_lat)<0.0001 and abs(current_lon-home_long)<0.0001 and abs(current_alt-rtl_altitude)<0.05:
        await drone.action.goto_location(home_lat, home_long, rtl_altitude, 0)
        if the drone is above the launch point:
            await drone.action.land()    
    return
