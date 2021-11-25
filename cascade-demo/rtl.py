def return_to_home(
    drone, home_lat, home_long, rtl_altitude, rtl_start_lat, rtl_start_lon
):
    await drone.action.goto_location(rtl_start_lat, rtl_start_lon, rtl_altitude, 0)
    if the dorne is at the right return altitude:
        await drone.action.goto_location(home_lat, home_long, rtl_altitude, 0)
        if the drone is above the launch point:
            await drone.action.land()    
    return
