async def return_to_home(
    drone, home_lat, home_long, rtl_altitude, rtl_start_lat, rtl_start_long
):
    await drone.action.goto_location(rtl_start_lat, rtl_start_long, rtl_altitude, 0)
    while abs(current_lat - rtl_start_lat) > 0.0001 or abs(current_long - rtl_start_long) < 0.0001 or abs(current_alt - rtl_altitude) > 0.05:
    
    # Now, the drone is at the right altitude to return to home
    await drone.action.goto_location(home_lat, home_long, rtl_altitude, 0)

    while abs(current_lat - home_lat) > 0.0001 or abs(current_long - home_long) > 0.0001 or abs(current_alt - rtl_altitude) > 0.05:  
    
    # Now, the drone is exactly above the launch point
    await drone.action.land()
return
