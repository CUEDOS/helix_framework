def return_to_home(
    drone, home_lat, home_long, rtl_altitude, rtl_start_lat, rtl_start_lon
):
    await drone.action.goto_location(rtl_start_lat, rtl_start_lon, rtl_altitude, 0)
    await drone.action.goto_location(home_lat, lome_long, rtl_altitude, 0)
    return
