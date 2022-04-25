import csv
from pyulog.ulog2csv import convert_ulog2csv  # convert_ulog2csv is a function in ulog2csv file in pyulog package
from pymap3d.ned import geodetic2ned
import plotly.express as px
import plotly.graph_objects as go
from scipy import interpolate
import glob, os
import math

def vis (**Input):  # input keyword arguments: ref_lat, ref_long, ref_alt
    """
    Draws the path of drones based on their ulog file and animates them in Cartesian coordinate
    Arguments:
        directory: directory of the folder containing all the ulog files of the drones we want to visualize
        ref_lat: latitude of geodetic origin reference point 
        ref_long: longitude of geodetic origin reference point 
        ref_alt: altitude of geodetic origin reference point
        drone_size: size of drones in visualization
        ticks_num: number of ticks for each cartesian axis
        
        Note: if a user does not provide one argumetns of ref_lat, ref_long and ref_alt, the function considers 
            a point with the least latitude, longitude and altitude as the re
    Returns:
        An animated figure of all drones with interpolated positions
    """
    REF_lat=None
    REF_long=None
    REF_alt=None
    Drone_size=10
    Ticks_num=10
    for key, value in Input.items():
        if key=="ref_lat":
            REF_lat=value
        elif key=="ref_long":
            REF_long=value
        elif key=="ref_alt":
            REF_alt=value
        elif key=="directory":
            directory=value
        elif key=="drone_size":
            Drone_size=value
        elif key=="ticks_num":
            Ticks_num=value

    x=[]
    x_max=-1*math.inf  #for figure range
    x_min=math.inf     #for figure range  
    fx=[]              #x interpolation 
    X_total=[]         #total interpolated x

    y=[]
    y_max=-1*math.inf  #for figure range
    y_min=math.inf     #for figure range
    fy=[]              #y interpolation
    Y_total=[]         #total interpolated y

    z=[]
    z_max=-1*math.inf  #for figure range
    z_min=math.inf     #for figure range
    fz=[]              #z interpolation
    Z_total=[]         #total interpolated z
    
    Time_total=[]      #Time span of interpolation used for figure
    interp_time=[]     #Time span of interpolation

    i=0
    interp_length=0
    latitude=[]
    longitude=[]
    altitude=[]
    Color_total=[]
    Time=[] # sampling time of each drone
    file_names=[]


    min_lat=math.inf
    max_lat=-1*math.inf
    min_long=math.inf
    max_long=-1*math.inf
    max_alt=math.inf
    min_alt=math.inf
    max_start_time=-1*math.inf
    min_finish_time=math.inf
    
    os.chdir(directory)
    for file in glob.glob("*.ulg"):
        convert_ulog2csv(directory+"/"+file, "vehicle_gps_position", directory, delimiter=";")
        file_names.append(file.replace(".ulg",""))
        latitude.append([])  # new line for latitudes
        longitude.append([]) # new line for longitudes
        altitude.append([])  # new line for altitudes
        Time.append([])      # new line for sampling time of each drone
    
        with open (directory+"/"+file.replace(".ulg","")+"_vehicle_gps_position_0.csv", newline="") as csv_file:
            Object_of_dictionaries=csv.DictReader(csv_file, delimiter=";")
            for row_dict in Object_of_dictionaries: # we should create the lists when the file is still open
                
                Time[i].append(float(row_dict["time_utc_usec"])/1000000) # in seconds, Unix

                latitude[i].append(float(row_dict ["lat"])/10000000)
                min_lat=min(min_lat, float(row_dict ["lat"])/10000000)
                max_lat=max(max_lat, float(row_dict ["lat"])/10000000)

                longitude[i].append(float(row_dict["lon"])/10000000)
                min_long=min(min_long, float(row_dict ["lon"])/10000000)
                max_long=max(max_long, float(row_dict ["lon"])/1000000)

                altitude[i].append(float(row_dict ["alt"])/1000) # in meters, relative to sea level
                min_alt=min(min_alt, float(row_dict ["alt"])/1000)
                max_alt=max(max_alt, float(row_dict ["alt"])/1000)
                
        max_start_time=max(max_start_time, Time[i][0])
        latest_drone=i
        min_finish_time=min(min_finish_time, Time[i][len(Time[i])-1])
        i+=1 #number of files

    if (REF_lat==None or REF_long==None or REF_alt==None):
        REF_lat=min_lat
        REF_long=min_long
        REF_alt=min_alt
        print("Auto generated geodetic origin is at: latitude=",REF_lat, "longitude=", REF_long, "altitude=", REF_lat)
    else:
        print("Entered geodetic origin is at: latitude=",REF_lat, "longitude=", REF_long, "altitude=", REF_lat)
    # Converting geodetics to Cartesian 
  
    for j in range(i):  # j is the number of a drone
        x.append([])  #new line for x coordinates
        y.append([])  #new line for y coordinates
        z.append([])  #new line for z coordinates 
        for  k in range(len(altitude[j])): # k is the number of samples of drone j
            n,e,d =geodetic2ned(latitude[j][k], longitude[j][k], altitude[j][k], REF_lat, REF_long, REF_alt, ell=None, deg=True)
            x[j].append(n)
            x_max=max(x_max, n)
            x_min=min(x_min, n)

            y[j].append(e)
            y_max=max(y_max, e)
            y_min=min(y_min, e)

            z[j].append(-1*d)
            z_max=max(z_max, -1*d)
            z_min=min(z_min, -1*d)
        
        fx.append(interpolate.interp1d(Time[j], x[j]))
        fy.append(interpolate.interp1d(Time[j], y[j]))
        fz.append(interpolate.interp1d(Time[j], z[j]))
    
    # Creating interpolated positions
    # Adding positions of the latest drone to the interpolted lists
    
    for k in range(len(x[latest_drone])): # k is the number of samples of the latest drone
        if Time[latest_drone][k]<= min_finish_time:
            X_total.append(x[latest_drone][k])
            Y_total.append(y[latest_drone][k])
            Z_total.append(z[latest_drone][k])
            Time_total.append(Time[latest_drone][k]-max_start_time)
            Color_total.append(file_names[latest_drone])
            interp_time.append(Time[latest_drone][k])
            interp_length+=1
    # Calculating and Adding positions of the latest drone to the interpolted lists
    for j in range(i): # j is the number of a drone
        if j==latest_drone:
            continue
        for t in interp_time:
            X_total.append(fx[j](t))
            Y_total.append(fy[j](t))
            Z_total.append(fz[j](t))
            Time_total.append(t-max_start_time)
            Color_total.append(file_names[j])

    x_right_margin=x_max+(x_max-x_min)*0.05
    x_left_margin=x_min-(x_max-x_min)*0.05
    x_range=x_right_margin-x_left_margin
    x_parts=Ticks_num

    y_up_margin=y_max+(y_max-y_min)*0.05
    y_down_margin=y_min-(y_max-y_min)*0.05
    y_range=y_up_margin-y_down_margin

    z_up_margin=z_max+(z_max-z_min)*0.05
    z_down_margin=z_min
    z_range=z_up_margin-z_down_margin
    
    # Making figure a cube with real scale
    max_range=max(x_range, y_range, z_range)
    x_right_margin=(x_right_margin+x_left_margin)/2 + max_range/2
    x_left_margin=(x_right_margin+x_left_margin)/2 - max_range/2
    x_range=max_range

    y_up_margin=(y_up_margin+y_down_margin)/2 + max_range/2
    y_down_margin=(y_up_margin+y_down_margin)/2 - max_range/2
    y_range=max_range

    z_up_margin=z_down_margin + max_range
    z_range=max_range

    
    SIZE=int(Drone_size)
    size=[SIZE for k in range(len(X_total))]
    fig= px.scatter_3d(x=X_total, range_x=[x_right_margin,x_left_margin], y=Y_total, range_y=[y_up_margin,y_down_margin], z=Z_total, range_z=[z_down_margin,z_up_margin], animation_frame=Time_total, opacity=1, size=size, color=Color_total, size_max=max(size))
    
    #Adding lines to the figure
    for j in range(i):
        fig.add_trace(            #should be an object of go
            go.Scatter3d(
                x=X_total[j*interp_length: ((j+1)*interp_length)-1], 
                y=Y_total[j*interp_length: ((j+1)*interp_length)-1],
                z=Z_total[j*interp_length: ((j+1)*interp_length)-1], 
                mode='lines', 
        )
    )
    fig.layout.updatemenus[0].buttons[0].args[1]['frame']['duration'] = interp_time[1]-interp_time[0]
    fig.layout.updatemenus[0].buttons[0].args[1]['transition']['duration'] = 0.1
    fig.update_layout(
        showlegend=True,
        legend=dict(itemsizing='constant',font=dict(family="Times New Roman",size=20), bgcolor="LightSteelBlue", bordercolor="Black", borderwidth=2),
        scene_aspectmode='manual',
        scene_aspectratio=dict(x=1, y=y_range/x_range, z=z_range/x_range), 
        scene = dict(xaxis = dict(nticks=x_parts,range=[x_right_margin,x_left_margin]), yaxis = dict(nticks=math.ceil((y_range/x_range)*x_parts), range=[y_up_margin,y_down_margin]),zaxis = dict(nticks=math.ceil((z_range/x_range)*x_parts),range=[z_down_margin,z_up_margin]))
        )
    fig.show()
    
   
# vis(directory="logs",drone_size=10, ticks_num=10)

