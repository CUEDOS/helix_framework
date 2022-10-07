import csv
from pyulog.ulog2csv import convert_ulog2csv  # convert_ulog2csv is a function in ulog2csv file in pyulog package
from pymap3d.ned import geodetic2ned
import plotly.express as px
import plotly.graph_objects as go
from scipy import interpolate
import glob, os
import math

def visualize_ulg (**Input):  # input keyword arguments: ref_lat, ref_long, ref_alt
    """
    Draws the path of drones based on their ulog file and animates them in Cartesian coordinate
    Arguments:
        ulg_folder: directory of the folder containing all the ulog files of the drones we want to visualize
        ref_lat: latitude of geodetic origin reference point 
        ref_long: longitude of geodetic origin reference point 
        ref_alt: altitude of geodetic origin reference point
        drone_size: size of drones in visualization
        ticks_num: number of ticks for each cartesian axis
        dt: time step of animation in seconds
        sitl_or_real: if its value is 'real', it means ulg is for a real experiment and if the value is 'sitl' it means ulg is for a sitl simulation (the default value is 'real'), 
        frame_duration: duratin of each frame of animation (second)
        cubic_space: if True, the whole sapce will be a cube

        Note: if a user does not provide one arguments of ref_lat, ref_long and ref_alt, the function considers 
            a point with the least latitude, longitude and altitude as the reference point
    Returns:
        An animated figure of all drones with interpolated positions
    """
    ref_lat=None
    ref_long=None
    ref_alt=None
    dt=None
    folder_of_ulg=None
    output_CSV_file_dir=None
    sitl_or_real='real'
    Drone_size=10
    Ticks_num=10
    frame_duration=None
    for key, value in Input.items():
        if key=="ref_lat":
            ref_lat=value
        elif key=="ref_long":
            ref_long=value
        elif key=="ref_alt":
            ref_alt=value
        elif key=="folder_of_ulg":
            folder_of_ulg=value
        elif key=="drone_size":
            Drone_size=value
        elif key=="ticks_num":
            Ticks_num=value
        elif key=='dt':
            dt=value
        elif key=="sitl_or_real":
            sitl_or_real=value
        elif key=='output_CSV_file_dir':
            output_CSV_file_dir=value
        elif key=='frame_duration':
            frame_duration=value
        elif key=='cubic_space':
            cubic_space=value
        
    if folder_of_ulg==None:
        print('Error: A directory to folder of input ulg files should be provided')
        return 0
    if output_CSV_file_dir== None:
        print('Error: A directory to output CSV file should be provided')
        return 0
    if (sitl_or_real!='real' and sitl_or_real!='sitl'):
        print('Error: argument sitl_or_real can just be real or sitl')
        return 0
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
    z_min=0            #for figure range
    fz=[]              #z interpolation
    Z_total=[]         #total interpolated z
    
    Time_total=[]      #Time span of interpolation used for figure
    interp_time=[]     #Time span of interpolation

    i=0 # i is the number of a file (drone)
    interp_length=0
    latitude=[]
    longitude=[]
    altitude=[]
    Color_total=[]
    labels_total=[]
    Time=[] # sampling time of each drone (synchronized)
    gps_timestamp=[] # sampling time of each drone since turning on (asynchronous)
    offboard_timestamp=[]
    file_names=[]


    min_lat=math.inf
    max_lat=-1*math.inf
    min_long=math.inf
    max_long=-1*math.inf
    max_alt=math.inf
    min_alt=math.inf
    max_start_time=-1*math.inf
    min_finish_time=math.inf
    
    # opening ulg files ----------------------
    os.chdir(folder_of_ulg)
    for ulg_file in glob.glob("*.ulg"):
        convert_ulog2csv(folder_of_ulg+"/"+ulg_file, 'vehicle_gps_position', folder_of_ulg, delimiter=";") # to get positions of a drone and synchronized time
        convert_ulog2csv(folder_of_ulg+"/"+ulg_file, 'offboard_control_mode',folder_of_ulg, delimiter=";") # to check when offboard mode is active (timing is asynchronous)
        file_names.append(ulg_file.replace(".ulg","")) # file names is the name of a ulg file without .ulg
        latitude.append([])         # new line for latitudes of drone i
        longitude.append([])        # new line for longitudes of drone i
        altitude.append([])         # new line for altitudes of drone i
        Time.append([])             # new line for sampling time of drone i
        gps_timestamp.append([])       # new line for gps timestamp drone i
        offboard_timestamp.append([])  # new line for offboard mode timestamp of drone i

        # Getting data from gps csv file created from ulg file ------
        csv_file_gps=open(folder_of_ulg+"/"+ulg_file.replace(".ulg","")+"_vehicle_gps_position_0.csv", newline="")
        Object_of_dictionaries_gps=csv.DictReader(csv_file_gps, delimiter=";")
        for row_dict in Object_of_dictionaries_gps: # we should create the lists when the file is still open
            gps_timestamp[i].append(float(row_dict["timestamp"])/1000000) # in seconds, since turning on (asynchronous)
            Time[i].append(float(row_dict["time_utc_usec"])/1000000) # in seconds, Unix (synchronized)

            latitude[i].append(float(row_dict ["lat"])/10000000)
            min_lat=min(min_lat, float(row_dict ["lat"])/10000000)
            max_lat=max(max_lat, float(row_dict ["lat"])/10000000)

            longitude[i].append(float(row_dict["lon"])/10000000)
            min_long=min(min_long, float(row_dict ["lon"])/10000000)
            max_long=max(max_long, float(row_dict ["lon"])/1000000)

            altitude[i].append(float(row_dict ["alt"])/1000) # in meters, relative to sea level
            min_alt=min(min_alt, float(row_dict ["alt"])/1000)
            max_alt=max(max_alt, float(row_dict ["alt"])/1000)

        csv_file_gps.close()
        
        if max_start_time<Time[i][0]:
            max_start_time=Time[i][0]
            latest_drone=i
        
        min_finish_time=min(min_finish_time, Time[i][len(Time[i])-1])

        # Getting data from offboard csv file created from ulg file --------
        csv_file_offboard=open(folder_of_ulg+"/"+ulg_file.replace(".ulg","")+"_offboard_control_mode_0.csv", newline="")
        Object_of_dictionaries_offboard=csv.DictReader(csv_file_offboard, delimiter=";")  
        for row_dict in Object_of_dictionaries_offboard: # we should create the lists when the file is still open  
            offboard_timestamp[i].append(float(row_dict['timestamp'])/1000000)    
        csv_file_offboard.close()

        i+=1 #number of files (drones)
        # End of opening ulg files  ---------------------

    if (ref_lat==None or ref_long==None or ref_alt==None):
        ref_lat=min_lat
        ref_long=min_long
        ref_alt=min_alt
        print("Auto generated geodetic origin is at: latitude=",ref_lat, "longitude=", ref_long, "altitude=", ref_lat)
    else:
        print("Entered geodetic origin is at: latitude=",ref_lat, "longitude=", ref_long, "altitude=", ref_alt)
    
    def index_checker(input_index, length) -> int:
        if input_index >= length:
            return int(input_index % length)
        return input_index
    # Converting geodetics to Cartesian -------
    offboard_mode_status=[] # shows the offboard status of a drone
    offboard_finish=[-1*math.inf for j in range(i)]
    offboard_start=[math.inf for j in range(i)]
    for j in range(i):  # j is the number of a drone
        x.append([])  #new line for x coordinates
        y.append([])  #new line for y coordinates
        z.append([])  #new line for z coordinates 
        for  k in range(len(Time[j])): # k is the number of a sample of drone j
            n,e,d =geodetic2ned(latitude[j][k], longitude[j][k], altitude[j][k], ref_lat, ref_long, ref_alt, ell=None, deg=True)
            x[j].append(n)
            x_max=max(x_max, n)
            x_min=min(x_min, n)

            y[j].append(e)
            y_max=max(y_max, e)
            y_min=min(y_min, e)
            
            z[j].append(-1*d)
            z_max=max(z_max, -1*d)
            z_min=min(z_min, -1*d)
            
        offboard_mode_status.append([0 for n in range(len(gps_timestamp[j]))])

        for l in range(len(offboard_timestamp[j])-1): # to check when drone j was on offboard mode
            for m in range (len(gps_timestamp[j])):
                if (gps_timestamp[j][m] >= offboard_timestamp[j][l] and gps_timestamp[j][m] <= offboard_timestamp[j][l+1]): # drone j were on offboard mode at gps_timestamp [j][m]
                    offboard_mode_status[j][m]=1
                    if offboard_start[j]>=Time[j][m]:
                        offboard_start[j]=Time[j][m] # offboard start for drone j based on synchronized time
                    if offboard_finish[j]<=Time[j][m]:
                        offboard_finish[j]=Time[j][m] # offboard finish for drone j based on synchronized time

    
        fx.append(interpolate.interp1d(Time[j], x[j]))
        fy.append(interpolate.interp1d(Time[j], y[j]))
        fz.append(interpolate.interp1d(Time[j], z[j]))

    latest_offboard_start=max(offboard_start)
    earliest_offboard_finish=min(offboard_finish)

    # Creating output CSV file --------
    Output_CSV_file=open(output_CSV_file_dir, 'w')
    writer = csv.writer(Output_CSV_file)
    header=['x(m)', 'y(m)', 'z(m)', 'time(s)', 'drone id', 'offboard mode status','type of experiment']
    writer.writerow(header)
    for j in range(i):
        for k in range(len(Time[j])):
            if (Time[j][k]>=latest_offboard_start and Time[j][k]<=earliest_offboard_finish): # drone j was on range of offboard mode at Time[j][k]   
                row=[x[j][k], y[j][k], z[j][k], Time[j][k]-latest_offboard_start, file_names[j], offboard_mode_status[j][k], sitl_or_real] # x, y , z, time (s), id, status of offboard mode, type of experiment
                writer.writerow(row)
    Output_CSV_file.close()
    
    # Creating interpolated positions
    # Adding positions of the latest drone to the interpolted lists
    
    for k in range(len(x[latest_drone])): # k is the number of samples of the latest drone
        if Time[latest_drone][k]<= min_finish_time:
            interp_time.append(Time[latest_drone][k])
            interp_length+=1
    # Adding positions of the latest drone and calculating & adding interpolated positions of the other drones to the total lists
    fig_colors=['blue','red', 'lightgreen', 'orange','aqua', 'silver', 'magenta', 'darkkhaki','dodgerblue','green','black','brown']
    for j in range(i): # j is the number of a drone
        if j==latest_drone:
            for k in range(len(x[latest_drone])): # k is the number of samples of the latest drone
                if Time[latest_drone][k]<= min_finish_time:
                    X_total.append(x[latest_drone][k])
                    Y_total.append(y[latest_drone][k])
                    Z_total.append(z[latest_drone][k])
                    Time_total.append(Time[latest_drone][k]-max_start_time)
                    labels_total.append(file_names[latest_drone])
        else:
            for t in interp_time:
                X_total.append(float(fx[j](t)))
                Y_total.append(float(fy[j](t)))
                Z_total.append(float(fz[j](t)))
                Time_total.append(t-max_start_time)
                labels_total.append(file_names[j])

    x_range=x_max-x_min
    x_max=x_max+(x_range)*0.05
    x_min=x_min-(x_range)*0.05
    x_range=x_max-x_min
    
    y_range=y_max-y_min
    y_max=y_max+(y_range)*0.05
    y_min=y_min-(y_range)*0.05
    y_range=y_max-y_min
    

    z_range=z_max-z_min
    z_max=z_max+(z_range)*0.05
    z_min=z_min
    z_range=z_max-z_min
    

    if cubic_space==True: #Making figure a cube with real scale
    
        max_range=max(x_range, y_range, z_range)
        x_mean=(x_max + x_min)/2.0
        x_max=x_mean + max_range/2.0
        x_min=x_mean- max_range/2.0
        x_range=max_range

        y_mean=(y_max+y_min)/2.0 
        y_max=y_mean + max_range/2.0
        y_min=y_mean - max_range/2.0
        y_range=max_range

        z_max=z_min + max_range
        z_range=max_range

    
    SIZE=int(Drone_size)
    size=[SIZE for k in range(len(X_total))]
    fig= px.scatter_3d(x=X_total, y=Y_total, z=Z_total, animation_frame=Time_total, opacity=1, size=size, color=labels_total, size_max=max(size),color_discrete_sequence=fig_colors)
    
    #Adding lines to the figure
    for j in range(i):
        
        fig.add_trace(            #should be an object of go
            go.Scatter3d(
                x=X_total[j*interp_length: ((j+1)*interp_length)], 
                y=Y_total[j*interp_length: ((j+1)*interp_length)],
                z=Z_total[j*interp_length: ((j+1)*interp_length)], 
                mode='lines',
                name="trace of "+file_names[j],
                marker=dict(color=fig_colors[index_checker(j,len(fig_colors))])
        )
    )
    
    if frame_duration==None:
        frame_duration=(interp_time[1]-interp_time[0]) # in seconds
    fig.layout.updatemenus[0].buttons[0].args[1]['frame']['duration'] = frame_duration*1000 # in milliseconds
    fig.layout.updatemenus[0].buttons[0].args[1]['transition']['duration'] = 1 # in milliseconds
    fig.update_layout(
        showlegend=True,
        legend=dict(itemsizing='constant',font=dict(family="Times New Roman",size=20), bgcolor="LightSteelBlue", bordercolor="Black", borderwidth=2),
        scene_aspectmode='manual',
        scene_aspectratio=dict(x=1, y=y_range/x_range, z=z_range/x_range), 
        scene = dict(
            xaxis = dict(tickmode = 'linear', dtick = int(x_range/Ticks_num), range=[x_min,x_max], visible=True), 
            yaxis = dict(tickmode = 'linear', dtick = int(x_range/Ticks_num), range=[y_min,y_max], visible=True),
            zaxis = dict(tickmode = 'linear', dtick = int(x_range/Ticks_num), range=[z_min,z_max], visible=True)
            ),
        legend_title_text='Drones & traces'
        )

    fig.layout.scene.camera.projection.type = "perspective" # for orthographic projection set it to "orthographic"
    fig.show()

     
visualize_ulg(output_CSV_file_dir='/home/m74744sa/Desktop/All_csvs/Real_shot.csv',folder_of_ulg="/home/m74744sa/Desktop/July_5th_shot",ref_lat= 52.816522986211055, ref_long= -4.1271978280723225, ref_alt= 6,drone_size=15, ticks_num=10, sitl_or_real='real')
#visualize_ulg(output_CSV_file_dir='/path_to_csv_file/csv_file_name.csv', folder_of_ulg='/path_to_folder_containing_ulg_files',ref_lat=latitude of the reference point, ref_long= longitude of the reference point, ref_alt= altitude of the reference point,drone_size= size of drone, ticks_num=number of partitions in the final fig, frame_duration= duration of each frame of animation in seconds)
