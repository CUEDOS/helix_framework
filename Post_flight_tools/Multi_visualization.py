import csv
import plotly.express as px
import plotly.graph_objects as go
from scipy import interpolate
import glob, os
import math

def multi_visualizer (**Input): 
    """
    Draws the path of drones based on the file and animates them in Cartesian coordinate
    Arguments:
        csv_folder: directory of the folder containing all the csv files created by functions visualize_ulg and visualize_path_following
        drone_size: size of drones in visualization
        ticks_num: number of ticks for each cartesian axis
        frame_duration: duratin of each frame of animation (second)

    Returns:
        An animated figure of all drones with interpolated positions
    """
    dt=None
    folder_of_csvs=None
    drone_size=10
    Ticks_num=10
    frame_duration=None
    for key, value in Input.items():
        if key=="folder_of_csvs":
            folder_of_csvs=value
        elif key=="drone_size":
            drone_size=value
        elif key=="ticks_num":
            Ticks_num=value
        elif key=='dt':
            dt=value
        elif key=='frame_duration':
            frame_duration=value
        
    if folder_of_csvs==None:
        print('Error: A directory to folder of containing csv files should be provided')
        return 0
    x_max=-1*math.inf  #for figure range
    x_min=math.inf     #for figure range  
    X_total=[]         #total interpolated x

    y_max=-1*math.inf  #for figure range
    y_min=math.inf     #for figure range
    Y_total=[]         #total interpolated y

    z_max=-1*math.inf  #for figure range
    z_min=0            #for figure range
    Z_total=[]         #total interpolated z
    
    Time_total=[]      #Time span of interpolation used for figure
    interp_time=[]     #Time span of interpolation

    interp_length=0
    Color_total=[]
    
    all_drones={}
    # opening csv files ----------------------
    os.chdir(folder_of_csvs)
    for csv_file in glob.glob("*.csv"):
        # Getting data from gps csv file from a csv file ------
        opened_csv_file=open(folder_of_csvs+"/"+csv_file, newline="")
        Object_of_dictionaries=csv.DictReader(opened_csv_file, delimiter=",")
        for row_dict in Object_of_dictionaries: # we should create the lists when the file is still open
            drone_id=row_dict["drone id"]
            if drone_id not in all_drones:
                all_drones[drone_id]=[[],[],[],[],[]] # x, y, z, timestamp, type of experiment
            
            all_drones[drone_id][0].append(float(row_dict["x(m)"]))
            x_max=max(x_max, float(row_dict["x(m)"]))
            x_min=min(x_min, float(row_dict["x(m)"]))

            all_drones[drone_id][1].append(float(row_dict["y(m)"]))
            y_max=max(y_max, float(row_dict["y(m)"]))
            y_min=min(y_min, float(row_dict["y(m)"]))


            all_drones[drone_id][2].append(float(row_dict["z(m)"]))
            z_max=max(z_max, float(row_dict["z(m)"]))
            z_min=min(z_min, float(row_dict["z(m)"]))

            all_drones[drone_id][3].append(float(row_dict["time(s)"]))
            all_drones[drone_id][4].append(str(row_dict["type of experiment"]))
        
        opened_csv_file.close()

    # End of opening csv files  --------------------- 
    #interpolation
    min_finish_time=math.inf
    max_start_time=-1*math.inf
    for drone_id in all_drones:
        min_finish_time=min(min_finish_time, max(all_drones[drone_id][3]))
        max_start_time=max(max_start_time, min(all_drones[drone_id][3]))
        all_drones[drone_id].append(interpolate.interp1d(all_drones[drone_id][3],all_drones[drone_id][0])) # interpolation of time and x position, all_drones[drone_id][5]
        all_drones[drone_id].append(interpolate.interp1d(all_drones[drone_id][3],all_drones[drone_id][1])) # interpolation of time and y position, all_drones[drone_id][6]
        all_drones[drone_id].append(interpolate.interp1d(all_drones[drone_id][3],all_drones[drone_id][2])) # interpolation of time and z position, all_drones[drone_id][7]
    
    drones=[] # to know the order of the drones in interpolation
    for drone_id in all_drones:
        t=max_start_time
        drones.append(drone_id)
        interp_length=0
        while(t<=min_finish_time):
            X_total.append(float(all_drones[drone_id][5](t))) # using x interpolation for drone
            Y_total.append(float(all_drones[drone_id][6](t))) # using y interpolation for drone
            Z_total.append(float(all_drones[drone_id][7](t))) # using z interpolation for drone
            Time_total.append(t)
            Color_total.append(drone_id+' ('+all_drones[drone_id][4][0]+')')
            t+=dt
            interp_length+=1

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

    
    SIZE=int(drone_size)
    size=[SIZE for k in range(len(X_total))]
    fig= px.scatter_3d(x=X_total, range_x=[x_right_margin,x_left_margin], y=Y_total, range_y=[y_up_margin,y_down_margin], z=Z_total, range_z=[z_down_margin,z_up_margin], animation_frame=Time_total, opacity=1, size=size, color=Color_total, size_max=max(size))
    
    #Adding lines to the figure
    for j in range(len(drones)):
        fig.add_trace(            #should be an object of go
            go.Scatter3d(
                x=X_total[j*interp_length: ((j+1)*interp_length)], 
                y=Y_total[j*interp_length: ((j+1)*interp_length)],
                z=Z_total[j*interp_length: ((j+1)*interp_length)], 
                mode='lines',
                name="trace of "+ drones[j]
        )
    )
    if frame_duration==None:
        frame_duration=dt # in seconds
    fig.layout.updatemenus[0].buttons[0].args[1]['frame']['duration'] = frame_duration*1000 # in milliseconds
    fig.layout.updatemenus[0].buttons[0].args[1]['transition']['duration'] = 1
    fig.update_layout(
        showlegend=True,
        legend=dict(itemsizing='constant',font=dict(family="Times New Roman",size=20), bgcolor="LightSteelBlue", bordercolor="Black", borderwidth=2),
        scene_aspectmode='manual',
        scene_aspectratio=dict(x=1, y=y_range/x_range, z=z_range/x_range), 
        scene = dict(xaxis = dict(nticks=x_parts,range=[x_right_margin,x_left_margin]), yaxis = dict(nticks=math.ceil((y_range/x_range)*x_parts), range=[y_up_margin,y_down_margin]),zaxis = dict(nticks=math.ceil((z_range/x_range)*x_parts),range=[z_down_margin,z_up_margin]))
        )
    fig.show()
    
   
multi_visualizer(folder_of_csvs='/home/m74744sa/Desktop/All_csvs',drone_size=10, ticks_num=10,dt=0.1)

