from experiment import Experiment
from data_structures import AgentTelemetry
from mavsdk.offboard import VelocityNedYaw
from onboard import Agent
import numpy as np
import math
import plotly.express as px
import plotly.graph_objects as go
import csv

def visualize_path_following (**Input):
    """
    Draws the path of drones based on path following code, and animates them in Cartesian coordinate
    Arguments:
        experiment_file_path: the path to input json file of experiment containing prestart positions, corridor points, pass permission, etc.
        output_CSV_file_dir: the path to output CSV file containg position, drone id, time stamp and type of experimetn
        simulation_time: simulation duration in seconds
        drone_size: size of drones in visualization
        ticks_num: number of ticks for each cartesian axis
        dt: time step in second
        drone_num: number of drones in Python simulation
        frame_duration: duratin of each frame of animation (second)
    
        Note: if a user does not provide one arguments of input experiment json file or output csv file, the code shows an error and stops
    Returns:
        An animated figure of all drones of simulation, and a CSV file containing position, drone id, time stamp and type of experiment
    """
    drone_size=10
    Ticks_num=10
    simulation_time=100
    dt=0.1
    drone_num = 2
    frame_duration=None
    output_CSV_file_dir=None
    experiment_file_path=None
    for key, value in Input.items():
        if key=="simulation_time":
            simulation_time=value
        elif key=="drone_size":
            drone_size=value
        elif key=="ticks_num":
            Ticks_num=value
        elif key=="dt":
            dt=value
        elif key=="drone_num":
            drone_num=value
        elif key=='experiment_file_path':
            experiment_file_path=value
        elif key=='output_CSV_file_dir':
            output_CSV_file_dir=value
        elif key=='frame_duration':
            frame_duration=value
            
    if experiment_file_path==None:
        print('Error: A directory to input experiment file should be provided')
        return 0
    if output_CSV_file_dir== None:
        print('Error: A directory to output CSV file should be provided')
        return 0
    
    def index_checker(input_index, length) -> int:
        if input_index >= length:
            return int(input_index % length)
        return input_index
    #Simulation ------------------------------------------------------------
    # Simulation parameters
    t=0
    x_max=-math.inf
    x_min=math.inf
    y_max=-math.inf
    y_min=math.inf
    z_max=-math.inf
    z_min=0
    max_speed=5
    time_step=dt
    max_accel=5
    X_total=[] # positions of all drones along x used for drawing figure
    Y_total=[] # positions of all drones along y used for drawing figure
    Z_total=[] # positions of all drones along z used for drawing figure

    swarm_telem = {}
    drones={}
    drone_ids=[]
    # Creating swarm_telem dictionary
    for i in range(drone_num):
        id = str('S'+ str(i+1).zfill(3))
        swarm_telem[id]=AgentTelemetry()
        drone_ids.append(id)


    # Creating drones dictionary
    for id in swarm_telem:
        drones.update({id: [Experiment(id,swarm_telem,experiment_file_path),[],[],[],[]]}) #{id: Experiment object, [record of x], [record of y], [record of z], [record of time]}

    # Assigning prestart positions
    for id in swarm_telem:
        prestart_positon=drones[id][0].get_pre_start_positions(swarm_telem, drones[id][0].get_swarm_priorities(swarm_telem))
        swarm_telem[id].position_ned=prestart_positon[id]

    for id in swarm_telem:
        drones[id][0].get_path_and_permission(drones[id][0].get_swarm_priorities(swarm_telem))

    Time_total=[]
    labels_total=[]
    simulation_steps=0
    
    while (t<=simulation_time):
        t+=dt
        simulation_steps+=1
        # Calculating target velocities
        for id in swarm_telem:
            velocity_ned_yaw = drones[id][0].path_following(swarm_telem, max_speed, time_step, max_accel)  # object of class VelocityNedYaw
            drones[id][0].velocity_ned =np.array([velocity_ned_yaw.north_m_s,velocity_ned_yaw.east_m_s,velocity_ned_yaw.down_m_s])
        # Integrating
        for id in swarm_telem:
            swarm_telem[id].position_ned=np.array(swarm_telem[id].position_ned) + drones[id][0].velocity_ned*dt

            drones[id][1].append(swarm_telem[id].position_ned[1])  # East is along x
            x_min=min(x_min,swarm_telem[id].position_ned[1])
            x_max=max(x_max,swarm_telem[id].position_ned[1])

            drones[id][2].append(swarm_telem[id].position_ned[0]) # North is along y
            y_min=min(y_min,swarm_telem[id].position_ned[0])
            y_max=max(y_max,swarm_telem[id].position_ned[0])

            drones[id][3].append(-1*swarm_telem[id].position_ned[2])
            z_min=min(z_min,-1*swarm_telem[id].position_ned[2])
            z_max=max(z_max,-1*swarm_telem[id].position_ned[2])

            drones[id][4].append(t) 

    # Preparing Final figure & output CSV file ---------------------------------------------------------
    fig_colors=['blue','red', 'lightgreen', 'orange','aqua', 'silver', 'magenta', 'darkkhaki','dodgerblue','green','black','brown']
    Output_CSV_file=open(output_CSV_file_dir, 'w')
    writer = csv.writer(Output_CSV_file)
    header=['x(m)', 'y(m)', 'z(m)', 'time(s)', 'drone id', 'offboard mode status','type of experiment']
    writer.writerow(header)

    for id in drone_ids:  # to count ids in order
        for i in range(simulation_steps):
            X_total.append(drones[id][1][i])
            Y_total.append(drones[id][2][i])
            Z_total.append(drones[id][3][i])
            Time_total.append(drones[id][4][i])
            labels_total.append(id)
            
            row=[drones[id][1][i], drones[id][2][i], drones[id][3][i], drones[id][4][i], id, 1, 'Python_simulation'] # x, y , z, time (s), id, offboard mode status, type of experiment
            writer.writerow(row)
    Output_CSV_file.close()

    x_right_margin=x_max+(x_max-x_min)*0.05
    x_left_margin=x_min-(x_max-x_min)*0.05
    x_range=x_right_margin-x_left_margin

    y_up_margin=y_max+(y_max-y_min)*0.05
    y_down_margin=y_min-(y_max-y_min)*0.05
    y_range=y_up_margin-y_down_margin

    z_up_margin=z_max+(z_max-z_min)*0.05
    z_down_margin=z_min
    z_range=z_up_margin-z_down_margin
    
    # Making figure a cube with real scale
    max_range=max(x_range, y_range, z_range)
    x_mean=(x_right_margin+x_left_margin)/2.0
    x_right_margin=x_mean + max_range/2.0
    x_left_margin=x_mean- max_range/2.0
    x_range=max_range

    y_mean=(y_up_margin+y_down_margin)/2.0 
    y_up_margin=y_mean + max_range/2.0
    y_down_margin=y_mean - max_range/2.0
    y_range=max_range

    z_up_margin=z_down_margin + max_range
    z_range=max_range


    SIZE=int(drone_size)
    size=[SIZE for k in range(len(X_total))]
    fig= px.scatter_3d(x=X_total, range_x=[x_right_margin,x_left_margin], y=Y_total, range_y=[y_up_margin,y_down_margin], z=Z_total, range_z=[z_down_margin,z_up_margin], animation_frame=Time_total, opacity=1, size=size, color=labels_total, size_max=max(size),color_discrete_sequence=fig_colors)

    #Adding lines to the figure
    for j in range(len(drone_ids)):
        fig.add_trace(            #should be an object of go
            go.Scatter3d(
            x=X_total[j*simulation_steps: ((j+1)*simulation_steps)], 
            y=Y_total[j*simulation_steps: ((j+1)*simulation_steps)],
            z=Z_total[j*simulation_steps: ((j+1)*simulation_steps)], 
            mode='lines',
            name="trace of "+drone_ids[j],
            marker=dict(color=fig_colors[index_checker(j,len(fig_colors))])

    )
    )

    if frame_duration==None:
        frame_duration=dt # in seconds
    fig.layout.updatemenus[0].buttons[0].args[1]['frame']['duration'] = frame_duration*1000 # in milliseconds
    fig.layout.updatemenus[0].buttons[0].args[1]['frame']['duration'] = frame_duration*1000 # in milliseconds
    fig.layout.updatemenus[0].buttons[0].args[1]['transition']['duration'] = 1 # in milliseconds
    fig.update_layout(
        showlegend=True,
        legend=dict(itemsizing='constant',font=dict(family="Times New Roman",size=20), bgcolor="LightSteelBlue", bordercolor="Black", borderwidth=2),
        scene_aspectmode='manual',
        scene_aspectratio=dict(x=1, y=1, z=1), 
        scene = dict(xaxis = dict(nticks=Ticks_num,range=[x_right_margin,x_left_margin]), yaxis = dict(nticks=Ticks_num, range=[y_up_margin,y_down_margin]),zaxis = dict(nticks=Ticks_num,range=[z_down_margin,z_up_margin])),
        legend_title_text='Drones & traces'
        )
    fig.show()


visualize_path_following(drone_num = 10, dt=0.1, output_CSV_file_dir='/home/m74744sa/Desktop/All_csvs/Python_sim.csv', experiment_file_path='/home/m74744sa/Documents/helix_framework/helix_framework/experiments/Roundabout_S_to_N_NZ.json')
#visualize_path_following(drone_num = number of drones, dt= time step in sec, frame_duration= duration of each frame of animation in seconds, output_CSV_file_dir='/path_to_output_CSV_file/output_CSV_file_name.csv', experiment_file_path='/path_to_experiment_json_file/json_file_name.json')