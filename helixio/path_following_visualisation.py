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
    drone_size=10
    Ticks_num=10
    simulation_time=200
    dt=0.1
    drone_num = 2
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
    X_total=[] # positions of all drones along x
    Y_total=[] # positions of all drones along y
    Z_total=[] # positions of all drones along z

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
    Color_total=[]  
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

            drones[id][1].append(swarm_telem[id].position_ned[0])
            x_min=min(x_min,swarm_telem[id].position_ned[0])
            x_max=max(x_max,swarm_telem[id].position_ned[0])

            drones[id][2].append(swarm_telem[id].position_ned[1])
            y_min=min(y_min,swarm_telem[id].position_ned[1])
            y_max=max(y_max,swarm_telem[id].position_ned[1])

            drones[id][3].append(-1*swarm_telem[id].position_ned[2])
            z_min=min(z_min,-1*swarm_telem[id].position_ned[2])
            z_max=max(z_max,-1*swarm_telem[id].position_ned[2])

            drones[id][4].append(t) 

    # Preparing Final figure ---------------------------------------------------------
    for id in swarm_telem:
        for i in range(simulation_steps):
            X_total.append(drones[id][1][i])
            Y_total.append(drones[id][2][i])
            Z_total.append(drones[id][3][i])
            Time_total.append(drones[id][4][i])
            Color_total.append(id)
            
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
    for j in range(len(drone_ids)):
        fig.add_trace(            #should be an object of go
            go.Scatter3d(
            x=X_total[j*simulation_steps: ((j+1)*simulation_steps)-1], 
            y=Y_total[j*simulation_steps: ((j+1)*simulation_steps)-1],
            z=Z_total[j*simulation_steps: ((j+1)*simulation_steps)-1], 
            mode='lines',
            name="trace of "+drone_ids[j]
    )
    )
    fig.layout.updatemenus[0].buttons[0].args[1]['frame']['duration'] = dt*1000 # in milliseconds
    fig.layout.updatemenus[0].buttons[0].args[1]['transition']['duration'] = 1 # in milliseconds
    fig.update_layout(
        showlegend=True,
        legend=dict(itemsizing='constant',font=dict(family="Times New Roman",size=20), bgcolor="LightSteelBlue", bordercolor="Black", borderwidth=2),
        scene_aspectmode='manual',
        scene_aspectratio=dict(x=1, y=y_range/x_range, z=z_range/x_range), 
        scene = dict(xaxis = dict(nticks=x_parts,range=[x_right_margin,x_left_margin]), yaxis = dict(nticks=math.ceil((y_range/x_range)*x_parts), range=[y_up_margin,y_down_margin]),zaxis = dict(nticks=math.ceil((z_range/x_range)*x_parts),range=[z_down_margin,z_up_margin]))
        )
    fig.show()

visualize_path_following(drone_num = 6, dt=0.2, experiment_file_path='/home/m74744sa//Documents/Helixio/helixio/helixio/experiment_3.json')