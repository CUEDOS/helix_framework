from experiment import Experiment
from data_structures import AgentTelemetry
from mavsdk.offboard import VelocityNedYaw
from onboard import Agent
import numpy as np
import math
import plotly.express as px
import plotly.graph_objects as go
import csv
import json

def visualize_path_following (**Input):
    """
    Draws the path of drones based on path following code, and animates them in Cartesian coordinate
    Arguments:
        JSON_file_path: the path to input json file of experiment containing prestart positions, corridor points, pass permission, etc.
        output_CSV_file_dir: the path to output CSV file containg position, drone id, time stamp and type of experimetn
        simulation_time: simulation duration in seconds
        drone_size: size of drones in visualization
        ticks_num: number of ticks for each cartesian axis
        dt: time step in second
        drone_num: number of drones in Python simulation
        frame_duration: duratin of each frame of animation (second)
        show_corridors: to show corridors or not
        cubic_space: if True, the whole sapce will be a cube
    
        Note: if a user does not provide one arguments of input experiment json file or output csv file, the code shows an error and stops
    Returns:
        An animated figure of all drones of simulation, and a CSV file containing position, drone id, time stamp and type of experiment
    """
    #default parameters value
    drone_size=10
    Ticks_num=5
    simulation_time=300
    dt=0.1
    drone_num = 2
    frame_duration=None
    output_CSV_file_dir=None
    JSON_file_dir=None
    show_corridors=False
    show_annotations=False
    CSV_order="vertical"
    cubic_space=True
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
        elif key=='JSON_file_dir':
            JSON_file_dir=value
        elif key=='output_CSV_file_dir':
            output_CSV_file_dir=value
        elif key=='frame_duration':
            frame_duration=value
        elif key=='show_corridors':
            show_corridors=value
        elif key=='CSV_order':
            CSV_order=value
        elif key=='show_annotations':
            show_annotations=value
        elif key=='cubic_space':
            cubic_space=value
            
    if JSON_file_dir==None:
        print('Error: A directory to input JSON file should be provided')
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
    samllest_min_distance=math.inf
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
        drones.update({id: [Experiment(id,swarm_telem, JSON_file_dir),[],[],[],[]]}) #{id: Experiment object, [record of x], [record of y], [record of z], [record of time]}

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
        t=round(t+dt, 3)
        simulation_steps+=1
        # Calculating target velocities
        for id in swarm_telem:
            velocity_ned_yaw = drones[id][0].path_following(swarm_telem, max_speed, time_step, max_accel)  # object of class VelocityNedYaw
            drones[id][0].velocity_ned =np.array([velocity_ned_yaw.north_m_s,velocity_ned_yaw.east_m_s,velocity_ned_yaw.down_m_s])
            if (t<=drones[id][0].start_delay/1000000):
                drones[id][0].velocity_ned =np.array([0, 0, 0], dtype="float64")
        # Integrating
        for id in swarm_telem:
            swarm_telem[id].position_ned=np.array(swarm_telem[id].position_ned) + drones[id][0].velocity_ned*dt

            drones[id][1].append(swarm_telem[id].position_ned[0])  # East is along x
            x_min=min(x_min,swarm_telem[id].position_ned[0])
            x_max=max(x_max,swarm_telem[id].position_ned[0])

            drones[id][2].append(swarm_telem[id].position_ned[1]) # North is along y
            y_min=min(y_min,swarm_telem[id].position_ned[1])
            y_max=max(y_max,swarm_telem[id].position_ned[1])

            drones[id][3].append(-1*swarm_telem[id].position_ned[2])
            z_min=min(z_min,-1*swarm_telem[id].position_ned[2])
            z_max=max(z_max,-1*swarm_telem[id].position_ned[2])

            drones[id][4].append(t) 

    # Preparing Final figure & output CSV file ---------------------------------------------------------
    fig_colors=['blue','red', 'lightgreen', 'orange','aqua', 'silver', 'magenta', 'darkkhaki','dodgerblue','green','black','brown']
    Output_CSV_file=open(output_CSV_file_dir, 'w')
    writer = csv.writer(Output_CSV_file)

    if (CSV_order=="horizontal"):
        header=['time(s)']
        for id in drone_ids:
            header.append("x(m)"+"_"+id)
            header.append("y(m)"+"_"+id)
            header.append("z(m)"+"_"+id)
        writer.writerow(header)
        # creating horizontal rows
        for i in range(simulation_steps):
            row=[drones[drone_ids[0]][4][i]]
            for id in drone_ids:
                row.append(drones[id][1][i])
                row.append(drones[id][2][i])
                row.append(drones[id][3][i])

            writer.writerow(row)
    else:
        header=['x(m)', 'y(m)', 'z(m)', 'time(s)', 'drone id', 'offboard mode status','type of experiment']
        writer.writerow(header)
    
    # Creatinf a total list of positions for animation and finding the closest drones positons for annotation
    for id in drone_ids:  # to count ids in order
        if drones[id][0].min_distance[0] < samllest_min_distance:
            samllest_min_distance=drones[id][0].min_distance[0]
            closest_drone_1=drones[id][0].min_distance[1]
            closest_drone_1_position=drones[id][0].min_distance[3]
            closest_drone_2=drones[id][0].min_distance[2]
            closest_drone_2_position=drones[id][0].min_distance[4]

        
        print("The least distance between dornes ", id, "and ",drones[id][0].min_distance[2], "=", drones[id][0].min_distance[0])
        for i in range(simulation_steps):
            X_total.append(drones[id][1][i])
            Y_total.append(drones[id][2][i])
            Z_total.append(drones[id][3][i])
            Time_total.append(drones[id][4][i])
            labels_total.append(id)

            if CSV_order=="vertical":
                row=[drones[id][1][i], drones[id][2][i], drones[id][3][i], drones[id][4][i], id, 1, 'Python_simulation'] # x, y , z, time (s), id, offboard mode status, type of experiment
                writer.writerow(row)
    
    
    print("The smallest least distance is between dornes ", closest_drone_1, "and ",closest_drone_2, "=", samllest_min_distance)
    Output_CSV_file.close()
    

    SIZE=int(drone_size)
    size=[SIZE for k in range(len(X_total))]
    fig= px.scatter_3d(x=X_total, y=Y_total, z=Z_total, animation_frame=Time_total, opacity=1, size=size, color=labels_total, size_max=max(size), color_discrete_sequence=fig_colors)

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
    if show_corridors==True:
        #Adding corridors
        with open(JSON_file_dir, "r") as f:
            experiment_parameters = json.load(f)

        corridor_points= experiment_parameters["corridor_points"]
    
        #Adding corridors to the figure
        for j in range(len(corridor_points)):
            X_corridor=[]
            Y_corridor=[]
            Z_corridor=[]
            for k in range(len(corridor_points[j])):

                X_corridor.append(corridor_points[j][k][0])
                x_max=max(x_max,corridor_points[j][k][0])
                x_min=min(x_min,corridor_points[j][k][0])

                Y_corridor.append(corridor_points[j][k][1])
                y_max=max(y_max,corridor_points[j][k][1])
                y_min=min(y_min,corridor_points[j][k][1])
  
                Z_corridor.append(-1* corridor_points[j][k][2])
                z_max=max(z_max,-1*corridor_points[j][k][2])
                z_min=min(z_min,-1*corridor_points[j][k][2])
           
            fig.add_trace(           
                go.Scatter3d(
                    x=X_corridor, 
                    y=Y_corridor,
                    z=Z_corridor, 
                    mode='lines',
                    line=dict(dash='longdash'),
                    name="corridor "+str(j),
                )
            )

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

    

    #Creating annotations
    annotation_list=[]
    if (show_annotations==True):
        for id in drone_ids:
            for swithced_position in drones[id][0].switched_positions:
                annotation_list.append(dict(x=swithced_position[0], y=swithced_position[1], z=-swithced_position[2], text= id+" switched", opacity=0.7, font=dict(color="black",size=5), arrowcolor="black", arrowsize=1, arrowwidth=0.5, arrowhead=1))
            
            # appending the annotations for closest drones
            annotation_list.append(dict(x=closest_drone_1_position[0], y=closest_drone_1_position[1], z=-closest_drone_1_position[2], text= closest_drone_1+" closest", opacity=0.7, font=dict(color="red",size=5), arrowcolor="red", arrowsize=1, arrowwidth=0.5, arrowhead=1))
            annotation_list.append(dict(x=closest_drone_2_position[0], y=closest_drone_2_position[1], z=-closest_drone_2_position[2], text= closest_drone_2+" closest", opacity=0.7, font=dict(color="red",size=5), arrowcolor="red", arrowsize=1, arrowwidth=0.5, arrowhead=1))

    if frame_duration==None:
        frame_duration=dt # in seconds
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
            zaxis = dict(tickmode = 'linear', dtick = int(x_range/Ticks_num), range=[z_min,z_max], visible=True), 
            annotations=annotation_list
            ),
        legend_title_text='Drones & traces'
        )

    fig.layout.scene.camera.projection.type = "perspective" # for orthographic projection set it to "orthographic"
    fig.show()


visualize_path_following(drone_num = 8, dt=0.1, output_CSV_file_dir='/home/m74744sa/Desktop/All_csvs/Python_sim.csv', JSON_file_dir='/home/m74744sa/Documents/helix_framework/helix_framework/experiments/divergence_S_to_N_NZ.json', show_corridors=True, CSV_order="horizontal", show_annotations=True, cubic_space=True)
#visualize_path_following(drone_num = number of drones, dt= time step in sec, frame_duration= duration of each frame of animation in seconds, output_CSV_file_dir='/path_to_output_CSV_file/output_CSV_file_name.csv', experiment_file_path='/path_to_experiment_json_file/json_file_name.json', show_corridors=True, cubic_space=True)