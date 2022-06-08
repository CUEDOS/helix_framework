from experiment import Experiment
from data_structures import AgentTelemetry
from mavsdk.offboard import VelocityNedYaw
from onboard import Agent
import numpy as np
import math
import plotly.express as px
import plotly.graph_objects as go
# Simulation parameters
t=0
dt=0.1
simulation_time=100
drone_num = 6
experiment_file_path="experiment_2.json" # file of parameters
x_max=-math.inf
x_min=math.inf
y_max=-math.inf
y_min=math.inf
z_max=-math.inf
z_min=math.inf
max_speed=5
time_step=0.1
max_accel=5

swarm_telem = {}
drones={}
# Creating swarm_telem dictionary
for i in range(drone_num):
    id = str('S'+ str(i+1).zfill(3))
    swarm_telem[id]=AgentTelemetry()


# Creating drones dictionary
for id in swarm_telem:
    drones.update({id: [Experiment(id,swarm_telem,experiment_file_path),[],[],[]]}) #{id: Experiment object, [record of x], [record of y] [record of z]}

while (t<=simulation_time):
    t+=dt
    # Calculating target velocities
    for id in swarm_telem:
        velocity_ned_yaw = drones[id][0].path_following(swarm_telem, max_speed, time_step, max_accel)  # object of class VelocityNedYaw
        drones[id][0].velocity_ned =np.array([velocity_ned_yaw.north_m_s,velocity_ned_yaw.east_m_s,velocity_ned_yaw.down_m_s])
    # Integrating
    for id in swarm_telem:
        swarm_telem[id].position_ned=np.array(swarm_telem[id].position_ned) + drones[id][0].velocity_ned*dt
        drones[id][1].append(swarm_telem[id].position_ned[0]) # x recorder for drone with this id
        drones[id][2].append(swarm_telem[id].position_ned[1]) # y recorder for drone with this id
        drones[id][3].append(-1*swarm_telem[id].position_ned[2]) # z recorder for drone with this id

        x_min=min(x_min,swarm_telem[id].position_ned[0])
        x_max=max(x_max,swarm_telem[id].position_ned[0])

        y_min=min(y_min,swarm_telem[id].position_ned[1])
        y_max=max(y_max,swarm_telem[id].position_ned[1])

        z_min=min(z_min,-1*swarm_telem[id].position_ned[2])
        z_max=max(z_max,-1*swarm_telem[id].position_ned[2])

# Preparing Final figure
data=()
for id in swarm_telem:
    drones[id][0].fig= px.scatter_3d(x=drones[id][1], y=drones[id][2], z=drones[id][3], size_max=1, opacity=1)
    drones[id][0].fig.update_traces(marker=dict(size=1.5,))
    data=data+drones[id][0].fig.data

Total = go.Figure(data)
x_right_margin=x_max+(x_max-x_min)*0.05
x_left_margin=x_min-(x_max-x_min)*0.05
x_parts=10
x_range=(x_right_margin-x_left_margin)

y_up_margin=y_max+(y_max-y_min)*0.05
y_down_margin=y_min-(y_max-y_min)*0.05
y_range=(y_up_margin-y_down_margin)

z_up_margin=z_max+(z_max-z_min)*0.05
z_down_margin=z_min
z_range=(z_up_margin-z_down_margin)

Total.update_layout(scene_aspectmode='manual',scene_aspectratio=dict(x=1, y=y_range/x_range, z=z_range/x_range), scene = dict(xaxis = dict(nticks=x_parts,range=[x_right_margin,x_left_margin]), yaxis = dict(nticks=math.ceil((y_range/x_range)*x_parts), range=[y_up_margin,y_down_margin]),zaxis = dict(nticks=math.ceil((z_range/x_range)*x_parts),range=[z_down_margin,z_up_margin])))
Total.show()