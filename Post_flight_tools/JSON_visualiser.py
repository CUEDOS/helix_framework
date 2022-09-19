import plotly.express as px
import plotly.graph_objects as go
import math
import json

def JSON_visualiser (**Input):  # input keyword arguments: ref_lat, ref_long, ref_alt
    """
    Draws the corridors
    Arguments:
        JSON_file_dir: directory of the JSON file
        ticks_num: number of ticks for each cartesian axis
    Returns:
        A figure of all the corridors with their number
    """

    JSON_file_dir=None
    Ticks_num=10
    for key, value in Input.items():
        
        if key=="ticks_num":
            Ticks_num=value

        elif key=='JSON_file_dir':
            JSON_file_dir=value
        elif key=='frame_duration':
            frame_duration=value
        
    if JSON_file_dir==None:
        print('Error: A directory to folder of input JSON file should be provided')
        return 0
    with open(JSON_file_dir, "r") as f:
        experiment_parameters = json.load(f)

    corridor_points= experiment_parameters["corridor_points"]

    x_max=-math.inf
    x_min=math.inf

    y_max=-math.inf
    y_min=math.inf

    z_max=-math.inf
    z_min=0

    fig= px.scatter_3d(x=[], y=[], z=[], opacity=1)
    
    #Adding lines to the figure
    for j in range(len(corridor_points)):
        X_total=[]
        Y_total=[]
        Z_total=[]
        for k in range(len(corridor_points[j])):

            X_total.append(corridor_points[j][k][0])
            x_max=max(x_max,corridor_points[j][k][0])
            x_min=min(x_min,corridor_points[j][k][0])

            Y_total.append(corridor_points[j][k][1])
            y_max=max(y_max,corridor_points[j][k][1])
            y_min=min(y_min,corridor_points[j][k][1])
  
            Z_total.append(-1* corridor_points[j][k][2])
            z_max=max(z_max,-1*corridor_points[j][k][2])
            z_min=min(z_min,-1*corridor_points[j][k][2])
           
        fig.add_trace(           
            go.Scatter3d(
                x=X_total, 
                y=Y_total,
                z=Z_total, 
                mode='lines',
                name="path "+str(j),
            )
        )
    
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

    fig.update_layout(
        showlegend=True,
        legend=dict(itemsizing='constant',font=dict(family="Times New Roman",size=20), bgcolor="LightSteelBlue", bordercolor="Black", borderwidth=2),
        scene_aspectmode='manual',
        scene_aspectratio=dict(x=1, y=1, z=1), 
        scene = dict(xaxis = dict(nticks=Ticks_num,range=[x_right_margin,x_left_margin], visible=False), yaxis = dict(nticks=Ticks_num, range=[y_up_margin,y_down_margin], visible=False),zaxis = dict(nticks=Ticks_num,range=[z_down_margin,z_up_margin], visible=False)),
        legend_title_text='Paths',
       )

    fig.layout.scene.camera.projection.type = "orthographic"
    fig.show()
    
     
JSON_visualiser(JSON_file_dir='/home/m74744sa/Documents/helix_framework/helix_framework/experiments/divergence_S_to_N_NZ.json',ticks_num=10)
#JSON_visualiser(JSON_file_dir='/path_to_csv_file/csv_file_name.csv', ticks_num=number of partitions in the final fig)
