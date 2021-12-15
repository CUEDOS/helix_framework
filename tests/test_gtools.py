from helixio.gtools import (alt_calc,proximity_check)  # importing the module we want to test its function (the test file should be in the same directory as module file)
import pytest
import numpy as np
from helixio.communication import AgentTelemetry
from math import sqrt

# test of alt_dict function ----------------------------------------------------------------------------------------------------------------------------------
    
#Set5: 8 drones, all same altitudes--------------------------------------------
alt_dict_5={"P101":20, "P102":20, "P103":20, "P104":20, "P105":20, "P106":20, "P107":20, "P108":20}
alt_return_dict_5={"P101":16.5, "P102":17.5, "P103":18.5, "P104":19.5, "P105":20.5, "P106":21.5, "P107":22.5, "P108":23.5}

    
# Set6: 7 drones, three & two same altitudes, huge differences -----------------
alt_dict_6={"P101":1000, "P102":0, "P103":1, "P104":1, "P105":1, "P106":20000, "P107":0}
alt_return_dict_6={"P101":99, "P102":94, "P103":96, "P104":97, "P105":98, "P106":100, "P107":95}

 
#Set7:# 10 drones, four & two same altitudes  ----------------------------------
alt_dict_7={"P101":3, "P102":2, "P103":4, "P104":3, "P105":3, "P106":3, "P107":9.5, "P108":3, "P109":11, "P110":2}
alt_return_dict_7={"P101":12, "P102":10, "P103":17, "P104":13, "P105":14, "P106":15, "P107":18, "P108":16, "P109":19, "P110":11}

@pytest.mark.parametrize(
    "dict_in, site_elevation, dict_out",
    [({"P101": 22, "P102": 20, "P103": 26, "P104": 24, "P105": 30, "P106": 28}, # input Set1: even number of drones
      10, 
      {"P101": 23.5, "P102": 22.5, "P103": 25.5, "P104": 24.5, "P105": 27.5, "P106": 26.5}), # output Set1
     
     ({"P101": 32, "P102": 30, "P103": 35, "P104": 35, "P105": 40, "P106": 38}, # input Set2: even number of drones, two same altitudes 
      20, 
      {"P101": 33.5, "P102": 32.5, "P103": 34.5, "P104": 35.5, "P105": 37.5, "P106": 36.5}), # output Set2
     
     ({"P101": 483, "P102": 482, "P103": 484, "P104": 483, "P105": 483, "P106": 483, "P107": 489.5, "P108": 483, "P109": 491, "P110": 482}, # Set3: 10 drones, mean below min+site_elevation
      480, 
      "P101": 492, "P102": 490, "P103": 497, "P104": 493, "P105": 494, "P106": 495, "P107": 498, "P108": 496, "P109": 499, "P110": 491}), # output Set3
     
     ({"P101": 10, "P102": 5, "P103": 11, "P104": 10, "P105": 4}, # input Set4: odd number of drones, two same altitudes
      -10,
      {"P101": 8, "P102": 7, "P103": 10, "P104": 9, "P105": 6}),
     (alt_dict_5, -20, alt_return_dict_5),
     (alt_dict_6, 30, alt_return_dict_6),
     (alt_dict_7, 1000, alt_return_dict_7)]
)

def test_alt_calc(dict_in, site_elevation, dict_out):
    assert alt_calc(dict_in,site_elevation) == dict_out

#test of proximity_check function -----------------------------------------------------------------------------------------------------------------------------

# Set1: 4 drones with the same position------------------------
swarm_telemetry_1 = {"P101": None,"P102": None, "P103": None, "P104": None} 
for key in swarm_telemetry_1.keys():
    swarm_telemetry_1[key] = AgentTelemetry()

swarm_telemetry_1["P101"].position_ned = [5, 5, 5]
swarm_telemetry_1["P102"].position_ned = [5, 5, 5]
swarm_telemetry_1["P103"].position_ned = [5, 5, 5]
swarm_telemetry_1["P104"].position_ned = [5, 5, 5]
output_1 = [["P101", "P102", 0],["P101", "P103", 0],["P101", "P104", 0],["P102", "P103", 0],["P102", "P104", 0],["P103", "P104", 0]]

# Set2: 5 drones---------------------------
swarm_telemetry_2 = {"P101": None, "P102": None, "P103": None,"P104": None,"P105": None}  
for key in swarm_telemetry_2.keys():
    swarm_telemetry_2[key] = AgentTelemetry()

swarm_telemetry_2["P101"].position_ned = [5.5, 4.5, 5]
swarm_telemetry_2["P102"].position_ned = [6, 7.2, 8]
swarm_telemetry_2["P103"].position_ned = [10, 7.2, 5]
swarm_telemetry_2["P104"].position_ned = [-1, -1.5, -0.5]
swarm_telemetry_2["P105"].position_ned = [5.3, 5, 4.9]
output_2 = [["P101", "P105",0.548]]

# Set3: 6 drones---------------------------
swarm_telemetry_3 = {"P101": None, "P102": None, "P103": None,"P104": None,"P105": None, "P106": None}  
for key in swarm_telemetry_3.keys():
    swarm_telemetry_3[key] = AgentTelemetry()

swarm_telemetry_3["P101"].position_ned = [-5.5, -4.5, -5]
swarm_telemetry_3["P102"].position_ned = [-6, -7.2, -8]
swarm_telemetry_3["P103"].position_ned = [-10, -7.2, -5]
swarm_telemetry_3["P104"].position_ned = [-11.1, -8.3, -6.1]
swarm_telemetry_3["P105"].position_ned = [5.3, 5, 4.9]
swarm_telemetry_3["P106"].position_ned = [-5.3, -5, -4.9]
output_3 = [["P101", "P106",0.548],["P103", "P104", 1.905]]

# Set4: 7 drones---------------------------
swarm_telemetry_4 = {"P101": None, "P102": None, "P103": None,"P104": None,"P105": None, "P106": None, "P107": None}  
for key in swarm_telemetry_4.keys():
    swarm_telemetry_4[key] = AgentTelemetry()

swarm_telemetry_4["P101"].position_ned = [1, 1.5, 2]
swarm_telemetry_4["P102"].position_ned = [-1, -1.5, 2]
swarm_telemetry_4["P103"].position_ned = [2.2, 2.7, 3.2]
swarm_telemetry_4["P104"].position_ned = [-2.2, -2.7, -3.2]
swarm_telemetry_4["P105"].position_ned = [3.4, 3.9, 4.4]
swarm_telemetry_4["P106"].position_ned = [-3.4, -3.9, -4.4]
swarm_telemetry_4["P107"].position_ned = [0, 0, 0]
output_4 = []

@pytest.mark.parametrize(
    "swarm_telemetry, output",
    [(swarm_telemetry_1, output_1), 
     (swarm_telemetry_2, output_2),
     (swarm_telemetry_3, output_3),
     (swarm_telemetry_4, output_4)]   
)

def test_proximity_check(swarm_telemetry, output):
    min_proximity = 2
    Function_output=proximity_check(swarm_telemetry, min_proximity)
    for value in Function_output:
        value[2]=round(value[2],3)
    
    assert  Function_output== output
    
    
    
    # New code lots of errors, need fixing


# -------------Set1

swarm_telemetry_1 = {
    "P101": None,
    "P102": None,
    "P103": None,
    "P104": None,
}  # Set1: 4 drones, all at the same position
for key in swarm_telemetry_1.keys():
    swarm_telemetry_1[key] = AgentTelemetry()

swarm_telemetry_1["P101"].position_ned = [5, 5, 5]
swarm_telemetry_1["P102"].position_ned = [5, 5, 5]
swarm_telemetry_1["P103"].position_ned = [5, 5, 5]
swarm_telemetry_1["P104"].position_ned = [5, 5, 5]
output_1 = [
    ["P101", "P102", 0],
    ["P101", "P103", 0],
    ["P101", "P104", 0],
    ["P102", "P103", 0],
    ["P102", "P104", 0],
    ["P103", "P104", 0],
]

# --------------Set2
swarm_telemetry_2 = {
    "P101": None,
    "P102": None,
    "P103": None,
    "P104": None,
    "P105": None,
}  # Set2: 5 drones, different positions
for key in swarm_telemetry_2.keys():
    swarm_telemetry_2[key] = AgentTelemetry()


output_2 = [
    ["P101", "P105", np.linalg.norm([0.2, 0.5, 0.1])]
]  # output_2=[["P101", "P105", sqrt(0.3)]]


@pytest.mark.parametrize(
    "positions, min_proximity, output",
    [
        (
            [[5.5, 4.5, 5]
            [10, 7.2, 8]
            [12, 7.2, 5]
            [-1, -1.5, -0.5]
            [5.3, 5, 4.9]],
            2,
            
        )
    ],
)
def test_proximity_check(positions, min_proximity, output):
    n = len(posiitons)
    swarm_telem = {}
    for i in range(1, n):
        agent = AgentTelemetry();
        agent.ned = position(i)
        swarm_telem[i] = agent 
    
    assert proximity_check(swarm_telemetry, min_proximity) == output

