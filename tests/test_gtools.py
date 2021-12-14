from helixio.gtools import (alt_calc,proximity_check)  # importing the module we want to test its function (the test file should be in the same directory as module file)
import pytest
import numpy as np
from helixio.communication import AgentTelemetry
from math import sqrt

# test of alt_dict function ----------------------------------------------------------------------------------------------------------------------------------
site_elevation=480

# Set1: even number of drones-------------------------------------------------
alt_dict_1={"P101":12, "P102":10, "P103":16, "P104":14, "P105":20, "P106":18}
alt_return_dict_1={"P101":13.5, "P102":12.5, "P103":15.5, "P104":14.5, "P105":17.5, "P106":16.5}

for key in alt_dict_1.keys():
    alt_dict_1[key]+=site_elevation
    alt_return_dict_1[key]+=site_elevation
 
#Set2: even number of drones, two same altitudes------------------------------
alt_dict_2={"P101":12, "P102":10, "P103":15, "P104":15, "P105":20, "P106":18}
alt_return_dict_2={"P101":13.5, "P102":12.5, "P103":14.5, "P104":15.5, "P105":17.5, "P106":16.5}

for key in alt_dict_2.keys():
    alt_dict_2[key]+=site_elevation
    alt_return_dict_2[key]+=site_elevation
 
#Set3: odd number of drones----------------------------------------------------
alt_dict_3={"P101":3, "P102":2, "P103":4, "P104":3, "P105":3, "P106":3, "P107":9.5, "P108":3, "P109":11, "P110":2}
alt_return_dict_3={"P101":12, "P102":10, "P103":17, "P104":13, "P105":14, "P106":15, "P107":18, "P108":16, "P109":19, "P110":11}

for key in alt_dict_3.keys():
    alt_dict_3[key]+=site_elevation
    alt_return_dict_3[key]+=site_elevation
    
#Set4: odd number of drones, two same altitudes--------------------------------
alt_dict_4={"P101":20, "P102":15, "P103":21, "P104":20, "P105":14}
alt_return_dict_4={"P101":18, "P102":17, "P103":20, "P104":19, "P105":16}

for key in alt_dict_4.keys():
    alt_dict_4[key]+=site_elevation
    alt_return_dict_4[key]+=site_elevation
    
#Set5: 8 drones, all same altitudes--------------------------------------------
alt_dict_5={"P101":20, "P102":20, "P103":20, "P104":20, "P105":20, "P106":20, "P107":20, "P108":20}
alt_return_dict_5={"P101":16.5, "P102":17.5, "P103":18.5, "P104":19.5, "P105":20.5, "P106":21.5, "P107":22.5, "P108":23.5}

for key in alt_dict_5.keys():
    alt_dict_5[key]+=site_elevation
    alt_return_dict_5[key]+=site_elevation
    
# Set6: 7 drones, three & two same altitudes, huge differences -----------------
alt_dict_6={"P101":1000, "P102":0, "P103":1, "P104":1, "P105":1, "P106":20000, "P107":0}
alt_return_dict_6={"P101":99, "P102":94, "P103":96, "P104":97, "P105":98, "P106":100, "P107":95}

for key in alt_dict_6.keys():
    alt_dict_6[key]+=site_elevation
    alt_return_dict_6[key]+=site_elevation
 
#Set7:# 10 drones, four & two same altitudes  ----------------------------------
alt_dict_7={"P101":3, "P102":2, "P103":4, "P104":3, "P105":3, "P106":3, "P107":9.5, "P108":3, "P109":11, "P110":2}
alt_return_dict_7={"P101":12, "P102":10, "P103":17, "P104":13, "P105":14, "P106":15, "P107":18, "P108":16, "P109":19, "P110":11}

for key in alt_dict_7.keys():
    alt_dict_7[key]+=site_elevation
    alt_return_dict_7[key]+=site_elevation

@pytest.mark.parametrize(
    "dict_in, dict_out",
    [(alt_dict_1,alt_return_dict_1),
     (alt_dict_2,alt_return_dict_2),
     (alt_dict_3,alt_return_dict_3),
     (alt_dict_4,alt_return_dict_4),
     (alt_dict_5,alt_return_dict_5),
     (alt_dict_6,alt_return_dict_6),
     (alt_dict_7,alt_return_dict_7)]
)
def test_alt_calc(dict_in, dict_out):
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
output_2 = [["P101", "P105",0.547]]

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
output_3 = [["P101", "P106",0.547],["P103", "P104", 1.905]]

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
    x=round(proximity_check(swarm_telemetry, min_proximity),3)
    assert  x== output
