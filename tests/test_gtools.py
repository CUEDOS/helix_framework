from helixio.gtools import alt_calc, proximity_check # importing the module we want to test its function (the test file should be in the same directory as module file)
import pytest

@pytest.mark.parametrize('dict_in, dict_out',[
	({"P101":12, "P102":10, "P103":16, "P104":14, "P105":20, "P106":18}, {"P101":13.5, "P102":12.5, "P103":15.5, "P104":14.5, "P105":17.5, "P106":16.5}), # Set1: even number of drones
	({"P101":12, "P102":10, "P103":15, "P104":15, "P105":20, "P106":18} ,{"P101":13.5, "P102":12.5, "P103":14.5, "P104":15.5, "P105":17.5, "P106":16.5}), # Set2: even number of drones, two same altitudes
	({"P101":3, "P102":2, "P103":4, "P104":3, "P105":3, "P106":3, "P107":9.5, "P108":3, "P109":11, "P110":2},{"P101":12, "P102":10, "P103":17, "P104":13, "P105":14, "P106":15, "P107":18, "P108":16, "P109":19, "P110":11}), # Set3: odd number of drones
	({"P101":20, "P102":15, "P103":21, "P104":20, "P105":14},{"P101":18, "P102":17, "P103":20, "P104":19, "P105":16}), # Set4: odd number of drones, two same altitudes
	({"P101":20, "P102":20, "P103":20, "P104":20, "P105":20, "P106":20, "P107":20, "P108":20},{"P101":16.5, "P102":17.5, "P103":18.5, "P104":19.5, "P105":20.5, "P106":21.5, "P107":22.5, "P108":23.5}), # Set5:8 drones, all same altitudes
	({"P101":1000, "P102":0, "P103":1, "P104":1, "P105":1, "P106":20000, "P107":0},{"P101":99, "P102":94, "P103":96, "P104":97, "P105":98, "P106":100, "P107":95}), # Set6: 7 drones, three & two same altitudes, huge differences
	({"P101":3, "P102":2, "P103":4, "P104":3, "P105":3, "P106":3, "P107":9.5, "P108":3, "P109":11, "P110":2},{"P101":12, "P102":10, "P103":17, "P104":13, "P105":14, "P106":15, "P107":18, "P108":16, "P109":19, "P110":11}) # Set7: 10 drones, four & two same altitudes
])
def test_alt_calc (dict_in, dict_out):
	assert alt_calc(dict_in)==dict_out

# -------------------------------------------------------------------------------------

class AgentTelemetry:
    arm_status = False
    geodetic = [0, 0, 0]
    position_ned = [0, 0, 0]
    velocity_ned = [0, 0, 0]

swarm_telemetry={"P101": None, "P102": None, "P103": None, "P104": None}
for key in swarm_telemetry.keys():
    swarm_telemetry[key] = AgentTelemetry()

swarm_telemetry["P101"].position_ned= [5,5,5]
swarm_telemetry["P102"].position_ned= [5,5,5]
swarm_telemetry["P103"].position_ned= [5,5,5]
swarm_telemetry["P104"].position_ned= [5,5,5]
output=[["P101", "P102", 0], ["P101", "P103", 0], ["P101", "P104", 0], ["P102", "P103", 0], ["P102", "P104", 0], ["P103", "P104", 0]]
	
@pytest.mark.parametrize('swarm_telem, output_dict',[
	(swarm_telemetry,output)
])
def test_proximity_check (swarm_telem, output_dict):
	min_proximity=2
	assert proximity_check(swarm_teleme, min_proximity)==output_dict
