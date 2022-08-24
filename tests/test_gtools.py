from helix_framework.gtools import (
    alt_calc,
    proximity_check,
)  # importing the module we want to test its function (the test file should be in the same directory as module file)
import pytest
import numpy as np
from helix_framework.data_structures import AgentTelemetry
from math import sqrt

# test of alt_dict function ------------------------------------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "dict_in, site_elevation, dict_out",
    [
        (
            {
                "P101": 22,
                "P102": 20,
                "P103": 26,
                "P104": 24,
                "P105": 30,
                "P106": 28,
            },  # input Set1: even number of drones
            10,  # site_elevation Set1
            {
                "P102": 20.0,
                "P101": 22.0,
                "P104": 24.0,
                "P103": 26.0,
                "P106": 28.0,
                "P105": 30.0,
            },
        ),  # output Set1
        (
            {
                "P101": 32,
                "P102": 30,
                "P103": 35,
                "P104": 35,
                "P105": 40,
                "P106": 38,
            },  # input Set2: even number of drones, two same altitudes
            20,  # site_elevation Set2
            {
                "P102": 30.0,
                "P101": 32.0,
                "P103": 34.0,
                "P104": 36.0,
                "P106": 38.0,
                "P105": 40.0,
            },
        ),  # output Set2
        (
            {
                "P101": 483,
                "P102": 482,
                "P103": 484,
                "P104": 483,
                "P105": 483,
                "P106": 483,
                "P107": 489.5,
                "P108": 483,
                "P109": 491,
                "P110": 482,
            },  # Set3: 10 drones, mean below min+site_elevation
            480,  # site_elevation Set3
            {
                "P102": 490.0,
                "P110": 492.0,
                "P101": 494.0,
                "P104": 496.0,
                "P105": 498.0,
                "P106": 500.0,
                "P108": 502.0,
                "P103": 504.0,
                "P107": 506.0,
                "P109": 508.0,
            },
        ),  # output Set3
        (
            {
                "P101": 10,
                "P102": 5,
                "P103": 11,
                "P104": 10,
                "P105": 4,
            },  # input Set4: odd number of drones, two same altitudes
            -10,  # site_elevation Set4
            {"P105": 4.0, "P102": 6.0, "P101": 8.0, "P104": 10.0, "P103": 12.0},
        ),  # output Set4
        (
            {
                "P101": 0,
                "P102": 0,
                "P103": 0,
                "P104": 0,
                "P105": 0,
                "P106": 0,
                "P107": 0,
                "P108": 0,
            },  # input Set5: 8 drones, all same altitudes
            -20,  # site_elevation Set5
            {
                "P101": -7.0,
                "P102": -5.0,
                "P103": -3.0,
                "P104": -1.0,
                "P105": 1.0,
                "P106": 3.0,
                "P107": 5.0,
                "P108": 7.0,
            },
        ),  # output Set5
        (
            {
                "P101": 1030,
                "P102": 30,
                "P103": 31,
                "P104": 31,
                "P105": 31,
                "P106": 20030,
                "P107": 30,
            },  # input Set6: 7 drones, three & two same altitudes, huge differences
            30,  # site_elevation Set6
            {
                "P102": 118.0,
                "P107": 120.0,
                "P103": 122.0,
                "P104": 124.0,
                "P105": 126.0,
                "P101": 128.0,
                "P106": 130.0,
            },
        ),  # output Set6
        (
            {
                "P101": 1003,
                "P102": 1002,
                "P103": 1004,
                "P104": 1003,
                "P105": 1003,
                "P106": 1003,
                "P107": 1009.5,
                "P108": 1003,
                "P109": 1011,
                "P110": 1002,
            },  # input Set7:# 10 drones, four & two same altitudes
            1000,  # site_elevation Set7
            {
                "P102": 1010.0,
                "P110": 1012.0,
                "P101": 1014.0,
                "P104": 1016.0,
                "P105": 1018.0,
                "P106": 1020.0,
                "P108": 1022.0,
                "P103": 1024.0,
                "P107": 1026.0,
                "P109": 1028.0,
            },
        ),
    ],  # output Set7
)
def test_alt_calc(dict_in, site_elevation, dict_out):
    assert alt_calc(dict_in, site_elevation) == dict_out


# test of proximity_check function -----------------------------------------------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "positions, min_proximity, output",
    [
        (
            [
                [5, 5, 5],
                [5, 5, 5],
                [5, 5, 5],
                [5, 5, 5],
            ],  # input Set1: 4 drones with the same position
            2,  # min_proximity Set1
            [
                ["P101", "P102", 0],
                ["P101", "P103", 0],
                ["P101", "P104", 0],
                ["P102", "P103", 0],
                ["P102", "P104", 0],
                ["P103", "P104", 0],
            ],
        ),  # output Set1
        (
            [
                [5.5, 4.5, 5],
                [6, 7.2, 8],
                [10, 7.2, 5],
                [-1, -1.5, -0.5],
                [5.3, 5, 4.9],
            ],  # input Set2: 5 drones
            2,  # min_proximity Set2
            [["P101", "P105", 0.548]],
        ),  # output Set2
        (
            [
                [-5.5, -4.5, -5],
                [-6, -7.2, -8],
                [-10, -7.2, -5],
                [-11.1, -8.3, -6.1],
                [5.3, 5, 4.9],
                [-5.3, -5, -4.9],
            ],  # input Set3: 6 drones
            2,  # min_proximity Set3
            [["P101", "P106", 0.548], ["P103", "P104", 1.905]],
        ),  # output Set3
        (
            [
                [1, 1.5, 2],
                [-1, -1.5, 2],
                [2.2, 2.7, 3.2],
                [-2.2, -2.7, -3.2],
                [3.4, 3.9, 4.4],
                [-3.4, -3.9, -4.4],
                [0, 0, 0],
            ],
            2,  # min_proximity Set4
            [],
        ),  # output Set4
    ],
)
def test_proximity_check(positions, min_proximity, output):
    n = len(positions)
    swarm_telem = {}
    for i in range(n):
        agent = AgentTelemetry()
        agent.position_ned = positions[i]
        swarm_telem["P" + str(101 + i)] = agent
    print("swarm_telem=", swarm_telem)

    Function_output = proximity_check(swarm_telem, min_proximity)
    for value in Function_output:
        value[2] = round(value[2], 3)

    assert Function_output == output
