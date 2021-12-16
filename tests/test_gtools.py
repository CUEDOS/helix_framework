from helixio.gtools import (
    alt_calc,
    proximity_check,
)  # importing the module we want to test its function (the test file should be in the same directory as module file)
import pytest
import numpy as np
from helixio.communication import AgentTelemetry
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
                "P101": 23.5,
                "P102": 22.5,
                "P103": 25.5,
                "P104": 24.5,
                "P105": 27.5,
                "P106": 26.5,
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
                "P101": 33.5,
                "P102": 32.5,
                "P103": 34.5,
                "P104": 35.5,
                "P105": 37.5,
                "P106": 36.5,
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
                "P101": 492,
                "P102": 490,
                "P103": 497,
                "P104": 493,
                "P105": 494,
                "P106": 495,
                "P107": 498,
                "P108": 496,
                "P109": 499,
                "P110": 491,
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
            {"P101": 8, "P102": 7, "P103": 10, "P104": 9, "P105": 6},
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
                "P101": -3.5,
                "P102": -2.5,
                "P103": -1.5,
                "P104": -0.5,
                "P105": 0.5,
                "P106": 1.5,
                "P107": 2.5,
                "P108": 3.5,
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
                "P101": 129,
                "P102": 124,
                "P103": 126,
                "P104": 127,
                "P105": 128,
                "P106": 130,
                "P107": 125,
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
                "P101": 1012,
                "P102": 1010,
                "P103": 1017,
                "P104": 1013,
                "P105": 1014,
                "P106": 1015,
                "P107": 1018,
                "P108": 1016,
                "P109": 1019,
                "P110": 1011,
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
        ),  # output Set 4
    ],
)
def test_proximity_check(positions, min_proximity, output):
    n = len(positions)
    swarm_telem = {}
    for i in range(0, n):
        agent = AgentTelemetry()
        agent.position_ned = positions[i]
        swarm_telem["P" + str(101 + i)] = agent
    print("swarm_telem=", swarm_telem)

    Function_output = proximity_check(swarm_telem, min_proximity)
    for value in Function_output:
        value[2] = round(value[2], 3)

    assert Function_output == output
