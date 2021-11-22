import numpy as np


def simple_flocking(drone_id, swarm_pos_vel, my_pos_vel, max_speed):
    com = np.array([0, 0, 0])
    k_cohesion = 1
    for key in swarm_pos_vel:
        p = np.array(swarm_pos_vel[key].position)
        com = com + p
    com = com / len(swarm_pos_vel)
    v_cohesion = (
        k_cohesion
        * (com - np.array(my_pos_vel.position))
        / np.linalg.norm(com - np.array(my_pos_vel.position))
    )

    r_0 = 10
    v_separation = np.array([0, 0, 0])
    for key in swarm_pos_vel:
        if key == drone_id:
            continue
        p = np.array(swarm_pos_vel[key].position)
        x = np.array(my_pos_vel.position) - p
        d = np.linalg.norm(x)
        v_separation = v_separation + ((x / d) * (r_0 - d) / r_0)

    output_vel = v_cohesion + v_separation
    if np.linalg.norm(output_vel) > max_speed:
        output_vel = output_vel / np.linalg.norm(output_vel) * max_speed

    return output_vel.tolist()
