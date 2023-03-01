import numpy as np
import gymnasium
import control

from PIL import Image
from PF_utils.linModel import *
from PF_utils.abstractions import precom, getCtrl, getRefs, readDS4


# r = readDS4() # For PS4 controller live inputs

# Longitudinal weighting matrices
p_long = 1000
Q_long = p_long*np.matmul(np.transpose(C_long), C_long)
R_long = np.eye(1)
# Longitudinal LQR gains
K_long, _, _ = control.lqr(A_long, B_long, Q_long, R_long)
Nbar_long = precom(A_long, B_long, C_long, K_long)


# Lateral weighting matrices
p_lat = 200
Q_lat = p_lat*np.matmul(np.transpose(C_lat), C_lat)
R_lat = np.eye(2)
# Lateral LQR gains
K_lat, _, _ = control.lqr(A_lat, B_lat, Q_lat, R_lat)
Nbar_lat = precom(A_lat, B_lat, C_lat, K_lat)

# Carrot chasing algorithm parameters
Kpsi = 1
Ktheta = 0.2






envs = gymnasium.make("PyFlyt/Fixedwing-DubinsPath-v0", render_mode="human", angle_representation='euler', flight_dome_size=200, turning_radius=40)

terminated = False
truncated = False
aviary_options = {"cameraTargetPosition":[-10, -10, 30]}
next_obs, _ =envs.reset(aviary_options=aviary_options)
imgs_array = []


if __name__ == "__main__":
    # imgs_array.append(envs.render()[..., :3].astype(np.uint8))

    # Get states from last time step
    obs = next_obs['attitude']
    carrot_pos = next_obs['carrot_pos']
    s_lat = [[obs[6]], [obs[1]], [obs[2]], [obs[4]]] # [v, p, r, phi]
    s_long = [[obs[7]], [obs[8]], [obs[0]], [obs[3]]] # [u, w, q, theta]

    # Get references from carrot chasing algorithm (carrot_pos, UAV_pos, UAV_ang, Kpsi, Ktheta)
    ref_lat, ref_long = getRefs(carrot_pos, obs[9:12], obs[3:6], Kpsi, Ktheta, 1, 1.4)

    ref_lat = [ref_lat, 0] # phi and r
    ref_long = [ref_long] # theta

    # Get control output from lqr control law
    ctrl_lat = getCtrl(s_lat, K_lat, Nbar_lat, ref_lat)
    ctrl_long = getCtrl(s_long, K_long, Nbar_long, ref_long)

    rolls = ctrl_lat[0][0]
    yaws = ctrl_lat[1][0]
    pitchs = ctrl_long[0][0]
    throttles = 0.6

    cmds = np.array([-pitchs, rolls, yaws, throttles])

    print([obs[7]], obs[3])
    cmds = np.clip(cmds, [-1, -1, -1, 0], 1)
    

    next_obs, reward, terminated, truncated, infos = envs.step(cmds)

print(infos)
imgs = [Image.fromarray(img) for img in imgs_array]
imgs[0].save("FWTarget.gif", save_all=True, append_images=imgs[1:], duration=100/3, loop=0)
