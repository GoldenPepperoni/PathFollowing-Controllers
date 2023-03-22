import numpy as np
import PyFlyt.gym_envs
import gymnasium
import control

from PIL import Image
from pathfollowingcontrollers.PF_utils.linModel import *
from pathfollowingcontrollers.PF_utils.abstractions import *


# r = readDS4() # For PS4 controller live inputs

# Carrot chasing algorithm parameters
Kpsi = 2.5
Ktheta = 0.5

# Create custom figure 8 path
custom_targets = [[0, 50, 10], [-100, 50, 10], [-200, 50, 10], [-100, 50, 10], [0, 50, 10], [-100, 50, 10], [-200, 50, 10], [-100, 50, 10], [0, 50, 10]]
custom_yaw_targets = [np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, np.pi/2,] # (-pi to pi)

# Create and initialise dubins path env
envs = gymnasium.make("PyFlyt/Fixedwing-CCDubinsPath-v0", render_mode=None, angle_representation="euler", flight_dome_size=500, turning_radius=50, num_targets=len(custom_yaw_targets), custom_targets=custom_targets, custom_yaw_targets=custom_yaw_targets)
next_obs, infos =envs.reset(aviary_options={"cameraTargetPosition":[-10, -10, 30]})    
terminated  = False
truncated = False

# Make gif?
makeGif = False
imgs_array = []

# Make plots?
makePlots = False
ctrlTraces = []
actualPath = [next_obs["attitude"][9:13]]
desiredPath = infos["path"]
tArray = []
t = 0

if __name__ == "__main__":
    while not (terminated or truncated):
        # Get states from last time step
        obs = next_obs['attitude']
        carrot_pos = next_obs['carrot_pos']
        # Assemble state vector for longitudinal and lateral linear models
        s_lat = [[obs[6]], [obs[1]], [obs[2]], [obs[4]]] # [v, p, r, phi]
        s_long = [[obs[7]], [obs[8]], [obs[0]], [obs[3]]] # [u, w, q, theta]

        # Get references from carrot chasing algorithm (carrot_pos, UAV_pos, UAV_ang, UAV_vel, Kpsi, Ktheta)
        ref_lat, ref_long = getCCRefs(carrot_pos, obs[9:12], obs[3:6], obs[6:9], Kpsi, Ktheta, 1, 1.4)

        ref_lat = [ref_lat, 0] # phi and r
        ref_long = [ref_long] # theta

        # Get control output from lqr control law
        ctrl_lat = getCtrl(s_lat, K_lat, Nbar_lat, ref_lat)
        ctrl_long = getCtrl(s_long, K_long, Nbar_long, ref_long)

        ail = ctrl_lat[0][0]
        rud = ctrl_lat[1][0]
        elev = ctrl_long[0][0]
        throttle = 0.5

        # Assemble and saturate commands for simulation input
        cmds = np.array([-elev, ail, rud, throttle])
        cmds = np.clip(cmds, [-1, -1, -1, 0], 1)
        
        # Simulation step
        next_obs, reward, terminated, truncated, infos = envs.step(cmds)
        t += 1/30

        # Collect frames if making gif
        if makeGif:
            imgs_array.append(envs.render()[..., :3].astype(np.uint8))
        
        # Collect trajectory and control traces
        actualPath.append(obs[9:13])
        ctrlTraces.append(cmds)
        tArray.append(t)

    if makeGif:
        imgs = [Image.fromarray(img) for img in imgs_array]
        imgs[0].save("LQR_CC_8.gif", save_all=True, append_images=imgs[1:], duration=100/3, loop=0)

    if makePlots:
        plotXY(desiredPath, actualPath, "Horizontal trajectory (LQR_CC_8)")
        plotZ(desiredPath, actualPath, "Vertical trajectory (LQR_CC_8)")
        plot3D(desiredPath, actualPath, "3D trajectory (LQR_CC_8)")
        plotCtrlTraces(ctrlTraces, tArray, "Control traces (LQR_CC_8)")
        plt.show()




        
