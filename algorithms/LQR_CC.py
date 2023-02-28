import numpy as np
import gymnasium
from PyFlyt.core import Aviary, loadOBJ, obj_collision, obj_visual
import time
from distutils.util import strtobool

import gymnasium as gym
import PyFlyt.gym_envs
from PIL import Image

from pyPS4Controller.controller import Controller
from threading import Thread, Event

import control
from PF_utils.abstractions import precom, getCtrl, getRefs


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)


    def on_R3_down(self, value):
        global pitch

        value = value / 32767

        pitch = value
        return value

    def on_R3_up(self, value):
        global pitch

        value = value / 32767

        pitch = value
        return value

    def on_R3_left(self, value):
        global roll

        value = value / 32767

        roll = value
        return value

    def on_R3_right(self, value):
        global roll

        value = value / 32767

        roll = value
        return value

    def on_L3_left(self, value):
        global yaw

        value = value / 32767

        yaw = value
        return value

    def on_L3_right(self, value):
        global yaw

        value = value / 32767

        yaw = value
        return value

    def on_R2_press(self, value):
        global throttle

        value = value / 32767

        throttle = value
        return value


def readDS4():
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()
    
t = Thread(target=readDS4, args=())
t.start()

roll = 0
pitch = 0.1
yaw = 0
throttle = 0.6

cmds = [roll, pitch, yaw, throttle]

# Longitudinal 
A_long = np.array([[-0.0127632, 0.353551, 0, -9.81],
          [-1.24324, -16.4517, 11.8219, 0],
          [0, -7.87678, -21.3803, 0],
          [0, 0, 1, 0]])
B_long = np.array([[0.02104349],
          [-28.0431],
          [-284.0878],
          [0]])
C_long = np.array([[0, 0, 0, 1]])
D_long = np.zeros((np.shape(C_long)[0], np.shape(B_long)[1]))

p_long = 1000
Q_long = p_long*np.matmul(np.transpose(C_long), C_long)
R_long = np.eye(1)

K_long, _, _ = control.lqr(A_long, B_long, Q_long, R_long)
Nbar_long = precom(A_long, B_long, C_long, K_long)

# Lateral
A_lat = np.array([[-0.618709, -0.251147, -15.2152, 9.811],
         [-1.41853, -20.1165, 2.34814, 0],
         [2.18038, -3.20227, -1.9908, 0],
         [0, 1, 0, 0]])
B_lat = np.array([[2.673794, 6.571011],
         [160.6404, 1.970345],
         [18.60043, -29.287260],
         [0, 0]])
C_lat = np.array([[0, 0, 0, 1],
                  [0, 0, 1, 0]])
D_lat = np.zeros((np.shape(C_lat)[0], np.shape(B_lat)[1]))

p_lat = 200
Q_lat = p_lat*np.matmul(np.transpose(C_lat), C_lat)
R_lat = np.eye(2)

K_lat, _, _ = control.lqr(A_lat, B_lat, Q_lat, R_lat)
Nbar_lat = precom(A_lat, B_lat, C_lat, K_lat)

# Carrot chasing algorithm parameters
Kpsi = 1
Ktheta = 0.2






envs = gymnasium.make("PyFlyt/Fixedwing-DubinsPath-v0", render_mode="human", angle_representation='euler', flight_dome_size=200, turning_radius=40)

print("Running")
terminated = False
truncated = False
rewards = 0
stepcount = 0
aviary_options = {"cameraTargetPosition":[-10, -10, 30]}
next_obs, _ =envs.reset(aviary_options=aviary_options)
timenow = time.time()
imgs_array = []
while not (terminated or truncated):
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
    throttles = throttle

    cmds = np.array([-pitchs, rolls, yaws, throttles])

    print([obs[7]], obs[3])
    cmds = np.clip(cmds, [-1, -1, -1, 0], 1)
    

    next_obs, reward, terminated, truncated, infos = envs.step(cmds)

print(infos)
imgs = [Image.fromarray(img) for img in imgs_array]
imgs[0].save("FWTarget.gif", save_all=True, append_images=imgs[1:], duration=100/3, loop=0)
print("Total time: {}".format(time.time()-timenow))
