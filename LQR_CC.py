import numpy as np
import gymnasium
from PyFlyt.core import Aviary, loadOBJ, obj_collision, obj_visual
import argparse
import os
import random
import time
from distutils.util import strtobool

import gymnasium as gym
import PyFlyt.gym_envs
from PIL import Image

from pyPS4Controller.controller import Controller
from threading import Thread, Event

import control

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)


    def on_R3_down(self, value):
        global cmds

        value = value / 32767

        cmds[0] = value
        return value

    def on_R3_up(self, value):
        global cmds

        value = value / 32767

        cmds[0] = value
        return value

    def on_R3_left(self, value):
        global cmds

        value = value / 32767

        cmds[1] = value
        return value

    def on_R3_right(self, value):
        global cmds

        value = value / 32767

        cmds[1] = value
        return value

    def on_L3_left(self, value):
        global cmds

        value = value / 32767

        cmds[2] = value
        return value

    def on_L3_right(self, value):
        global cmds

        value = value / 32767

        cmds[2] = value
        return value

    def on_R2_press(self, value):
        global cmds

        value = value / 32767

        cmds[3] = value
        return value

def precom(A, B, C, K):
    """Implementation of precompensation block to scale reference for LQR reference tracking"""
    closed_loop = A - np.matmul(B, K)
    inv = np.linalg.inv(closed_loop)
    mul = np.matmul(inv, B)
    denom = np.matmul(C, mul)
    Nbar = np.divide(-1, denom)
    return Nbar 

def getCtrl(s, K, Nbar, ref):
    """Calculates control effort given a reference and state vector"""
    s_gained = np.matmul(K, s) # Kx
 
    ref_comp = np.matmul(Nbar, ref) # Nr

    ctrl = ref_comp - s_gained # u

    return ctrl

def getRefs(carrot_pos, UAV_pos, UAV_ang, Kpsi, Ktheta):
    """Calculates the controller reference based on the position of the carrot and gains
        Longitudinal reference: 
        Lateral reference: heading error to carrot
    """
    # Get vector from UAV to carrot
    vector2carrot = carrot_pos - UAV_pos

    # UAV euler angles
    UAV_psi = UAV_ang[2]
    UAV_theta = UAV_ang[0]

    # Calculate carrot heading vector and UAV heading vector in world frame
    carrot_heading_vector = [vector2carrot[1], -vector2carrot[0]]
    UAV_heading_vector = [np.cos(UAV_psi), np.sin(UAV_psi)]

    # Calculate flight path angle from UAV to carrot
    XY_vect = np.linalg.norm([vector2carrot[0], vector2carrot[1]])
    carrot_flightpath = np.arctan(vector2carrot[2]/XY_vect)

    # Calculate errors
    # Handle +pi/-pi problem
    dot = np.dot(carrot_heading_vector, UAV_heading_vector)
    norm = np.linalg.norm(carrot_heading_vector) * np.linalg.norm(UAV_heading_vector)

    heading_err_val = np.arccos(np.clip(dot/norm, -1, 1))
    heading_sign = np.sign(np.cross(carrot_heading_vector, UAV_heading_vector))
    heading_err = np.abs(heading_err_val) * heading_sign

    flightpath_err = carrot_flightpath - UAV_theta

    # Multiply gains
    lat_ref = Kpsi * heading_err
    long_ref = Ktheta * flightpath_err

    # Saturate limits
    lat_ref = np.clip(lat_ref, -1.4, 1.4)
    long_ref = np.clip(long_ref, -1.4, 1.4)

    return lat_ref, long_ref


def readDS4():
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()
    
# t = Thread(target=readDS4, args=())
# t.start()

roll = 0
pitch = 0
yaw = 0
throttle = 0.6

cmds = [roll, pitch, yaw, throttle]

# Longitudinal 
A_long = np.array([[-0.0127632, 0.353551, 0, -9.81],
          [-1.24324, -16.4517, 11.8219, 0],
          [0, -7.87678, -21.3803, 0],
          [0, 0, 1, 0]])*4
B_long = np.array([[0.02104349],
          [-28.0431],
          [-284.0878],
          [0]])*4
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
         [0, 1, 0, 0]])*4
B_lat = np.array([[2.673794, 6.571011],
         [160.6404, 1.970345],
         [18.60043, -29.287260],
         [0, 0]])*4
C_lat = np.array([[0, 0, 0, 1],
         [0, 0, 1, 0]])
D_lat = np.zeros((np.shape(C_lat)[0], np.shape(B_lat)[1]))

p_lat = 2500
Q_lat = p_lat*np.matmul(np.transpose(C_lat), C_lat)
R_lat = np.eye(2)

K_lat, _, _ = control.lqr(A_lat, B_lat, Q_lat, R_lat)
Nbar_lat = precom(A_lat, B_lat, C_lat, K_lat)

# Carrot chasing algorithm parameters
Kpsi = 0.5
Ktheta = 10






envs = gymnasium.make("PyFlyt/Fixedwing-DubinsPath-v0", render_mode="human", angle_representation='euler')

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
    ref_lat, ref_long = getRefs(carrot_pos, obs[9:12], obs[3:6], Kpsi, Ktheta)

    ref_lat = [ref_lat, 0] # phi and r
    ref_long = [ref_long] # theta

    # Get control output from lqr control law
    ctrl_lat = getCtrl(s_lat, K_lat, Nbar_lat, ref_lat)
    ctrl_long = getCtrl(s_long, K_long, Nbar_long, ref_long)

    roll = ctrl_lat[0][0]
    yaw = ctrl_lat[1][0]
    pitch = ctrl_long[0][0]

    cmds = np.array([-pitch, roll, yaw, throttle])

    print([obs[7]])
    cmds = np.clip(cmds, -1, 1)
    

    next_obs, reward, terminated, truncated, infos = envs.step(cmds)
    rewards += reward
    stepcount += 1
    # print("Reward: {}".format(reward))

# print("Observation: {}".format(next_obs))
imgs = [Image.fromarray(img) for img in imgs_array]
imgs[0].save("FWTarget.gif", save_all=True, append_images=imgs[1:], duration=100/3, loop=0)
print("Total time: {}".format(time.time()-timenow))
