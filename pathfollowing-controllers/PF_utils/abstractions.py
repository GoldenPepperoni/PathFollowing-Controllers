import numpy as np

from pyPS4Controller.controller import Controller
from threading import Thread, Event

DS4cmds = [0, 0, 0, 0]

def precom(A, B, C, K):
    """Implementation of precompensation block to scale reference for LQR reference tracking"""
    closed_loop = A - np.matmul(B, K)
    inv = np.linalg.inv(closed_loop)
    mul = np.matmul(inv, B)
    denom = np.matmul(C, mul)
    Nbar = np.divide(-1, denom)
    print(K)

    return Nbar 

def getCtrl(s, K, Nbar, ref):
    """Calculates control effort given a reference and state vector"""
    s_gained = np.matmul(K, s) # Kx
 
    ref_comp = np.matmul(Nbar, ref) # Nr

    ctrl = ref_comp - s_gained # u

    return ctrl

def getRefs(carrot_pos, UAV_pos, UAV_ang, Kpsi, Ktheta, latlim, longlim):
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

    # flightpath_err = carrot_flightpath - UAV_theta
    flightpath_err = carrot_pos[2] - UAV_pos[2]

    # Multiply gains
    lat_ref = Kpsi * heading_err
    long_ref = Ktheta * flightpath_err

    # Saturate limits
    lat_ref = np.clip(lat_ref, -latlim, latlim)
    long_ref = np.clip(long_ref, -longlim, longlim)

    return lat_ref, long_ref


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)


    def on_R3_down(self, value):
        global DS4cmds

        value = value / 32767

        DS4cmds[0] = value
        return value

    def on_R3_up(self, value):
        global DS4cmds

        value = value / 32767

        DS4cmds[0] = value
        return value

    def on_R3_left(self, value):
        global DS4cmds

        value = value / 32767

        DS4cmds[1] = value
        return value

    def on_R3_right(self, value):
        global DS4cmds

        value = value / 32767

        DS4cmds[1] = value
        return value

    def on_L3_left(self, value):
        global DS4cmds

        value = value / 32767

        DS4cmds[2] = value
        return value

    def on_L3_right(self, value):
        global DS4cmds

        value = value / 32767

        DS4cmds[2] = value
        return value

    def on_R2_press(self, value):
        global DS4cmds

        value = value / 32767

        DS4cmds[3] = value
        return value


def DS4Targ():
    controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()

def readDS4():
    global DS4cmds
    t = Thread(target=DS4Targ, args=())
    t.start()
    return DS4cmds