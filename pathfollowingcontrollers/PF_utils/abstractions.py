import numpy as np
import matplotlib.pyplot as plt
import os

from os import path
from matplotlib.ticker import FormatStrFormatter
from pyPS4Controller.controller import Controller
from threading import Thread, Event

# Create directory for plots if dont already exist
if not os.path.exists('plots'):
    os.mkdir('plots')

DS4cmds = [0, 0, 0, 0]

def plotZP(poles, zeros, title):

    fig = plt.figure()
    ax = fig.add_subplot()  

    ax.set_xlabel('Real')
    ax.set_ylabel('Imaginary')
    ax.axhline(y=0, color='black', lw=1)
    ax.axvline(x=0, color='black', lw=1)

    if len(poles) > 0:
        ax.scatter(poles.real, poles.imag, s=50, marker='x',
                    facecolors='k', label='Poles')
    if len(zeros) > 0:
        ax.scatter(zeros.real, zeros.imag, s=50, marker='o',
                    facecolors='none', edgecolors='k', label='Zeros')
    
    plt.legend(loc='upper center')
    plt.title(title)
    plt.savefig("plots/"+title)


def plotXY(desired, actual, title):
    """ Create a plot showing the UAV's desired vs actual trajectory in the XY plane.
        desired: n by 3 array, n = number of time steps
        actual: n by 3 array, n = number of time steps
        title: string
    """

    # Extract X and Y coordinates from path array
    X_D = [x[0] for x in desired]
    Y_D = [x[1] for x in desired]

    # Extract X and Y coordinates from actual trajectory array
    X_A = [x[0] for x in actual]
    Y_A = [x[1] for x in actual]

    plt.figure()
    plt.scatter(X_D[0], Y_D[0], label="Start", c=[[0, 1, 0]])
    plt.scatter(X_D[-1], Y_D[-1], label="End", c=[[1, 0, 0]])
    plt.plot(X_D, Y_D, label="Desired")
    plt.plot(X_A, Y_A, label="Actual")
    plt.xlabel("X(m)")
    plt.ylabel("Y(m)")
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.title(title)
    plt.savefig("plots/"+title)


def plotZ(desired, actual, title):
    """ Create a plot showing the UAV's desired vs actual trajectory in the Z axis.
        desired: n by 3 array, n = number of time steps
        actual: n by 3 array, n = number of time steps
        title: string
    """

    # Extract Z and X coordinates from path array
    X_D = [x[0] for x in desired]
    Z_D = [x[2] for x in desired]

    # Extract Z and X coordinates from actual trajectory array
    X_A = [x[0] for x in actual]
    Z_A = [x[2] for x in actual]

    plt.figure()
    plt.scatter(X_D[0], Z_D[0], label="Start", c=[[0, 1, 0]])
    plt.scatter(X_D[-1], Z_D[-1], label="End", c=[[1, 0, 0]])
    plt.plot(X_D, Z_D, label="Desired")
    plt.plot(X_A, Z_A, label="Actual")
    plt.ylim(min(Z_D)-5, max(Z_D)+5) # +- max and min in the Z axis, to prevent drawn out plot
    plt.xlabel("X(m)")
    plt.ylabel("Z(m)")
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.title(title)
    plt.savefig("plots/"+title)


def plot3D(desired, actual, title):
    """ Create a plot showing the UAV's desired vs actual trajectory in the 3-Dimensions.
        desired: n by 3 array, n = number of time steps
        actual: n by 3 array, n = number of time steps
        title: string
    """

    # Extract coordinates from path array
    X_D = [x[0] for x in desired]
    Y_D = [x[1] for x in desired]
    Z_D = [x[2] for x in desired]

    # Extract coordinates from actual trajectory array
    X_A = [x[0] for x in actual]
    Y_A = [x[1] for x in actual]
    Z_A = [x[2] for x in actual]

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')    
    ax.scatter(X_D[0], Y_D[0], Z_D[0], label="Start", c=[[0, 1, 0]])
    ax.scatter(X_D[-1], Y_D[-1], Z_D[-1], label="End", c=[[1, 0, 0]])
    plt.plot(X_D, Y_D, Z_D, label="Desired")
    plt.plot(X_A, Y_A, Z_A, label="Actual")
    ax.set_zlim3d(min(Z_D)-5, max(Z_D)+5) # +- max and min in the Z axis, to prevent drawn out plot
    ax.set_xlabel("X(m)")
    ax.set_ylabel("Y(m)")
    ax.set_zlabel("Z(m)")
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.title(title)
    plt.savefig("plots/"+title)


def plotCtrlTraces(ctrlArray, t, title):
    """Create a plot containing 4 subplots for each control trace. [Pitch, Roll, Yaw, Throttle]
        ctrlArray: n by 4 array, n = number of time steps
        t: 1 by n array, in seconds
        title: string
    """

    pitchCtrl = [x[0] for x in ctrlArray] 
    rollCtrl = [x[1] for x in ctrlArray] 
    yawCtrl = [x[2] for x in ctrlArray] 
    throttleCtrl = [x[3] for x in ctrlArray] 

    fig, axs = plt.subplots(4, 1)
    for axis in axs:
        axis.yaxis.set_major_formatter(FormatStrFormatter('%.1f'))

    axs[0].plot(t, pitchCtrl)
    axs[0].set_xlim(0, t[-1])
    axs[0].set_ylim(-1, 1)
    axs[0].set_ylabel('Elevator')
    axs[0].grid(True)

    axs[1].plot(t, rollCtrl)
    axs[1].set_xlim(0, t[-1])
    axs[1].set_ylim(-1, 1)
    axs[1].set_ylabel('Aileron')
    axs[1].grid(True)

    axs[2].plot(t, yawCtrl)
    axs[2].set_xlim(0, t[-1])
    axs[2].set_ylim(-1, 1)
    axs[2].set_ylabel('Rudder')
    axs[2].grid(True)

    axs[3].plot(t, throttleCtrl)
    axs[3].set_xlim(0, t[-1])
    axs[3].set_xlabel('Time(s)')
    axs[3].set_ylim(0, 1)
    axs[3].set_ylabel('Throttle')
    axs[3].grid(True)

    fig.tight_layout()
    fig.suptitle("Control traces", fontsize="15")
    fig.subplots_adjust(top=0.92)
    plt.savefig("plots/"+title)


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


def getCCRefs(carrot_pos, UAV_pos, UAV_ang, UAV_vel, Kpsi, Ktheta, latlim, longlim):
    """Calculates the controller reference based on the position of the carrot and gains
        Longitudinal reference: 
        Lateral reference: heading error to carrot
    """

    # Get UAV speed 
    UAV_speed = np.linalg.norm(UAV_vel)

    # Get vector from UAV to carrot
    vector2carrot = carrot_pos - UAV_pos

    # UAV euler angles
    UAV_psi = UAV_ang[2]

    # Calculate carrot heading vector and UAV heading vector in world frame
    carrot_heading_vector = [vector2carrot[1], -vector2carrot[0]]
    UAV_heading_vector = [np.cos(UAV_psi), np.sin(UAV_psi)]

    # Calculate errors
    # Handle +pi/-pi problem
    dot = np.dot(carrot_heading_vector, UAV_heading_vector)
    norm = np.linalg.norm(carrot_heading_vector) * np.linalg.norm(UAV_heading_vector)

    heading_err_val = np.arccos(np.clip(dot/norm, -1, 1))
    heading_sign = np.sign(np.cross(carrot_heading_vector, UAV_heading_vector))
    heading_err = np.abs(heading_err_val) * heading_sign
    heading_err =  heading_err * Kpsi # Apply gain

    # Apply coordinated turn kinematic model to convert heading error (psi_dot) to phi
    phi = np.arctan(heading_err * UAV_speed / 9.81)
    lat_ref = phi

    # Get altitude error
    altitude_err = carrot_pos[2] - UAV_pos[2]
    long_ref = Ktheta * altitude_err

    # Saturate limits
    lat_ref = np.clip(lat_ref, -latlim, latlim)
    long_ref = np.clip(long_ref, -longlim, longlim)

    return lat_ref, long_ref


def getNLGLRefs(carrot_pos, UAV_pos, UAV_ang, UAV_vel, Kpsi, Ktheta, L1, latlim, longlim):
    """Calculates the controller reference based on NLGL
        Longitudinal reference: 
        Lateral reference: heading error to carrot
    """
    # Get UAV speed 
    UAV_speed = np.linalg.norm(UAV_vel)

    # Get vector from UAV to carrot
    vector2carrot = carrot_pos - UAV_pos

    # UAV euler angles
    UAV_psi = UAV_ang[2]

    # Calculate carrot heading vector and UAV heading vector in world frame
    carrot_heading_vector = [vector2carrot[1], -vector2carrot[0]]
    UAV_heading_vector = [np.cos(UAV_psi), np.sin(UAV_psi)]

    # Calculate errors
    # Handle +pi/-pi problem
    dot = np.dot(carrot_heading_vector, UAV_heading_vector)
    norm = np.linalg.norm(carrot_heading_vector) * np.linalg.norm(UAV_heading_vector)

    heading_err_val = np.arccos(np.clip(dot/norm, -1, 1))
    heading_sign = np.sign(np.cross(carrot_heading_vector, UAV_heading_vector))
    eta = np.abs(heading_err_val) * heading_sign # heading error

    # Get turn radius from NLGL
    R = 0 if eta == 0 else (L1 / (2 * np.sin(eta))) # Handle division by zero

    # Get NLGL lateral acceleration command 
    acc_cmd = 2 * np.square(UAV_speed) / L1 * np.sin(eta)

    # Apply coordinated turn kinematics to get phi
    phi = np.arctan((np.sqrt(acc_cmd * R) * eta) / 9.81)

    lat_ref = Kpsi * phi

    # Get altitude error
    altitude_err = carrot_pos[2] - UAV_pos[2]   
    long_ref = Ktheta * altitude_err

    # Saturate limits
    lat_ref = np.clip(lat_ref, -latlim, latlim)
    long_ref = np.clip(long_ref, -longlim, longlim)

    return lat_ref, long_ref


class GameController(Controller):
    """ 
        Gets DS4 controller inputs for manual flying
    """
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
    controller = GameController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    controller.listen()


def readDS4():
    global DS4cmds
    t = Thread(target=DS4Targ, args=())
    t.start()
    return DS4cmds