import numpy as np




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

    flightpath_err = carrot_flightpath - UAV_theta

    # Multiply gains
    lat_ref = Kpsi * heading_err
    long_ref = Ktheta * flightpath_err

    # Saturate limits
    lat_ref = np.clip(lat_ref, -latlim, latlim)
    long_ref = np.clip(long_ref, -longlim, longlim)

    return lat_ref, long_ref
