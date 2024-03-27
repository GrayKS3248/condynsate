"""
This modules provides utility functions that are used by the 
visualizer and simulator modules.
"""


###############################################################################
#DEPENDENCIES
###############################################################################
import numpy as np
from pathlib import Path


###############################################################################
#PUBLIC FUNCTIONS
###############################################################################
def get_rot_from_2_vecs(vec1,
                        vec2):
    """
    Calculates a quaternion representing the transformation of the
    the vec1 vector to the vec2 vector.

    Parameters
    ----------
    vec1 : array-like, shape(3,)
        The initial vector.
    vec2 : array-like, shape(3,)
        The vector to which the transformation is calculated.
        
    Returns
    -------
    xyzw_rot : array-like, shape(4,)
        The JPL quaternion (xyzw) the takes the vec1 vector to the
        vec2 vector (without scaling). vec2 = xyzw_rot*vec1

    """
    # Convert to numpy array
    arr1 = np.array(vec1)
    arr2 = np.array(vec2)
    
    # Calculate the norm of vec
    mag1 = np.linalg.norm(arr1)
    mag2 = np.linalg.norm(arr2)
    
    # If either magnitude is 0, no rotation can be found.
    if mag1==0. or mag2==0.:
        xyzw_rot = [0., 0., 0., 1.]
        return xyzw_rot
    
    # If the magnitude is not zero, get the direction of vec
    dirn1 = arr1/mag1
    dirn2 = arr2/mag2
    
    # If the vec is exactly 180 degrees away, set the 180 deg quaternion
    if (dirn2==-1*dirn1).all():
        xyzw_rot = [0.5*np.sqrt(2), -0.5*np.sqrt(2), 0., 0.]
        return xyzw_rot
    
    # If the vec is some other relative orientation, calculate it
    q_xyz = np.cross(dirn1, dirn2)
    q_w = 1.0 + np.dot(dirn1, dirn2)
    xyzw_rot = np.append(q_xyz, q_w).tolist()
    xyzw_rot = xyzw_rot/np.linalg.norm(xyzw_rot)
    return xyzw_rot


def xyzw_to_wxyz(xyzw_quaternion):
    """
    Converts a JPL quaternion (xyzw) to a Hamilton quaternion (wxyz)

    Parameters
    ----------
    xyzw_quaternion : array-like, size (4,)
        A JPL quaternion to be converted.

    Returns
    -------
    wxyz_quaternion : array-like, size (4,)
        The Hamilton representation of the input JPL quaterion

    """
    wxyz_quaternion = np.roll(xyzw_quaternion, 1)
    return wxyz_quaternion


def wxyz_to_xyzw(wxyz_quaternion):
    """
    Converts a Hamilton quaternion (wxyz) to a JPL quaternion (xyzw)

    Parameters
    ----------
    wxyz_quaternion : array-like, size (4,)
        A Hamilton quaternion to be converted.

    Returns
    -------
    xyzw_quaternion : array-like, size (4,)
        The JPL representation of the input Hamilton quaterion.

    """
    xyzw_quaternion = np.roll(wxyz_quaternion, -1)
    return xyzw_quaternion


def xyzw_quat_mult(q1, q2):
    """
    Gets the resultant JPL quaternion (xyzw) that arises from first 
    applying the q1 (xyzw) rotation then applying the q2 (xyzw) rotation.

    Parameters
    ----------
    q1 : array-like, shape(4,)
        The first xyzw quaternion applied.
    q2 : array-like, shape(4,)
        The second xyzw quaternion applied.

    Returns
    -------
    q3_xyzw : array-like, shape(4,)
        The resultant transformation from first doing the q1 transformation
        then doing the q2 transformation. Given in JPL form (xyzw).

    """
    q1_wxyz = xyzw_to_wxyz(q1)
    q2_wxyz = xyzw_to_wxyz(q2)
    q3_wxyz = wxyz_quat_mult(q1_wxyz, q2_wxyz)
    q3_xyzw = wxyz_to_xyzw(q3_wxyz)
    return q3_xyzw


def wxyz_quat_mult(q1, q2):
    """
    Gets the resultant Hamilton quaternion (wxyz) that arises from first 
    applying the q1 (wxyz) rotation then applying the q2 (wxyz) rotation.

    Parameters
    ----------
    q1 : array-like, shape(4,)
        The first wxyz quaternion applied.
    q2 : array-like, shape(4,)
        The second wxyz quaternion applied.

    Returns
    -------
    q3_wxyz : array-like, shape(4,)
        The resultant transformation from first doing the q1 transformation
        then doing the q2 transformation. Given in Hamilton form (wxyz).

    """
    a1 = q1[0]
    b1 = q1[1]
    c1 = q1[2]
    d1 = q1[3]
    a2 = q2[0]
    b2 = q2[1]
    c2 = q2[2]
    d2 = q2[3]
    q3w = a2*a1 - b2*b1 - c2*c1 - d2*d1
    q3x = a2*b1 + b2*a1 + c2*d1 - d2*c1
    q3y = a2*c1 - b2*d1 + c2*a1 + d2*b1
    q3z = a2*d1 + b2*c1 - c2*b1 + d2*a1
    q3_wxyz = [q3w, q3x, q3y, q3z]
    return q3_wxyz


def wxyz_from_euler(roll, pitch, yaw):
    """
    Converts Euler angles to a Hamilton quaternion (wxyz)

    Parameters
    ----------
    roll : float
        The roll angle in rad.
    pitch : float
        The pitch angle in rad.
    yaw : float
        The yaw angle in rad.

    Returns
    -------
    wxyz_quaternion : array-like, shape (4,)
        The Hamilton quaternion representation of the input Euler angles.

    """
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    wxyz_quaternion = [w, x, y, z]
    return wxyz_quaternion


def R_ofB_inW_from_euler(roll, pitch, yaw):
    """
    Gets the orientation of a body in world coordinates from the euler angles
    of the body.

    Parameters
    ----------
    roll : float
        The roll angle in rad.
    pitch : float
        The pitch angle in rad.
    yaw : float
        The yaw angle in rad.

    Returns
    -------
    R_ofC_inW : array-like, shape(3,3)
        The orientation of the body in world coordinates. This rotation matrix
        takes vectors in body coordinates to world coordinates .

    """
    cx = np.cos(roll)
    sx = np.sin(roll)
    cy = np.cos(pitch)
    sy = np.sin(pitch)
    cz = np.cos(yaw)
    sz = np.sin(yaw)
    Rx = np.array([[1., 0., 0.],
                   [0., cx, -sx],
                   [0., sx, cx]])
    Ry = np.array([[cy, 0., sy],
                   [0., 1., 0.],
                   [-sy, 0., cy]])
    Rz = np.array([[cz, -sz, 0.],
                   [sz, cz, 0.],
                   [0., 0., 1.]])
    R_ofB_inW = Rz@Ry@Rx
    return R_ofB_inW


def format_path(unformatted_path):
    """
    Converts an unformatted relative path to a formatted Windows path string

    Parameters
    ----------
    unformatted_path : string
        The unformatted relative path.

    Returns
    -------
    formatted_path : string
        The formatted Windows path string.

    """
    formatted_path = str(Path(unformatted_path))
    return formatted_path
    

def format_RGB(unformatted_rgb,
                range_to_255=True):
    """
    Converts unformatted rgb array-like objects to formatted rgb lists

    Parameters
    ----------
    unformatted_rgb : array-like, shape(3,)
        A 3 item array-like that contains the RGB data either in 0-1 or 0-255.
    range_to_255 : bool, optional
        A boolean flag that indicates whether the unformatted_rgb data needs to
        be converted from 0-1 to 0-255. The default is True.

    Returns
    -------
    list_rgb_255_int : list of ints, shape(3,)
        The formatted RGB list.

    """
    rgb = np.array(unformatted_rgb)
    if range_to_255:
        rgb_255 = np.round(rgb*255)
    else:
        rgb_255 = np.round(rgb)
    rgb_255_int = rgb_255.astype(int)
    list_rgb_255_int = rgb_255_int.tolist()
    return list_rgb_255_int


def RAB_to_RBA(R_ofA_inB):
    """
    Takes a rotation cosine matrix of the A frame in B coords to the rotation
    cosine of the B frame in A coords.

    Parameters
    ----------
    R_ofA_inB : valid rotation matrix, shape(3,3)
        The rotation cosine matrix of the A frame in B coords.

    Returns
    -------
    R_ofB_inA : valid rotation matrix, shape(3,3)
        The rotation cosine matrix of the B frame in A coords.

    """
    R_ofB_inA = np.array(R_ofA_inB).T
    return R_ofB_inA
    

def OAB_to_OBA(R_ofA_inB, O_ofA_inB):
    """
    Takes the origin of frame A in B coords to the origin of frame B in A
    coords.

    Parameters
    ----------
    R_ofA_inB : valid rotation matrix, shape(3,3)
        The rotation cosine matrix of the A frame in B coords.
    O_ofA_inB : array, shape(3,)
        The origin of frame A in B coords.

    Returns
    -------
    O_ofB_inA : array, shape(3,)
        The origin of frame B in A coords.

    """
    R_ofB_inA = RAB_to_RBA(R_ofA_inB)
    O_ofB_inA = -R_ofB_inA @ np.array(O_ofA_inB)
    return O_ofB_inA


def vc_inA_toB(R_ofA_inB, vc_inA):
    """
    Based on a relative cosine matrix, takes vecotrs in frame A coords to 
    frame B coords.
    
    Parameters
    ----------
    R_ofA_inB : valid rotation matrix, shape(3,3)
        The rotation cosine matrix of the A frame in B coords.
    vc_inA : array, shape(3,)
        A 3 vector in frame A.

    Returns
    -------
    vc_inB : array, shape(3,)
        The same 3 vector in frame B coords.

    """
    vc_inB = np.array(R_ofA_inB) @ np.array(vc_inA)
    return vc_inB
    

def pt_inA_toB(R_ofA_inB, O_ofA_inB, pt_inA):
    """
    Based on a relative cosine matrix and origin vector, takes a point in frame
    A coords to frame B coords.

    Parameters
    ----------
    R_ofA_inB : valid rotation matrix, shape(3,3)
        The rotation cosine matrix of the A frame in B coords.
    O_ofA_inB : array, shape(3,)
        The origin of frame A in B coords.
    pt_inA : array, shape(3,)
        A 3D point in frame A.

    Returns
    -------
    pt_inB : array, shape(3,)
        The same 3D point in B coords.

    """
    pt_inB = np.array(R_ofA_inB) @ np.array(pt_inA) + np.array(O_ofA_inB)
    return pt_inB
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    