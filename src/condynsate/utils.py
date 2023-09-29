"""
utils
"""


###############################################################################
#DEPENDENCIES
###############################################################################
import numpy as np
from pathlib import Path


###############################################################################
#PUBLIC FUNCTIONS
###############################################################################
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
