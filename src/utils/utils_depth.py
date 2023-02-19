
import torch

from scipy.spatial.transform import Rotation as R
import numpy as np




def depth_estimation(i ,flow ,corners ,t_vels ,r_vels ,focal_length ,img_shape ,trial_type,K):
    """
    This function estimates the depth of an aruco marker.

    Parameters:
        img (numpy.array): Input image.
        flow (numpy.array): Estimated flow of the projected marker in image plane.
        corners (numpy.array): The estimated positions of the aruco markers in image frame.
        velocity_t (numpy.array): Translational velocity of the camera in world frame.
        velocity_r (numpy.array): Rotational velocity of the camera in world frame .
        focal_length (int): Focal length of the camera
        img_shape (numpy.shape): Shape of the input image

      Returns:
        numpy.array: The depth of each pixel in the aruco marker.
    """
    
    tvel_c = ((t_vels[i] +t_vels[i+1] ) /2).reshape(3 ,1)  # *fps #m/s
    rvel_c = ((r_vels[i] +r_vels[i+1] ) /2).reshape(3 ,1)  # *fps #rad/s

    cx_opencv = (int(corners[0][0] ) +int(corners[1][0] ) +int(corners[2][0] ) +int(corners[3][0])) / 4
    cy_opencv = (int(corners[0][1] ) +int(corners[1][1] ) +int(corners[2][1] ) +int(corners[3][1])) / 4

    cx= cx_opencv -  K[0][2] 
    cy = cy_opencv -  K[1][2]

    jacobian_trans = np.array([[-focal_length ,0 ,cx] ,[0 ,-focal_length ,cy]])
    jacobian_rot = np.array([[cx *cy /focal_length, -(focal_length**2+ cx**2)/focal_length, cy],
                             [(focal_length ** 2 + cy ** 2) / focal_length, -cx * cy / focal_length, -cx]])

    rot = np.matmul(jacobian_rot, rvel_c)
    trans = np.matmul(jacobian_trans, tvel_c)

    if trial_type == "translational":
        vel_filter = flow
    else:
        vel_filter = - rot  

    trans_inv = np.linalg.pinv(trans)
    d = (trans_inv @ vel_filter)
    Z = 1 / d
    return Z

def iqr(thresh, a):
    """
    Applies the interquartile range (IQR) method to remove outliers from a numpy array.

    Args:
        thresh (float): The threshold value for the IQR method. Typically a value of 1.5 is used.
        a (numpy.ndarray): The input array to remove outliers from.

    Returns:
        numpy.ndarray: The input array with outliers removed based on the IQR method.
    """
    a = np.array(a).flatten()
    Q1 = np.quantile(a, 0.25)
    Q3 = np.quantile(a, 0.75)
    IQR = Q3 - Q1
    a = a[~((a < (Q1 - thresh * IQR)) | (a > (Q3 + thresh * IQR)))]
    return a

