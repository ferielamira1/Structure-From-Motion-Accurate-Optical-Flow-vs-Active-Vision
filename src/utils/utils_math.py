
import torch

from scipy.spatial.transform import Rotation as R
import numpy as np



def depth_estimation(i ,flow ,corners ,t_vels ,r_vels ,focal_length ,img_shape ,trial_type):
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
    # Translational and rotational transformation from end effector frame to camera frame
    tvel_c = ((t_vels[i] +t_vels[i+1] ) /2).reshape(3 ,1)  # *fps #m/s
    rvel_c = ((r_vels[i] +r_vels[i+1] ) /2).reshape(3 ,1)  # *fps #rad/s

    cx_opencv = (int(corners[0][0] ) +int(corners[1][0] ) +int(corners[2][0] ) +int(corners[3][0])) / 4
    cy_opencv = (int(corners[0][1] ) +int(corners[1][1] ) +int(corners[2][1] ) +int(corners[3][1])) / 4
    cx= cx_opencv - (img_shape[1 ] /2)
    cy = (cy_opencv - (img_shape[0 ] /2))

    jacobian_trans = np.array([[-focal_length ,0 ,cx] ,[0 ,-focal_length ,cy]])
    jacobian_rot = np.array([[cx *cy /focal_length, -(focal_length** 2+ cx**2) /focal_length, cy],
                             [(focal_length ** 2 + cy ** 2) / focal_length, -cx * cy / focal_length, -cx]])

    rot = np.matmul(jacobian_rot, rvel_c)
    trans = np.matmul(jacobian_trans, tvel_c)
    if trial_type == "translational":
        vel_filter = mflow
        vel_filter_mixed = mflow - rot
    else:
        vel_filter = - rot  # take this out when using translation
        vel_filter_mixed = mflow - rot

    trans_inv = np.linalg.pinv(trans)

    d = (trans_inv @ vel_filter)
    d_mixed = (trans_inv @ vel_filter_mixed)

    Z = 1 / d
    Z_mixed = 1 / d_mixed
    '''if trial_type=="translational":
       Z1 =(-focal_length  * velocity_t[0] )/flow[0] +(cx *velocity_t[2])/flow[0]

       Z2=(-focal_length  * velocity_t[1])/flow[1] + (cy *velocity_t[2])/flow[1]

    if trial_type =="gazelock":
      Z1= velocity_t[1]/velocity_r[0]
      Z2= velocity_t[0]/velocity_r[1]

      Z= (abs(Z1)+abs(Z2))/2
    '''
    return Z, Z_mixed


def transformation_matrix(tf_base_camera):
    """Calculates the 4x4 transformation matrix from a ROS TransformStamped message

       Args:
       - tf_base_camera: A ROS TransformStamped message representing the transformation between two coordinate frames.

       Returns:
       - tf_matrix: A 4x4 transformation matrix representing the rotation and translation between the two frames.

       """
    tf_quat_base_camera = tf_base_camera.transform.rotation
    r = R.from_quat([tf_quat_base_camera.x, tf_quat_base_camera.y, tf_quat_base_camera.z, tf_quat_base_camera.w])
    tf_rot_matrix = r.as_matrix()

    # Create 4x4 transformation matrix
    tf_matrix = np.zeros((4, 4))
    tf_matrix[:3, :3] = tf_rot_matrix
    tf_matrix[0, 3] = tf_base_camera.transform.translation.x
    tf_matrix[1, 3] = tf_base_camera.transform.translation.y
    tf_matrix[2, 3] = tf_base_camera.transform.translation.z
    tf_matrix[3, 3] = 1
    return tf_matrix




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

