import os
import rospy
import rosbag
import tf2_ros as tf2
import numpy as np
from rosbag.bag import Bag
import cv2
from cv_bridge import CvBridge

def find_match(pose_stime, frame_infos, frames):
    frame_stime = rospy.Time(secs=frame_infos[0]["header"]["stamp"]["secs"],
                             nsecs=frame_infos[0]["header"]["stamp"]["nsecs"])

    while frame_stime.to_nsec() < pose_stime.to_nsec():
        frame_infos.pop(0)
        frames.pop(0)
        frame_stime = rospy.Time(secs=frame_infos[0]["header"]["stamp"]["secs"],
                                 nsecs=frame_infos[0]["header"]["stamp"]["nsecs"])
    return frame_infos, frames



def rosbag_to_vel(rosbagf, tf_topics, pose_topic):
    """
    Extracts end effector position and rotation from a ROS bag file containing messages of type `geometry_msgs/PoseStamped`
    and `geometry_msgs/TransformStamped`. The end effector position is transformed into the camera optical frame.

    Args:
        rosbagf (str): The path to the ROS bag file.
        tf_topics (str): The topic name containing the transformation data of type `geometry_msgs/TransformStamped`.
        pose_topic (str): The topic name containing the end effector position of type `geometry_msgs/PoseStamped`.

    Returns:
        A tuple containing:
        - t_vels (List[np.ndarray]): A list of position velocities in the camera optical frame.
        - r_vels (List[np.ndarray]): A list of rotation velocities in the camera optical frame.
        - stime (rospy.Time): The start time of the bag file.
    """
    t_vels = []
    r_vels = []
    first = True
    try:
        rospy.init_node("depth_estimation")
    except:
        print("Already registered")

    try:
        buffer = tf2.Buffer(rospy.Duration(10000))
    except:
        print("Buffer could not be initialized")
    # Open the ROS bag
    bag = rosbag.Bag(rosbagf)
    stime = rospy.Time()

    # Iterate through the messages in the bag
    for topic, msg, t in bag.read_messages(topics=[tf_topics]):
        for msg_tf in msg.transforms:
            if topic == '/tf_static':
                buffer.set_transform_static(msg_tf, "default_authority")
            else:
                buffer.set_transform(msg_tf, "default_authority")

    for topic, msg, t in bag.read_messages(topics=[pose_topic]):
        try:
            # last commandd end effector velocity in base frame
            O_dp_EE_c = np.array(msg.O_dP_EE_c)  # .reshape((4,4)).T

            # end effector pose in base frame
            tf_base_EE = buffer.lookup_transform('panda_EE', 'panda_link0', msg.header.stamp)
            tf_mbase_EE = transformation_matrix(tf_base_EE)  # same as: O_T_EE= np.array(msg.O_T_EE).reshape((4,4)).T
            tf_camera_EE = buffer.lookup_transform('camera_color_optical_frame', 'panda_EE', msg.header.stamp)
            tf_mcamera_EE = transformation_matrix(tf_camera_EE)
            p_camera = (tf_mcamera_EE[:3, :3] @ (tf_mbase_EE[:3, :3] @ O_dp_EE_c[:3].reshape(3, 1)))
            rotvec_camera = (tf_mcamera_EE[:3, :3] @ (tf_mbase_EE[:3, :3] @ O_dp_EE_c[3:].reshape(3, 1)))
            if first == True:
                stime = msg.header.stamp
                first = False

            t_vels.append(p_camera)
            r_vels.append(rotvec_camera)

        except Exception as e:
            print(e)

    buffer.clear()
    bag.close()
    return t_vels, r_vels, stime


def rosbag_to_dataset(rosbagf ,rosbag_topic, outputf):
    """
    This function exports the dataset from a rosbag file and returns a  list of dicts containing the video frames and their meta-information (i.e time-stamp).

    Parameters:
        rosbagf (string): Path to the rosbag file.
        rosbag_topic (string): Name of the rosbag topic.
        outputf (string): Path of the output folder

    Returns:
        dict: Dictionary of video frames and meta-information
    """
    bag = Bag(rosbagf ,"r")
    bridge = CvBridge()
    num_frames = 0
    img_dicts =[]
    save_dataset =False
    cv_imgs =[]

    # Create the output folder in case it does not exist
    if not os.path.exists(outputf):
        os.makedirs(outputf)
        print("Directory does not exist, saving dataset ...   ")
        save_dataset =True
    # Iterate through rosbag messages


    for topic, msg, t in bag.read_messages(topics=[rosbag_topic]):
        # converting message to a dictionary
        cv_img = bridge.compressed_imgmsg_to_cv2(msg)
        img_dict = convert_ros_message_to_dictionary(msg, True)
        cv_imgs.append(cv_img)
        img_dicts.append(img_dict)

        # Save the images to the output file in case they do not already exist
        if save_dataset:
            cv2.imwrite(os.path.join(outputf, "frame%06i.jpg" % num_frames), cv_img)
            num_frames+=1

    bag.close()
    return cv_imgs ,img_dicts



def rosbag_to_dict(rosbagf ,topic):
    """
    This function converts the messages of a ros topic to a dictionary and saves them as yaml files in case they don't exist.

    Parameters:
        rosbagf (string): Path to the rosbag file.
        rosbag_topic (string): Name of the rosbag topic.

    Returns:
        dict: Dictionary containing ros messages
    """

    bag = Bag(rosbagf ,"r")
    count = 0
    infos =[]
    for topic, msg, t in bag.read_messages(topics=[topic]):
        info_dict = convert_ros_message_to_dictionary(msg, True)
        infos.append(info_dict)

    bag.close()
    return infos


def get_trials():
    """
        This function defines a list of trials with different parameters for distance, marker size, marker type,
        trial type and ground truth values. There are three categories of trials: translational, rotational, and
        gazelock. Each category contains several trials, each of which is defined as a dictionary with the
        corresponding parameters.

        Returns:
        trials_ld_straight (list): A list of dictionaries that define the parameters for the straight, translational
        trials.
        trials_ld_oblique (list): A list of dictionaries that define the parameters for the oblique, translational
        trials.
        trials_ld_gl_straight (list): A list of dictionaries that define the parameters for the straight,
        gazelock trials.
        """
    # Define the trials
    trials_ld_straight = [
        {"trial": "approxdist_100cm_markersize_270mm_heuristic4gain_0.05_straight_2021-10-26-12-01-55", "gt": 1, "marker_size": 0.27,"marker_type":"straight","trial_type":"translational"},
        {"trial": "approxdist_150cm_markersize_270mm_heuristic4gain_0.05_straight_2021-10-26-12-13-45", "gt": 1.5, "marker_size": 0.27,"marker_type":"straight","trial_type":"translational"},
        {"trial": "approxdist_200cm_markersize_400mm_heuristic4gain_0.05_straight_2021-10-26-12-25-30", "gt": 2, "marker_size": 0.4,"marker_type":"straight","trial_type":"translational"},
        {"trial": "approxdist_270cm_markersize_400mm_heuristic4gain_0.05_straight_2021-10-26-12-36-40", "gt": 2.7, "marker_size": 0.4,"marker_type":"straight","trial_type":"translational"},
        {"trial": "approxdist_330cm_markersize_400mm_heuristic4gain_0.075_straight_2021-10-26-12-45-15", "gt": 3.3, "marker_size": 0.4,"marker_type":"straight","trial_type":"translational"},
        {"trial": "approxdist_400cm_markersize_400mm_heuristic4gain_0.075_straight_2021-10-26-12-56-05", "gt": 4, "marker_size": 0.4,"marker_type":"straight","trial_type":"translational"},
        {"trial": "approxdist_500cm_markersize_400mm_heuristic4gain_0.075_straight_2021-10-26-13-30-30", "gt": 5, "marker_size": 0.4,"marker_type":"straight","trial_type":"translational"}
    ]


    # Name of rosbag files : Long distance, straight , marker oblique

    trials_ld_oblique= [
        { 'trial': 'approxdist_100cm_markersize_270mm_heuristic4gain_0.05_straight_oblique_2021-10-26-12-04-51','gt': 1,'marker_size': 0.27,"marker_type":"oblique","trial_type":"translational"},
        { 'trial': 'approxdist_150cm_markersize_270mm_heuristic4gain_0.05_straight_oblique_2021-10-26-12-16-05','gt': 1.5,'marker_size': 0.27,"marker_type":"oblique","trial_type":"translational"},
        { 'trial': 'approxdist_200cm_markersize_400mm_heuristic4gain_0.05_straight_oblique_2021-10-26-12-27-40',    'gt': 2, 'marker_size': 0.4,"marker_type":"oblique","trial_type":"translational"},
        { 'trial': 'approxdist_270cm_markersize_400mm_heuristic4gain_0.05_straight_oblique_2021-10-26-12-38-15',    'gt': 2.7, 'marker_size': 0.4,"marker_type":"oblique","trial_type":"translational"},
        { 'trial': 'approxdist_330cm_markersize_400mm_heuristic4gain_0.075_straight_oblique_2021-10-26-12-47-10',        'gt': 3.3, 'marker_size': 0.4,"marker_type":"oblique","trial_type":"translational"},
        {'trial':'approxdist_400cm_markersize_400mm_heuristic4gain_0.075_straight_oblique_2021-10-26-12-58-39','gt': 4,'marker_size': 0.4,"marker_type":"oblique","trial_type":"translational"},
        { 'trial': 'approxdist_500cm_markersize_400mm_heuristic4gain_0.075_straight_oblique_2021-10-26-13-34-10','gt': 5,'marker_size': 0.4,"marker_type":"oblique","trial_type":"translational"}]


    # Long distance, gazelock , marker horizontal
    trials_ld_gl_straight = [
        {"trial": "approxdist_100cm_markersize_270mm_heuristic4gain_0.05_gazelock_2021-10-26-11-58-31", "gt": 1, "marker_size": 0.27,"marker_type":"straight","trial_type":"gazelock"},
        {"trial": "approxdist_150cm_markersize_270mm_heuristic4gain_0.05_gazelock_2021-10-26-12-11-20", "gt": 1.5, "marker_size": 0.27,"marker_type":"straight","trial_type":"gazelock"},
        {"trial": "approxdist_200cm_markersize_400mm_heuristic4gain_0.05_gazelock_2021-10-26-12-23-40", "gt": 2, "marker_size": 0.4,"marker_type":"straight","trial_type":"gazelock"},
        {"trial": "approxdist_270cm_markersize_400mm_heuristic4gain_0.05_gazelock_2021-10-26-12-35-15", "gt": 2.7, "marker_size": 0.4,"marker_type":"straight","trial_type":"gazelock"},
        {"trial": "approxdist_330cm_markersize_400mm_heuristic4gain_0.075_gazelock_2021-10-26-12-43-15", "gt": 3.3, "marker_size": 0.4,"marker_type":"straight","trial_type":"gazelock"},
        {"trial": "approxdist_400cm_markersize_400mm_heuristic4gain_0.075_gazelock_2021-10-26-12-54-30", "gt": 4, "marker_size": 0.4,"marker_type":"straight","trial_type":"gazelock"},
        {"trial": "approxdist_500cm_markersize_400mm_heuristic4gain_0.075_gazelock_2021-10-26-13-28-30", "gt": 5, "marker_size": 0.4,"marker_type":"straight","trial_type":"gazelock"}]


    trials_ld_gl_oblique= [
        {"trial": "approxdist_100cm_markersize_270mm_heuristic4gain_0.05_gazelock_oblique_2021-10-26-12-06-10", "gt": 1, "marker_size": 0.27,"marker_type":"oblique","trial_type":"gazelock"},
        {"trial": "approxdist_150cm_markersize_270mm_heuristic4gain_0.05_gazelock_oblique_2021-10-26-12-17-35", "gt": 1.5, "marker_size": 0.27,"marker_type":"oblique","trial_type":"gazelock"},
        {"trial": "approxdist_200cm_markersize_400mm_heuristic4gain_0.05_gazelock_oblique_2021-10-26-12-23-40", "gt": 2, "marker_size": 0.4,"marker_type":"oblique","trial_type":"gazelock"},
        {"trial": "approxdist_270cm_markersize_400mm_heuristic4gain_0.05_gazelock_oblique_2021-10-26-12-40-20", "gt": 2.7, "marker_size": 0.4,"marker_type":"oblique","trial_type":"gazelock"},
        {"trial": "approxdist_330cm_markersize_400mm_heuristic4gain_0.075_gazelock_oblique_2021-10-26-12-48-25", "gt": 3.3, "marker_size": 0.4,"marker_type":"oblique","trial_type":"gazelock"},
        {"trial": "approxdist_400cm_markersize_400mm_heuristic4gain_0.075_gazelock_oblique_2021-10-26-13-00-56", "gt": 4, "marker_size": 0.4,"marker_type":"oblique","trial_type":"gazelock"},
        {"trial": "approxdist_500cm_markersize_400mm_heuristic4gain_0.075_gazelock_oblique_2021-10-26-13-37-15", "gt": 5, "marker_size": 0.4,"marker_type":"oblique","trial_type":"gazelock"}
    ]
    return [trials_ld_gl_straight,trials_ld_straight,trials_ld_oblique,trials_ld_gl_oblique]
