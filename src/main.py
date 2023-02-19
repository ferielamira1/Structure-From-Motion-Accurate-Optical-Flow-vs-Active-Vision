# -*- coding: utf-8 -*-


import rospy
from scipy.spatial.transform import Rotation as R
import cv2
import torch
#import torchvision
torch.cuda.is_available = lambda: True
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
from mmflow.apis import init_model, inference_model
from mmflow.datasets import visualize_flow, write_flow

import time
import subprocess
import importlib
from rospy_message_converter import message_converter

from src.utils.utils_rosbag import *
from src.utils.utils_opticflow import *
from src.utils.utils_detection import *
from src.utils.utils_plot import *
from src.utils.utils_depth import *

if __name__ == '__main__':
    errors = []
    models = get_models()
    trials = get_trials()
    trials= np.array(trials).flatten()

    for algo_counter, model in enumerate(models):

        corner_counter = 0
        config_file =  model["config_f"]

        checkpoint_file = model["checkpoint_f"]
        mname = model["name"]
        gts = []  # Ground truth distances
        gts_ros=[]
        # Build the model from a config file and a checkpoint file
        model = init_model(config_file, checkpoint_file, device='cuda:0')
        for counter, t in enumerate(trials):
            print(" Trial type: {}, Marker type {}".format(t["trial_type"],t["marker_type"] ))   
            rosbagf = "rosbags/{}.bag".format(t["trial"])
            # Convert the required Rosbag topics to dicts
            
            try:
              rospy.init_node("depth_estimation")
              buffer = tf2.Buffer(rospy.Duration(10000))
            except:
              print("Already registered")
            
     
            camera_infos = rosbag_to_dict(rosbagf, "/camera/color/camera_info")

            cam_poses = rosbag_to_dict(rosbagf, "/vrpn_client_node/pandactrl5_EE/pose")

            target_poses = rosbag_to_dict(rosbagf, "/vrpn_client_node/GCVS_largedist_target/pose")

            frames, frame_infos = rosbag_to_dataset(rosbagf, "/camera/color/image_raw/compressed",
                                                    "datasets/" + t["trial_type"] + "_" + t["marker_type"] + "_" + str(
                                                        t["gt"]) + "m")

            t_vels, r_vels, stime = rosbag_to_vel(rosbagf, ['/tf', '/tf_static'],
                                                  "/franka_state_controller/franka_states",buffer)

            frame_infos, frames = find_match(stime, frame_infos, frames)
            # Compute fps
            stamp1 = rospy.Time(secs=frame_infos[0]["header"]["stamp"]["secs"],
                                nsecs=frame_infos[0]["header"]["stamp"]["nsecs"])
            stamp2 = rospy.Time(secs=frame_infos[2]["header"]["stamp"]["secs"],
                                nsecs=frame_infos[2]["header"]["stamp"]["nsecs"])
            duration = (stamp2 - stamp1).to_sec()
            fps = 1 / duration
            camera_matrix = np.array(camera_infos[0]["K"]).reshape(3, 3)
            distortion_coefficients = np.array(camera_infos[0]["D"])
            focal_length = (camera_matrix[0][0] + camera_matrix[1][1]) / 2

            Z_mcenters = []  # Array for the depth of centers of the marks
            avg_depths = []  # Array for the average depth of the marks

            display_flow = False  # Whether or not to display the flow
            display_marker = False  # Whether or not to display the flow

            last_corners = np.array([])

            start_time = time.time()

            dists=[]
            # Ground-truth distances
            for j in range(len(target_poses)):
              t_p= np.array([[target_poses[j]["pose"]["position"]["x"]], [target_poses[j]["pose"]["position"]["y"]],[target_poses[j]["pose"]["position"]["z"]]])
              c_p= np.array([[cam_poses[j]["pose"]["position"]["x"]], [cam_poses[j]["pose"]["position"]["y"]],[cam_poses[j]["pose"]["position"]["z"]]])
              dists.append(abs(np.linalg.norm(t_p-c_p)))

            gt = np.mean(np.array(dists))
           
            if t["gt"] not in gts_ros:
              gts.append(gt)
              gts_ros.append(t["gt"] )
        
         
            for i in range(240): # was picked to be fair between trials since they contain different numbers of 
                # Calling only indices that allign with the poses (there are half as many poses as there are frames)
                img1 = frames[i * 2]
                img2 = frames[(i + 1) * 2]
                # Corner detection
                corners, succ = corner_detection(img1, img2, last_corners, i)
                # If no corner was detected and there is no alternative corner, skip iteration 
                if succ == False:
                    continue

                last_corners = corners

                # Optical flow inference: in case of gazelock it will be a zero array 
                flow = inference(model,mname, img1, img2, fps,True, t, i)
                
                mflow = marker_flow(corners ,flow ,t["trial_type"] ,img1.shape) # (pixel/s)

                avg_depth = depth_estimation(i, mflow, corners, t_vels, r_vels, focal_length,
                                                              img2.shape, t["trial_type"],camera_matrix)
                if avg_depth > 0:
                    avg_depths.append(avg_depth)
                   

            # Finished one trial
            avg_depths = iqr(1.0, avg_depths)
            trial_depth = sum(avg_depths) / len(avg_depths)
          
            trial_error = abs(trial_depth - gt)

            runtime = (time.time() - start_time)

            error_dict = {'gt': gt,
                          'marker_type': t["marker_type"],
                          'trial_type': t["trial_type"],
                          'mname': mname,
                          'error': round(trial_error, 3),
                          'stdev': np.std(avg_depths) /4, # for visualization only 
                          'runtime': runtime}
            errors.append(error_dict)
            print(error_dict)

    plot_all(gts_ros, errors)


