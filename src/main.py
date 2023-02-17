import torch
from src.utils.utils_detection import corner_detection
from src.utils.utils_opticflow import get_models, inference
from src.utils.utils_plot import plot_all
from src.utils.utils_rosbag import rosbag_to_dict, rosbag_to_dataset, rosbag_to_vel, find_match, get_trials

torch.cuda.is_available = lambda: True
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
from mmflow.apis import init_model


import time
from src.utils.utils_math import *

import rospy

import numpy as np



if __name__ == '__main__':

    all_corners = []
    errors = []

    models = get_models()
    trials = get_trials()

    for algo_counter, (cf, checkpoint, name) in enumerate(models):

        corner_counter = 0
        config_file = cf
        checkpoint_file = checkpoint
        mname = name

        gts = []  # Ground truth distances

        # Build the model from a config file and a checkpoint file
        model = init_model(config_file, checkpoint_file, device='cuda:0')
        for counter, t in enumerate(trials):
            # Ground-truth distances
            if counter < 8:
                gts.append(t.gt)
            if t.trial_type == "translational" and t.marker_type == "straight":
                print("Translational trial, straight marker")
            elif t.trial_type == "translational" and t.marker_type == "oblique":
                print("Translational trial, oblique marker")
            elif t.trial_type == "gazelock" and t.marker_type == "straight":
                print("Gazelock trial,straight marker")
            elif t.trial_type == "gazelock" and t.marker_type == "oblique":
                print("Gazelock trial, oblique marker")

            rosbagf = "./rosbags/{}.bag".format(t.trial)

            # Convert the required Rosbag topics to dicts
            camera_infos = rosbag_to_dict(rosbagf, "/camera/color/camera_info")
            cam_poses = rosbag_to_dict(rosbagf, "/vrpn_client_node/pandactrl5_EE/pose")

            target_poses = rosbag_to_dict(rosbagf, "/vrpn_client_node/GCVS_largedist_target/pose")

            frames, frame_infos = rosbag_to_dataset(rosbagf, "/camera/color/image_raw/compressed",
                                                    "datasets/" + t.trial_type + "_" + t.marker_type + "_" + str(
                                                        t.gt) + "m")

            t_vels, r_vels, stime = rosbag_to_vel("./rosbags/{}.bag".format(t.trial), ['/tf', '/tf_static'],
                                                  "/franka_state_controller/franka_states")

            frame_infos, frames = find_match(stime, frame_infos, frames)
            # Compute fps
            stamp1 = rospy.Time(secs=frame_infos[0]["header"]["stamp"]["secs"],
                                nsecs=frame_infos[0]["header"]["stamp"]["nsecs"])
            stamp2 = rospy.Time(secs=frame_infos[2]["header"]["stamp"]["secs"],
                                nsecs=frame_infos[2]["header"]["stamp"]["nsecs"])
            duration = (stamp2 - stamp1).to_sec()
            fps = 1 / duration
            camera_matrix = np.array(camera_infos[i]["K"]).reshape(3, 3)
            distortion_coefficients = np.array(camera_infos[i]["D"])
            focal_length = (camera_matrix[0][0] + camera_matrix[1][1]) / 2

            Z_mcenters = []  # Array for the depth of centers of the marks
            avg_depths = []  # Array for the average depth of the marks
            avg_depths_mixed = []  # Array for the average depth of the marks

            display_flow = False  # Whether or not to display the flow
            display_marker = False  # Whether or not to display the flow

            last_corners = np.array([])

            start_time = time.time()

            for i in range(240):  # 240 was picked to be fair between trials since they contain different numbers of
                # Calling only indices that allign with the poses (there are half as many poses as there are frames)
                img1 = frames[i * 2]
                img2 = frames[(i + 1) * 2]

                # Optical flow inference
                flow = inference(model, img1, img2, fps, display_flow, t, i)

                # Corner detection
                if algo_counter == 0:
                    corners, succ = corner_detection(img1, img2, last_corners, i)
                    all_corners.append(corners)
                    last_corners = corners
                    if succ == False:
                        continue
                else:
                    corner_counter += 1
                    if len(all_corners[corner_counter]) == 0:
                        print("Iteration {} skipped due to missing corner".format(i))
                        continue
                    else:
                        corners = all_corners[corner_counter]

                avg_depth, avg_depth_mixed = depth_estimation(i, flow, corners, t_vels, r_vels, focal_length,
                                                              img2.shape, t.trial_type)
                if avg_depth_mixed > 0:
                    avg_depths_mixed.append(abs(avg_depth_mixed))
                if avg_depth > 0:
                    avg_depths.append(abs(avg_depth))

            # Finished one trial
            avg_depths = iqr(1.0, avg_depths)
            trial_depth = sum(avg_depths) / len(avg_depths)
            trial_error = abs(trial_depth - t.gt)

            avg_depths_mixed = iqr(1.0, avg_depths_mixed)
            trial_depth_mixed = sum(avg_depths_mixed) / len(avg_depths_mixed)
            trial_error_mixed = abs(trial_depth_mixed - t.gt)

            runtime = (time.time() - start_time)
            error_dict = {'gt': t.gt,
                          'marker_type': t.marker_type,
                          'trial_type': t.trial_type,
                          'mname': mname,
                          'error': round(trial_error, 3),
                          'stdev': np.std(avg_depths) / 8,
                          'error_mixed': round(trial_error_mixed, 3),
                          'stdev_mixed': np.std(avg_depths_mixed) / 8,
                          'runtime': runtime}
            errors.append(error_dict)

    plot_all(gts, errors)


