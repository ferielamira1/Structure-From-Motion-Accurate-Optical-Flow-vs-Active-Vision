from mmflow.apis import init_model, inference_model
from mmflow.datasets import visualize_flow, write_flow
import numpy as np
import random

def get_models():
    """
    A function that returns a list of dictionaries, each representing an optic flow algorithm model.
    Each dictionary contains the name of the algorithm, its configuration file, and its corresponding checkpoint.
    Returns:
    models (list): A list of dictionaries, each representing an optic flow algorithm model.

    """

    # Name of optic flow algorithm, configuration files and checkpoints
    # PWC-NET
    pwcnet_dict = {"config_f": 'models/configs/pwcnet/pwcnet_ft_4x1_300k_sintel_final_384x768.py',
                   "checkpoint_f": "models/checkpoints/pwcnet_ft_4x1_300k_sintel_final_384x768.pth",
                   "name": "PWC-NET "}

    # GMA
    gma_dict = {
        "config_f": "models/configs/gma/gma_8x2_120k_flyingthings3d_sintel_368x768.py",
        "checkpoint_f": "models/checkpoints/gma_8x2_120k_flyingthings3d_sintel_368x768.pth",
        "name": "GMA"
    }

    # RAFT
    raft_dict = {
        "config_f": "models/configs/raft/raft_8x2_100k_flyingthings3d_sintel_368x768.py",
        "checkpoint_f": "models/checkpoints/raft_8x2_100k_flyingthings3d_sintel_368x768.pth",
        "name": "RAFT"
    }

    # IRR-PWC
    irrpwc_dict = {
        "config_f": "models/configs/irr/irrpwc_ft_4x1_300k_sintel_final_384x768.py",
        "checkpoint_f": "models/checkpoints/irrpwc_ft_4x1_300k_sintel_final_384x768.pth",
        "name": " IRR-PWC"
    }
    models = [pwcnet_dict, irrpwc_dict, raft_dict, gma_dict]
    return models


def marker_flow(corners ,flow, marker_type ,img_shape):
    """
    Computes the mean optical flow velocities within a region of interest defined by the marker corners.
    Args:
    - corners (np.ndarray): 2D array of shape (4, 2) with (x, y) coordinates of the marker corners.
    - flow (np.ndarray): 3D array of shape (height, width, 2) with the optical flow vectors in the x and y directions.
    - marker_type (str): Type of the marker, either "oblique" or "rectangular".
    - img_shape (tuple): Tuple with (height, width) of the image.

    Returns:
    - np.ndarray: 2D array of shape (2, 1) with the mean optical flow velocity in the x and y directions, respectively.
    """
    if marker_type=='oblique':
        offset_out= 2  # diff / thresh
    else:
        offset_out =0

    h_min = min([int(corners[0][1]) ,int(corners[1][1]) ,int(corners[2][1]) ,int(corners[3][1])] ) +offset_out
    h_max = max([int(corners[0][1]) ,int(corners[1][1]) ,int(corners[2][1]) ,int(corners[3][1])] ) -offset_out
    w_min = min([int(corners[0][0]) ,int(corners[1][0]) ,int(corners[2][0]) ,int(corners[3][0])] ) +offset_out
    w_max = max([int(corners[0][0]) ,int(corners[1][0]) ,int(corners[2][0]) ,int(corners[3][0])] ) -offset_out

    rect = corners.reshape([4, 1, 2]).astype(np.int64)
    img_velocity_x = []
    img_velocity_y = []

    # Accumulate flow
    for y in range(h_min ,h_max):
        for x in range(w_min ,w_max):
            dist= cv2.pointPolygonTest(rect ,(x ,y) ,False)
            if (dist>=0 and random.choice([True, False])):  # and (x == w_min or x==w_max or y == h_min or y == h_max)):
                img_velocity_x.append(flow[y ,x ,0])
                img_velocity_y.append(flow[y ,x ,1])

    img_velocity_x = iqr(1.0,np.array(img_velocity_x))
    img_velocity_x = np.mean(img_velocity_x)

    img_velocity_y = iqr(1.0,np.array(img_velocity_y))
    img_velocity_y = np.mean(img_velocity_y)

    return np.array([[img_velocity_x] ,[img_velocity_y]])

def inference(model,img1,img2,fps,display_flow,t,i):
    """
    Function: inference

    This function takes in a pretrained model by mmflow [https://mmflow.readthedocs.io/en/latest/], two input images and performs inference
    on the input images.
    If the trial type is "translational," the function uses an DL based optic flow model to generate and returns an optical
    flow estimate between the two images, and saves the flow file and the visualized flow map.
    If the trial type is "gazelock" the function returns a zero flow vector.

    Parameters:

    model: A pretrained model for optical flow estimation by MMFlow
    img1: The first image frame
    img2: The second image frame
    fps: The frame rate of the input images in frames per second
    display_flow: A boolean flag indicating whether to display the flow map during inference
    t: A dict containing information about the trial, including the trial type.
    i: The index of the current image pair (used for file naming)
    Returns:

    flow: An estimated flow vector between the two input images
    """
    if t.trial_type == "translational":
        flow = inference_model(model, img1, img2) * fps  # pixel / second
        # Save the optical flow file
        if display_flow and i % 4 == 0:
            write_flow(flow, flow_file='flow_{}.flo'.format(i))
            # save the visualized flow map
            flow_map = visualize_flow(flow, save_file='flow/flow_map_{}_{}_{}_{}.png'.format(
                t.marker_type, t.trial_type, str(t.gt), str(i * 2)))
    else:
        flow = np.array([[0], [0], [0]])
    return flow
