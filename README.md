# Structure From Motion: Accurate Optical Flow vs Active Vision

## Project Overview
Estimating depth of objects in a video is an important problem in computer vision, with practical applications in robotics, autonomous driving,etc. . One traditional method for estimating depth is Structure from Motion (SfM), which uses the motion of a camera to reconstruct the 3D structure of a scene. To achieve this two pathways are of interest: 
1. Generating camera movement and estimating the velocity of the target projection onto the image plane using state-of-the-art optical flow algorithms. Then relating the 3D camera velocity to the velocity of the projected target
2. Orchestrating the camera movement to fixate on the target, which creates a coupling between the translational and rotational velocities of the camera. This coupling is then used to estimate the depth of the target.  This method relies on the robots proprioception and was introduced by [5]

In this work, we implement and compare the above mentioned methods and  compare their performance in terms of accuracy, run-time, and specifications. 

We found that IRR-PWC had the best performance among the tested DL-based optic flow algorithms (RAFT,GMA,IRR-PWC).
We found that Sfm with fixation-based active vision is better suited in scenarios where the target is close to the camera 
We found that Sfm with optic flow  is better suited in scenarios where the target has more complex features or bigger size and is further away from the camera.

## Code Structure
The project's code is organized as follows:

- rosbags: contains the data used in the experiments
- models: contains the trained models for the different algorithms. 
- results: contains the results of the experiments
- src: contains the main.py (and .ipynb) file and the necessarry utilities for handling the rosbags, target detection, optic flow and depth estimation.
### Getting Started
The code was implemented and tested on Google Colab, I added a main.py version in case there are problems with the execution on Colab.
#### Local execution
To run the project, you will need to clone this repository and install the dependencies. 
You can use the following commands:
1. Clone repository and enter top folder:
2. Install the OpenMIM library :
``` pip3 install -U openmim ``` 
3. Install the MMCV-full package using MIM:
``` mim install mmcv-full``` 
4. Install the MMFlow library :
``` pip3 install mmflow ```
5. Install ROS (Robot Operating System)

Before runninng the code the rosbags in the "largedist_eval" folder on tubcloud need to downloaded  under their original names and put in the rosbags folder

``` 
sudo sh -c  echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'])
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key  C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update'])
sudo apt install ros-noetic-desktop-full
pip install --extra-index-url https://rospypi.github.io/simple/ rospy
pip install --extra-index-url https://rospypi.github.io/simple/ tf2-ros
pip install bagpy
pip install cvbridge3
``` 
To run the code you first need to start roscore then run src/main.py
#### Note: 
The above installation commands assume the use of Ubuntu and ROS Noetic. If using a different operating system or version of ROS, the installation commands may be different.

#### Google Colab

1. Open Google Colab in your web browser.
2. Click on "New Notebook" to create a new notebook. 
3. In the new notebook, Change the "Runtime type" to "GPU" 
4. Click on "File" in the top left corner and select "Open notebook". 
5. In the dialog box that appears, click on the "GitHub" tab. 
6. Enter the URL of the GitHub repository you want to open in the search box and click on the magnifying glass icon to search. 
7. Once the repository is found, click on the "Open in Colab" button next to the repository name. 
8. Before runninng the code the rosbags in the "largedist_eval" folder on tubcloud need to downloaded  under their original names and put in the rosbags folder
9. Once the repository is loaded, you can install all dependencies, start roscore and run the code by opening src/main.ipynb  clicking on "Run all"
#### Note: 
If the repository is private, you'll need to provide Colab with your GitHub credentials to access it. You can do this by clicking on the "Connect to GitHub" button that appears when you try to open a private repository.

## Conclusion
The project explored the performance of state-of-the-art optical flow algorithms for depth estimation and compared it with fixation-based active vision. The project found that IRR-PWC had the best performance among the tested algorithms and compared its performance with fixation-based active vision. The project's code is available in the src directory, and the results of the experiments are available in the results directory.

## References
### Literature
[1]	S. Bae, J. Chung, J. Son, and K. Sohn, "Learning to Estimate Hidden Motions with Global Motion 	Aggregation," arXiv:2104.02409 [cs.CV], Apr. 2021.

[2]	J. Hur and S. Roth, "Iterative residual refinement for joint optical flow and occlusion estimation," in 	Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition, 2019, pp. 	5754-5763.

[3]	D. Sun, X. Yang, M.-Y. Liu, and J. Kautz, "PWC-Net: CNNs for Optical Flow Using Pyramid, 	Warping, and Cost Volume," in 2018 IEEE Conference on Computer Vision and Pattern 	Recognition, 2018, pp. 8934-8943.

[4]	Z. Teed and J. Deng, "RAFT: Recurrent All-Pairs Field Transforms for Optical Flow," in 2020 	IEEE/CVF Conference on Computer Vision and Pattern Recognition, 2020, pp. 3853-3861.

[5]	A. Battaje and O. Brock, "One Object at a Time: Accurate and Robust Structure From Motion for 	Robots," in 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 	2022, pp. 2126-2133
### Toolboxes
```
@misc{2021mmflow,
    title={{MMFlow}: OpenMMLab Optical Flow Toolbox and Benchmark},
    author={MMFlow Contributors},
    howpublished = {\url{https://github.com/open-mmlab/mmflow}},
    year={2021}
}
```

```
@article{mmdetection,
  title   = {{MMDetection}: Open MMLab Detection Toolbox and Benchmark},
  author  = {Chen, Kai and Wang, Jiaqi and Pang, Jiangmiao and Cao, Yuhang and
             Xiong, Yu and Li, Xiaoxiao and Sun, Shuyang and Feng, Wansen and
             Liu, Ziwei and Xu, Jiarui and Zhang, Zheng and Cheng, Dazhi and
             Zhu, Chenchen and Cheng, Tianheng and Zhao, Qijie and Li, Buyu and
             Lu, Xin and Zhu, Rui and Wu, Yue and Dai, Jifeng and Wang, Jingdong
             and Shi, Jianping and Ouyang, Wanli and Loy, Chen Change and Lin, Dahua},
  journal= {arXiv preprint arXiv:1906.07155},
  year={2019}
}```
