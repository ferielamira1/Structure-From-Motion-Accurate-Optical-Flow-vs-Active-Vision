# Structure From Motion: Accurate Optical Flow vs Active Vision

## Introduction
This project aims to explore and compare the performance of two approaches for depth estimation using visual data captured by a monocular camera: accurate optical flow and active vision. The project makes use of state-of-the-art optical flow algorithms such as PWC-Net, RAFT, GMA, and IRR-PWC, and compares their performance to that of fixation-based active vision.

## Background
Structure from Motion (SfM) is a computer vision technique that leverages motion parallax to estimate depth to any visible point in the world. Optical flow is a commonly used tool in SfM, which captures the apparent movement of different objects in the visual field. However, many classical optical flow methods are plagued by the aperture problem, resulting in distance overestimates.

Active vision, on the other hand, involves taking actions to simplify perception. Fixation-based active vision has shown superior depth recovery to specific points than naive SfM, due to regularities made apparent in visual motion representation (optic flow) that sidesteps the aperture problem.

## Project Overview
The project aims to compare the performance of state-of-the-art optical flow algorithms such as PWC-Net, RAFT, GMA, and IRR-PWC for depth estimation, and to compare their performance with that of fixation-based active vision. The project uses the mmflow library for deep learning-based optic flow algorithms and mmdetection for marker detection.

The project found that IRR-PWC had the best performance among the tested algorithms, and compared its performance with fixation-based active vision.

## Code Structure
The project's code is organized as follows:

- rosbags: contains the data used in the experiments
- models: contains the trained models for the different algorithms. This 
- results: contains the results of the experiments
- src: contains the source code for the project, including an .ipynb implementation.
### Getting Started
#### Local execution
To run the project, you will need to clone this repository and install the dependencies. 
You can use the following commands:
1. Clone repository and enter top folder:
``` 
git clone ... 
cd ./depth_estimation
```

3. The OpenMIM library should be installed:

``` pip3 install -U openmim ``` 
2. The MMCV-full package should be installed using MIM:

``` mim install mmcv-full``` 
3. The MMFlow library should be installed:


``` pip3 install mmflow ``` 
4. ROS (Robot Operating System) should be installed:

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
#### Note: 
The above installation commands assume the use of Ubuntu and ROS Noetic. If using a different operating system or version of ROS, the installation commands may be different.

#### Google Colab
The installation is self-contained in "src/main.ipynb"

After installing the dependencies, you can run the experiments using the scripts in the src directory.

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
