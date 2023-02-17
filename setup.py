import subprocess


subprocess.run(['pip3', 'install', '-U', 'openmim'])
subprocess.run(['mim', 'install', 'mmcv-full'])
subprocess.run(['pip', 'install', 'mmflow'])


# ROS installation
subprocess.run(['sudo', 'sh', '-c', 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'])
subprocess.run(['sudo', 'apt-key', 'adv', '--keyserver', 'hkp://keyserver.ubuntu.com:80', '--recv-key', 'C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'])
subprocess.run(['sudo', 'apt', 'update'])
subprocess.run(['sudo', 'apt', 'install', 'ros-noetic-desktop-full'])
subprocess.run(['pip', 'install', '--extra-index-url', 'https://rospypi.github.io/simple/', 'rospy'])
subprocess.run(['pip', 'install', '--extra-index-url', 'https://rospypi.github.io/simple/', 'tf2-ros'])
subprocess.run(['pip', 'install', 'bagpy'])
subprocess.run(['pip', 'install', 'cvbridge3'])


