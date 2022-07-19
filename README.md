# ROS Bridge for Parrot Drones
This ROS package contains interface to Olympe SDK. Currently, it supports only the Parrot Anafi drone.

## Overview

**Author:** Andriy Sarabakha<br />
**Affiliation:** [Technical University of Munich (TUM)](https://www.tum.de/en/), Germany<br />
**Maintainer:** Andriy Sarabakha, andriy.sarabakha@tum.de

**Keywords:** Parrot, UAV, controller

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

This package has been tested with **python3** in **ROS Melodic**/**Ubuntu 18.04** and **ROS Noetic**/**Ubuntu 20.04**.

### Dependencies

- [Parrot Ground SDK](https://developer.parrot.com/) - SDK for Parrot drones:

      mkdir -p ~/code/parrot-groundsdk
      cd ~/code/parrot-groundsdk
      sudo snap install git-repo
      git config --global user.email "you@example.com"
      git config --global user.name "Your Name"
      repo init -u https://github.com/Parrot-Developers/groundsdk-manifest.git
      repo sync
      ./products/olympe/linux/env/postinst
      ./build.sh -p olympe-linux -A all final -j
    
### Troubleshooting

#### Issue:
    pkg_resources.DistributionNotFound: The 'osrf-pycommon>0.1.1' distribution was not found and is required by catkin-tools
#### Solution:
Install python3-catkin-tools:

    sudo apt install python3-catkin-lint python3-pip
    pip3 install osrf-pycommon

#### Issue:
    File "/home/andriy/code/parrot-groundsdk/.repo/repo/main.py", line 79
      file=sys.stderr)
          ^
    SyntaxError: invalid syntax
#### Solution:
Use Google Repo binary:

    mkdir -p ~/.bin
    PATH="${HOME}/.bin:${PATH}"
    curl https://storage.googleapis.com/git-repo-downloads/repo > ~/.bin/repo
    chmod a+rx ~/.bin/repo
    
#### Issue:
    /usr/bin/env: ‘python’: No such file or directory
#### Solution:
Set python3 as default:

    echo 'alias python=python3' >> ~/.bash_aliases
    source ~/.bash_aliases
    
Create a symbolic link to python3:

    sudo ln -s /usr/bin/python3 /usr/bin/python

## Check
    source ~/code/parrot-groundsdk/./products/olympe/linux/env/shell
    python -c 'import olympe; print("Installation OK")'
    
### Troubleshooting

#### Issue:
    ModuleNotFoundError: No module named 'olympe.messages'
#### Solution:
Downgraded aenum:

    pip3 install --upgrade aenum==2.2.5

#### Issue:
    ModuleNotFoundError: No module named 'ulog'
#### Solution:
Downgraded aenum:

    pip install ulog

#### Issue:
    AttributeError: 'module' object has no attribute 'abc'
#### Solution:
Set python3 as default:

    echo 'alias python=python3' >> ~/.bash_aliases
    source ~/.bash_aliases
    
## Clone

Clone the latest version from this repository into your catkin workspace using:

	cd ~/catkin_ws/src
	git clone https://github.com/andriyukr/olympe_bridge.git
	sudo chmod -R 777 olympe_bridge/

## Run
    source ~/code/parrot-groundsdk/./products/olympe/linux/env/shell
    roslaunch olympe_bridge anafi.launch

### Troubleshooting

#### Issue:
    ModuleNotFoundError: No module named 'roslaunch'
#### Solution:
Set up ROS environment:

    echo 'source /opt/ros/noetic/setup.bash' >> ~/code/parrot-groundsdk/./products/olympe/linux/env/shell
    echo 'source ~/catkin_ws/devel/setup.bash' >> ~/code/parrot-groundsdk/./products/olympe/linux/env/shell
    source ~/code/parrot-groundsdk/./products/olympe/linux/env/shell

#### Issue:
    ModuleNotFoundError: No module named 'cv2'
#### Solution:
Install OpenCV for python:

    pip install opencv-python
