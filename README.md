# ROS Bridge for Parrot Drones
This ROS package contains interface to Olympe SDK. Currently, it supports only Parrot Anafi drones (4K, Thermal, USA, AI).

## Overview

**Author:** Andriy Sarabakha<br />
**Affiliation:** [Nanyang Technological University (NTU)](https://www.ntu.edu.sg), Singapore<br />
**Maintainer:** Andriy Sarabakha, andriy.sarabakha@ntu.edu.sg

**Keywords:** Parrot, UAV, controller

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

This package has been tested with **python3** in **ROS Melodic**/**Ubuntu 18.04** and **ROS Noetic**/**Ubuntu 20.04**.

### Dependencies

- [Parrot Olympe](https://developer.parrot.com/docs/olympe/installation.html) - SDK for Parrot drones:

      pip install parrot-olympe
      
  Check installation:
  
      python -c 'import olympe; print("Installation OK")'
      
  **Troubleshooting**
  
  - *Issue*

        ERROR: Could not find a version that satisfies the requirement parrot-olympe (from versions: none)
        ERROR: No matching distribution found for parrot-olympe
   
    *Solution*
  
    Install the latest version of `pip`:

        sudo apt-get install python3-pip python-dev
        echo 'export PATH="~/.local/bin:$PATH"' >> ~/.bashrc
        source ~/.bashrc

  - *Issue*

        /usr/bin/env: ‘python’: No such file or directory
      
    *Solution*
  
    Set `python3` as default `python` version:

        echo 'alias python=python3' >> ~/.bash_aliases
        source ~/.bash_aliases
    
  - *Issue*

        AttributeError: module 'collections' has no attribute 'MutableMapping'
    
    *Solution*
  
    Install `protobuf` version `3.20.0`:

        pip install protobuf==3.20.0

- [OpenCV](https://pypi.org/project/opencv-python/) - library for real-time computer vision:

      pip install opencv-python
    
- [SciPy](https://scipy.org/install/) - library for scientific and technical computing:

      pip install scipy    
    
## Clone

Clone the latest version from this repository into your ROS workspace:

	cd ~/catkin_ws/src
	git clone -b ros1 https://github.com/andriyukr/olympe_bridge.git
	sudo chmod -R 777 olympe_bridge/

## Run

    roslaunch olympe_bridge anafi.launch
