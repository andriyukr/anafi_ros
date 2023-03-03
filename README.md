# ROS2 Bridge for Parrot ANAFI Drones
This ROS2 package contains interface to Olympe SDK. Currently, it is compatible with the following models from Parrot ANAFI family: 4K, Thermal, USA, Ai.

## Overview

**Author:** Andriy Sarabakha<br />
**Affiliation:** [Nanyang Technological University (NTU)](https://www.ntu.edu.sg), Singapore<br />
**Maintainer:** Andriy Sarabakha, andriy.sarabakha@ntu.edu.sg

**Keywords:** Parrot, UAV, controller

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

This package has been tested with **python3** in **ROS2 Humble**/**Ubuntu 22.04**.

### Dependencies

- [Parrot Olympe](https://developer.parrot.com/docs/olympe/installation.html) - SDK for Parrot drones:

      pip install parrot-olympe
      
  Check installation:
  
      python -c 'import olympe; print("Installation OK")'
      
  **Troubleshooting**

  - *Issue*

        /usr/bin/env: ‘python’: No such file or directory
      
    *Solution*
  
    Set `python3` as default `python` version:

        echo 'alias python=python3' >> ~/.bash_aliases
        source ~/.bash_aliases
  
  - *Issue*

        TypeError: Expected a message Descriptor, got Descriptor
    
    *Solution*
  
    Install `protobuf` version `3.6`:

        pip install protobuf==3.6
    
  - *Issue*

        AttributeError: module 'collections' has no attribute 'MutableMapping'
    
    *Solution*
  
    Install `protobuf` version `3.20.0`:

        pip install protobuf==3.20.0

- [OpenCV](https://pypi.org/project/opencv-python/) - library for real-time computer vision:

      pip install opencv-python
    
- [SciPy](https://scipy.org/install/) - library for scientific and technical computing:

      pip install scipy
    
### Clone

To build from source, clone the latest version from this repository into your ROS2 workspace and build the package using:

	cd ~/dev_ws/src
	git clone -b ros2 https://github.com/andriyukr/olympe_bridge.git
	sudo chmod -R 777 olympe_bridge/
	cd ..
	colcon build
	
  - *Issue*

        SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
    
    *Solution*
  
    Install `setuptools` version `58.2.0`:

        pip install setuptools==58.2.0
        	
## Usage

To connect to the drone, run in the terminal:

    ros2 launch olympe_bridge_nodes anafi_launch.py
    
## Package details

The complete list of [subscribed](details.md#subscribed-topics) and [published](details.md#published-topics) topics, available [services](details.md#services) and [parameters](details.md#parameters) is [here](details.md).
