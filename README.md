# ROS2 Bridge for Parrot ANAFI Drones
This ROS2 package contains interface to Olympe SDK. Currently, it is compatible with the following models from Parrot ANAFI family: 4K, Thermal, USA, Ai.

## Overview

**Author:** Andriy Sarabakha<br />
**Affiliation:** [Nanyang Technological University (NTU)](https://www.ntu.edu.sg), Singapore<br />
**Maintainer:** Andriy Sarabakha, andriy.sarabakha@ntu.edu.sg

**Keywords:** Parrot Anafi, ROS, controller

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Publications

For more details, please refer to: 

* A. Sarabakha, "`anafi_ros`: from Off-the-Shelf Drones to Research Platforms," *arXiv*, 2023. ([pdf](Parrot_Anafi.pdf))

If you use this work in an academic context, please cite the paper:
```bibtex
@misc{AnafiROS,
  author = {Sarabakha, Andriy},  
  title = {anafi_ros: from Off-the-Shelf Drones to Research Platforms},
  publisher = {arXiv},
  year = {2023},
  doi = {10.48550/ARXIV.2303.01813},
  url = {https://arxiv.org/abs/2303.01813}
}
```

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

	cd ~/ros2_ws/src
	git clone -b ros2 https://github.com/andriyukr/anafi_ros.git
	sudo chmod -R 777 anafi_ros/
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
