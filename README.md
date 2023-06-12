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

* A. Sarabakha, "anafi_ros: from Off-the-Shelf Drones to Research Platforms," in *2023 International Conference on Unmanned Aircraft Systems (ICUAS)*, 2023, pp. 1308-1315. ([pdf](Parrot_Anafi.pdf))

If you use this work in an academic context, please cite the paper:
```bibtex
@INPROCEEDINGS{AnafiROS,
  author={Sarabakha, Andriy},
  booktitle={2023 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
  title={{anafi_ros: from Off-the-Shelf Drones to Research Platforms}}, 
  year={2023},
  volume={},
  number={},
  pages={1308--1315},
  doi = {10.48550/ARXIV.2303.01813},
  url = {https://arxiv.org/abs/2303.01813}
}
```

## Installation

This package has been tested with **python3** in **ROS2 Humble**/**Ubuntu 22.04** (*recommended*) and **ROS2 Foxy**/**Ubuntu 20.04**.

### Dependencies

- [Parrot Olympe](https://developer.parrot.com/docs/olympe/installation.html) **7.5.0** - SDK for Parrot drones:

      pip install parrot-olympe==7.5.0
      
  Check installation:
  
      python -c 'import olympe; print("Installation OK")'
      
  **Troubleshooting**
  
    <details> 
        <summary>ERROR: Could not find a version that satisfies the requirement parrot-olympe (from versions: none)</summary>
	
    	Install the latest version of `pip`:

        	sudo apt-get install python3-pip python-dev
        	echo 'export PATH="~/.local/bin:$PATH"' >> ~/.bashrc
        	source ~/.bashrc
    </details>
    
    <details> 
        <summary>Command 'python' not found</summary>
  
    	Set `python3` as default `python` version:

        	echo 'alias python=python3' >> ~/.bash_aliases
        	source ~/.bash_aliases
    </details>
    
    <details> 
        <summary>TypeError: Expected a message Descriptor, got Descriptor</summary>
  
    	Install `protobuf` version `3.6`:

        	pip install protobuf==3.6
    </details>
    
    <details> 
        <summary>AttributeError: module 'collections' has no attribute 'MutableMapping'</summary>
  
    	Install `protobuf` version `3.20.0`:

        	pip install protobuf==3.20.0
    </details>
  
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
  
**Troubleshooting**
  
<details> 
  <summary>SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.</summary>
  
   Install `setuptools` version `58.2.0`:

     pip install setuptools==58.2.0
</details>
        	
## Usage

To connect to the drone, run in the terminal:

    ros2 launch anafi_ros_nodes anafi_launch.py
    
## Package details

The complete list of [subscribed](details.md#subscribed-topics) and [published](details.md#published-topics) topics, available [services](details.md#services) and [parameters](details.md#parameters) is [here](details.md).
