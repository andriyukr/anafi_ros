# ROS Bridge for Parrot Drones

This ROS2 package contains the interface to Olympe SDK. Currently, it is compatible with the following models from the **Parrot ANAFI** family: **4K**, **Thermal**, **USA** and **Ai**.

## Overview

**Author:** Andriy Sarabakha<br />
**Affiliation:** [Nanyang Technological University (NTU)](https://www.ntu.edu.sg), Singapore<br />
**Maintainer:** Andriy Sarabakha, andriy.sarabakha@ntu.edu.sg

**Keywords:** Parrot Anafi, ROS, controller

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Publications

For more details, please refer to: 

* A. Sarabakha and P. N. Suganthan, "anafi_ros: from Off-the-Shelf Drones to Research Platforms," *2023 International Conference on Unmanned Aircraft Systems (ICUAS)*, Warsaw, Poland, 2023, pp. 1308-1315, doi: [10.1109/ICUAS57906.2023.10155881](https://doi.org/10.1109/ICUAS57906.2023.10155881). ([PDF](Parrot_Anafi.pdf))

If you use this work in an academic context, please cite the paper:
```bibtex
@INPROCEEDINGS{anafiROS,
  author={Sarabakha, Andriy and Suganthan, Ponnuthurai Nagaratnam},
  booktitle={2023 International Conference on Unmanned Aircraft Systems~(ICUAS)}, 
  title={{anafi_ros: from Off-the-Shelf Drones to Research Platforms}}, 
  year={2023},
  volume={},
  number={},
  pages={1308-1315},
  doi={10.1109/ICUAS57906.2023.10155881}}
```

## Installation

This package has been tested with **python3** in **ROS Melodic**/**Ubuntu 18.04** and **ROS Noetic**/**Ubuntu 20.04**.

### Dependencies

- [Parrot Olympe](https://developer.parrot.com/docs/olympe/installation.html) **7.5.0** - SDK for Parrot drones:

      pip install parrot-olympe==7.5.0

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

    Set `python3` as the default `python` version:

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
    
### Build

Clone the latest version from this repository into your ROS workspace:

	cd ~/catkin_ws/src
	git clone -b ros1 https://github.com/andriyukr/anafi_ros.git
	sudo chmod -R 777 anafi_ros/

**Troubleshooting**

<details> 
  <summary>SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.</summary>

Install `setuptools` version `58.2.0`:

    pip install setuptools==58.2.0
</details>

## Usage

    roslaunch olympe_bridge anafi.launch ip:='192.168.53.1' model:='ai'

where
* `ip` is the IP address of the device to connect, it has to be
  * `'192.168.53.1'`, for connection through Skycontroller (*recommended*),
  * `'192.168.42.1'`, for direct connection to the drone through WiFi,
  * `'10.202.0.1'`, for connection to the simulated drone in Sphinx;
* `model` (ignored, when connecting through Skycontroller) is the model of the drone, it has to be `'4k'`, `'thermal'`, `'usa'` or `'ai'`, depending on the drone model you are connecting to.

## Connect multiple drones

Follow the instructions [here](https://github.com/andriyukr/anafi_ros/blob/ros2/multiple-drones.md).

## Simulation environment

Follow the instructions [here](https://github.com/andriyukr/anafi_ros/blob/ros2/simulation.md).
