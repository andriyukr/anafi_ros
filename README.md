# ROS2 Bridge for Parrot ANAFI Drones

This ROS2 package contains the interface to the Olympe SDK. Currently, it is compatible with the following models from the **Parrot ANAFI** family: **4K**, **Thermal**, **USA** and **Ai**.

## Overview

**Author:** Andriy Sarabakha<br />
**Affiliation:** [Aarhus University]([https://www.au](https://international.au.dk/)), Denmark<br />
**Maintainer:** Andriy Sarabakha, andriy@ece.au.dk

**Keywords:** Parrot Anafi, ROS, controller

This is research code, expect that it changes often, and any fitness for a particular purpose is disclaimed.

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

This package has been tested with **python3** in **ROS2 Humble**/**Ubuntu 22.04** (*recommended*), **ROS2 Kilted**/**Ubuntu 24.04** and **ROS2 Foxy**/**Ubuntu 20.04**.

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

> [!NOTE]  
> To install Python packages using `pip` in Ubuntu 24.04, you can add `--break-system-packages` flag:
> 
>     pip install --user <package_name> --break-system-packages

### Build

To build from source, clone the latest version from this repository into your ROS2 workspace and build the package using:

    cd ~/ros2_ws/src
    git clone -b ros2 https://github.com/andriyukr/anafi_ros.git
    sudo chmod -R 777 anafi_ros/
    cd ..
    colcon build
    source install/setup.bash

**Troubleshooting**

<details> 
  <summary>SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.</summary>

Install `setuptools` version `58.2.0`:

    pip install setuptools==58.2.0
</details>

## Usage

> [!IMPORTANT]
> Remember to update the firmware of the drone and the SkyController.

> [!TIP]
> - To update the firmware of ANAFI **4K** and **Thermal**, use **FreeFlight 6** app for [Android](https://www.parrot.com/en/support/anafi/how-do-i-get-the-android-freeflight-6-app) or [iOS](https://www.parrot.com/en/support/anafi/how-do-i-get-the-ios-freeflight-6-app).
> - To update the firmware of ANAFI **USA**, use **FreeFlight 6 USA** app for [Android](https://www.parrot.com/en/support/anafi-usa/how-do-i-get-android-freeflight-6-usa-app) or [iOS](https://www.parrot.com/en/support/anafi-usa/how-do-i-get-the-ios-freeflight-6-usa-app).
> - To update the firmware of ANAFI **Ai**, use **FreeFlight 7** app for [iOS](https://www.parrot.com/en/support/anafi-ai/how-do-i-get-the-ios-freeflight-7-app).

To connect to the drone, run in the terminal:

    ros2 launch anafi_ros_nodes anafi_launch.py namespace:='anafi' ip:='192.168.53.1' model:='ai'

where
* `namespace` is the namespace for this specific drone, it is used to connect to multiple drones;
* `ip` is the IP address of the device to connect, it has to be
  * `'192.168.53.1'`, for connection through Skycontroller (*recommended*),
  * `'192.168.42.1'`, for direct connection to the drone through WiFi,
  * `'10.202.0.1'`, for connection to the simulated drone in Sphinx;
* `model` (ignored, when connecting through Skycontroller) is the model of the drone, it has to be `'4k'`, `'thermal'`, `'usa'` or `'ai'`, depending on the drone model you are connecting to.

> [!IMPORTANT]
> To send commands to the drone, you must switch to offboard mode. To switch to offboard mode, set `drone/offboard` parameter to `True`.

> [!NOTE]  
> The [`anafi_autonomy`](https://github.com/andriyukr/anafi_autonomy) ROS2 package is based on this `anafi_ros` package and contains an interface to control Parrot Anafi drones.

## Package details

The complete list of [subscribed](details.md#subscribed-topics) and [published](details.md#published-topics) topics, available [services](details.md#services) and [parameters](details.md#parameters) is [here](details.md).

## Connect multiple drones

Follow the instructions [here](multiple-drones.md).

## Simulation environment

Follow the instructions [here](simulation.md).
