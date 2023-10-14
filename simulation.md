# Simulation Environment

The simulation environment is based on [Parrot Sphinx](https://developer.parrot.com/docs/sphinx/index.html). It supports [ANAFI 4K](https://developer.parrot.com/docs/sphinx/supported_drones.html#anafi) and [ANAFI Ai](https://developer.parrot.com/docs/sphinx/supported_drones.html#anafi-ai).

> [!NOTE]
> Sphinx simulator does not allow running multiple drones.

## Installation

> [!IMPORTANT]
> Sphinx simulator runs only on 64-bit **Ubuntu 20.04** and **Ubuntu 22.04**.

Follow the [installation procedure](https://developer.parrot.com/docs/sphinx/installation.html).

## Usage

1. *In terminal 1*,

   1. start the system service firmwared:

          sudo systemctl start firmwared.service
   
   2. launch the firmware
  
      * for ANAFI **Ai**:
      
            sphinx "/opt/parrot-sphinx/usr/share/sphinx/drones/anafi_ai.drone"::firmware="https://firmware.parrot.com/Versions/anafi2/pc/%23latest/images/anafi2-pc.ext2.zip"
      
      * for ANAFI **4K**:
      
            sphinx "/opt/parrot-sphinx/usr/share/sphinx/drones/anafi.drone"::firmware="https://firmware.parrot.com/Versions/anafi/pc/%23latest/images/anafi-pc.ext2.zip"

2. *In terminal 2*, start the empty world

       parrot-ue4-empty

   > [!NOTE]
   > You can check other [available worlds](https://developer.parrot.com/docs/sphinx/available_worlds.html).

3. *In terminal 3*, launch ROS bridge
  
   * for ANAFI **Ai**:
      
         ros2 launch anafi_ros_nodes anafi_launch.py ip:='10.202.0.1' model:='ai'

   * for ANAFI **4K**:
      
         ros2 launch anafi_ros_nodes anafi_launch.py ip:='10.202.0.1' model:='4k'

### Interaction with Sphinx

It is possible to interact with Sphinx simulation environment by using [pysphinx](https://developer.parrot.com/docs/sphinx/pysphinxapi.html).

1. Before being able to use it, you have to set the `LD_LIBRARY_PATH` and `PYTHONPATH` environment variables, by runnung in a *new terminal*:

       . /opt/parrot-sphinx/usr/bin/parrot-sphinx-setenv.sh

2. Then, in the *same terminal* run the ROS node

   * for ANAFI **Ai**:
      
         ros2 run anafi_ros_nodes sphinx --ros-args -r __ns:=/anafi -p drone_name:=anafi_ai

   * for ANAFI **4K**:
      
         ros2 run anafi_ros_nodes sphinx --ros-args -r __ns:=/anafi -p drone_name:=anafi

#### Ground truth pose

The ground truth pose is published on `drone/pose` topic at 1KHz.

#### Teleportation

It is possible to move the drone by publishing the desired pose on the `drone/move` topic.
