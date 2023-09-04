# Simulation Environment

The simulation environment is based on [Parrot Sphinx](https://developer.parrot.com/docs/sphinx/index.html). It supports [ANAFI 4K](https://developer.parrot.com/docs/sphinx/supported_drones.html#anafi) and [ANAFI Ai](https://developer.parrot.com/docs/sphinx/supported_drones.html#anafi-ai).

> [!IMPORTANT]  
> Sphinx simulator does not allow running multiple drones.

## Installation

> [!IMPORTANT]  
> Sphinx simulator runs only on 64-bit **Ubuntu 20.04** and **Ubuntu 22.04**.

Follow the [installation procedure](https://developer.parrot.com/docs/sphinx/installation.html).

> [!WARNING]  
> Currently Sphinx simulator supports only kernel versions **< 6.2**.

So, the default supported distributions of Ubuntu are **22.04.2**, **22.04.1**, **22.04.0**, **20.04.5**, **20.04.4**, **20.04.3**, **20.04.2**.

> To check you kernel version, use
> 
>     uname -r
>
> To downgrade your kernel, follow [this guide](https://linux.how2shout.com/how-to-install-linux-kernal-5-19-on-ubuntu-22-04-or-20-04/).

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

4. *In terminal 3*, launch ROS bridge
  
   * for ANAFI **Ai**:
      
         ros2 launch anafi_ros_nodes anafi_launch.py ip:='10.202.0.1' model:='ai'

   * for ANAFI **4K**:
      
         ros2 launch anafi_ros_nodes anafi_launch.py ip:='10.202.0.1' model:='4k'

### Ground truth pose

