# Connect multiple drones

1. Connect Skycontrollers binded with Anafi drones to USB ports.

   Each Skycontroller will create a `192.168.53.0` network on the `usbX` interface.

3. In the terminal, run

       "$(ros2 pkg prefix anafi_ros_nodes)"/share/anafi_ros_nodes/setup_drones.sh number_drones

   where `number_drones` is the number of connected drones.

   > For example, if you have two connected Skycontrollers, run
   >
   >     "$(ros2 pkg prefix anafi_ros_nodes)"/share/anafi_ros_nodes/setup_drones.sh 2

   The script will create a new subnet with IP address `192.168.6X.0` for each `usbX` interface. All traffic on this address will be routed to the associated Skycontroller.

   > If you have two connected Skycontrollers, the Skycontroller connected on `usb0` interface will be asigned with `192.168.60.1` IP, while the Skycontroller connected on `usb1` interface will be asigned with `192.168.61.1` IP.
