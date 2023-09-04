#!/bin/bash

if [[ $# -ne 1 || $1 == '-h' || $1 == '--help' ]]
then
	echo "
	Setups the connection with multiple drones.
	     
	USAGE:                                                                 
	  $0 ['-h' | '--help' | number_drones]
	  '-h' | '--help'   Prints this message.
	  number_drones     Setups the connection with 'number_drones' drones.
		                                                               
	E.g. $0 2 - will connect to two drones on usb0 and usb1 interfaces.
	"                                                                      
	exit 0                                                             
fi

if ! [[ $1 =~ ^[0-9]+$ && $1 -ge 0 ]]
then
	echo "The number of drones must be a positive numeric value."
	exit 1
fi

number_drones=$1

#dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
dir="$(ros2 pkg prefix anafi_ros_nodes)"/share/anafi_ros_nodes

for ((i=1; i<=$number_drones; i++))
do
	$dir/config_skycontroller_ip.sh clean usb$((i - 1)) 192.168.6$i.1 20$i
done

for ((i=1; i<=$number_drones; i++))
do
	$dir/config_skycontroller_ip.sh setup usb$((i - 1)) 192.168.6$i.1 20$i
done

for ((i=1; i<=$number_drones; i++))
do
	ros2 launch anafi_ros_nodes anafi_launch.py namespace:=anafi$i ip:=192.168.6$i.1 &
done

ros2 run rqt_reconfigure rqt_reconfigure

kill -9 $(pgrep -f reconfigure)
kill -9 $(pgrep -f anafi) &

for ((i=1; i<=$number_drones; i++))
do
	$dir/config_skycontroller_ip.sh clean usb$((i - 1)) 192.168.6$i.1 20$i
done

ros2 daemon stop
#ros2 daemon start
