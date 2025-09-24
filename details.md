# Package details

## Subscribed topics

| Topic Name 		| Message Type 				| Topic Description                                                  		|
| --------------------- | ------------------------------------- | ----------------------------------------------------------------------------- |
| `camera/command` 	| [`anafi_ros_interfaces/CameraCommand`](details.md#cameracommand) 	   | Camera zoom commands. |
| `drone/command` 	| [`anafi_ros_interfaces/PilotingCommand`](details.md#pilotingcommand) | Drone piloting commands. |
| `drone/moveby` 		| [`anafi_ros_interfaces/MoveByCommand`](details.md#movebycommand)	   | Move the drone by the given displacement and rotate by the given angle. |
| `drone/moveto` 		| [`anafi_ros_interfaces/MoveToCommand`](details.md#movetocommand) 	   | Move the drone to the specified location. |
| `gimbal/command` 	| [`anafi_ros_interfaces/GimbalCommand`](details.md#gimbalcommand) 	   | Gimbal attitude commands. |

## Published topics

| Topic Name			| Message Type 					| Frequency	| Values set/range 									| Topic Description 					| Units	|
| ----------------------------- | --------------------------------------------- | ------------- | ------------------------------------------------------------------------------------- | ----------------------------------------------- | ----- |
| `battery/health` | `std_msgs/UInt8` 				| 1 Hz 		| [`0`: bad, `100`: good] 									| Battery health 						| % 	| 
| `battery/percentage` | `std_msgs/UInt8` 				| 30 Hz 	| [`0`: empty, `100`: full] 								| Battery level 						| % 	| 
| `battery/voltage` | `std_msgs/Float32` 				| 1 Hz 		| 											| Battery voltage 					| V 	|
| `camera/awb_b_gain` | `std_msgs/Float32` 				| 30 Hz 	| 											| Camera automatic white balance (AWB) blue gain 			| 	|
| `camera/awb_r_gain` | `std_msgs/Float32` 				| 30 Hz 	| 											| Camera automatic white balance (AWB) red gain 				|	|
| `camera/camera_info` | `sensor_msgs/CameraInfo` 			| 30 Hz 	| 											| Main camera's info 						|	|
| `camera/exposure_time` | `std_msgs/Float32` 				| 30 Hz 	| 											| Exposure time of the main camera 					| s	| 
| `camera/image` 			| `sensor_msgs/Image` 				| 30 Hz 	| 											| Image from the main front camera 					|	|
| `camera/hfov` 			| `std_msgs/Float32` 				| 30 Hz 	| 											| Camera's horizontal field of view 					| º 	| 
| `camera/iso_gain` 		| `std_msgs/UInt16` 				| 30 Hz 	| 											| Camera's sensitivity gain 						|	|
| `camera/vfov` 			| `std_msgs/Float32` 				| 30 Hz 	| 											| Camera's vertical field of view 					| º 	| 
| `camera/zoom` 			| `std_msgs/Float32` 				| 5 Hz 		| 											| Camera zoom level 						| x 	| 
| `drone/altitude` 		| `std_msgs/Float32` 				| 30 Hz 	| > `0.0` 										| Drone's ground distance 					| m 	| 
| `drone/altitude_above_to` 	| `std_msgs/Float32` 				| 5 Hz 		| 											| Drone's ground distance above the take-off point 			| m 	| 
| `drone/attitude` 		| `geometry_msgs/QuaternionStamped` 		| 30 Hz 	| 											| Drone’s attitude in north-west-up frame 				|	|
| `drone/gps/fix` 		| `std_msgs/Bool` 				| 1 Hz 		| {`true`: GPS is fixed, `false`: GPS is not fixed} 					| 			| |
| `drone/gps/location` 		| `sensor_msgs/NavSatFix` 			| 1 Hz 		| 											| Drone’s GPS location 					|	|
| `drone/gps/satellites` 		| `std_msgs/UInt8` 				| 		| 											| Number of GPS satellites 					|	|
| `drone/rpy` 			| `geometry_msgs/Vector3Stamped` 			| 30 Hz 	| 											| Drone’s roll, pitch and yaw in north-west-up frame 			| º 	| 
| `drone/rpy_slow` 		| `geometry_msgs/Vector3Stamped` 			| 		| 											| Drone’s attitude 					|	|
| `drone/speed` 			| `geometry_msgs/Vector3Stamped` 			| 30 Hz 	| 											| Drone's speed in body frame 						| m/s 	| 
| `drone/state` 			| `std_msgs/String` 				| 30 Hz 	| {`CONNECTING`, `LANDED`, `TAKINGOFF`, `HOVERING`, `FLYING`, `LANDING`, `EMERGENCY`, `DISCONNECTED`} 	| Drone's state 						|	|
| `drone/steady` 			| `std_msgs/Bool` 				| 		| {`true`: the drone is steady, `false`: the drone is moving} 				| 			|
| `gimbal/attitude/absolute` 	| `geometry_msgs/Vector3Stamped` 		| 5 Hz 		| 											| Gimbal's attitude in north-west-up frame 				|	|
| `home/location` 		| `geometry_msgs/PointStamped` 			| 		| 											| Home location 					|	|
| `link/goodput` 			| `std_msgs/UInt16` 				| 30 Hz 	| 											| Connection throughput (available only for ANAFI 4K, Thermal and USA) 	| b/s 	|
| `link/quality` 			| `std_msgs/UInt8` 				| 30 Hz 	| [`0`: bad, `5`: good] 									| Link quality 					|
| `link/rssi` 			| `std_msgs/Int8` 				| 30 Hz 	| [`-100`: bad, `0`: good] 									| Signal strength (available only for ANAFI 4K, Thermal and USA) 	| dBm 	|
| `skycontroller/attitude` 	| `geometry_msgs/QuaternionStamped` 		| 20 Hz 	| 											| SkyController's attitude in north-west-up frame 				|	|
| `skycontroller/command` 	| [`anafi_ros_interfaces/SkycontrollerCommand`](details.md#skycontrollercommand) 	| 100 Hz 	| 											| Command from SkyController 					|
| `skycontroller/rpy` 		| `geometry_msgs/Vector3Stamped` 			| 20 Hz 	| 											| SkyController's attitude in north-west-up frame 				| º 	| 
| `storage/available` 		| `std_msgs/UInt64` 				| 		| 											| Available storage space 						| B 	| 
| `target/trajectory` 		| [`anafi_ros_interfaces/TargetTrajectory`](details.md#targettrajectory) 	| 		| 											| Target estimated trajectory 					|	|
| `time` 				| `std_msgs/Time` 				| 30 Hz 	| 											| Drone's local time 						|	|

## Services

| Service name 				| Service type 				| Service description 										|
| ------------------------------------- | ------------------------------------- | --------------------------------------------------------------------------------------------- |
| `camera/photo/stop` 			| [`anafi_ros_interfaces/srv/Photo`](details.md#photo) 	| stop photo capture 										|
| `camera/photo/take` 			| [`anafi_ros_interfaces/srv/Photo`](details.md#photo) 	| take a photo 											|
| `camera/recording/start`	 	| [`anafi_ros_interfaces/srv/Recording`](details.md#recording) 	| start video recording 									|
| `camera/recording/stop` 		| [`anafi_ros_interfaces/srv/Recording`](details.md#recording) 	| stop video recording 										|
| `camera/reset` 				| `std_srvs/srv/Trigger` 			| reset zoom level 										|
| `drone/arm` 				| `std_srvs/srv/SetBool` 			| {`true`: arm the drone, `false`: disarm the drone} 						|
| `drone/calibrate` 			| `std_srvs/srv/Trigger` 			| start drone's magnetometer calibration process 						|
| `drone/emergency` 			| `std_srvs/srv/Trigger` 			| cut out the motors 										|
| `drone/halt` 				| `std_srvs/srv/Trigger` 			| halt and start hovering 									|
| `drone/land` 				| `std_srvs/srv/Trigger` 			| land the drone 										|
| `drone/reboot` 				| `std_srvs/srv/Trigger` 			| reboot the drone 										|
| `drone/rth` 				| `std_srvs/srv/Trigger` 			| return home 											|
| `drone/takeoff` 			| `std_srvs/srv/Trigger` 			| take-off the drone 										|
| `flightplan/pause` 			| `std_srvs/srv/Trigger` 			| pause the flight plan 									|
| `flightplan/start` 			| [`anafi_ros_interfaces/srv/FlightPlan`](details.md#flightplan) 	| start the flight plan based on the Mavlink file existing on the drone 			|
| `flightplan/stop` 			| `std_srvs/srv/Trigger` 			| stop the flight plan 										|
| `flightplan/upload` 			| [`anafi_ros_interfaces/srv/FlightPlan`](details.md#flightplan) 	| upload the Mavlink file to the drone 								|
| `followme/start` 			| [`anafi_ros_interfaces/srv/FollowMe`](details.md#followme) 	| start follow-me 										|
| `followme/stop` 			| `std_srvs/srv/Trigger` 			| stop follow-me 										|
| `gimbal/calibrate` 			| `std_srvs/srv/Trigger` 			| start gimbal calibration 									|
| `gimbal/reset` 				| `std_srvs/srv/Trigger` 			| reset the reference orientation of the gimbal 						|
| `home/navigate` 			| `std_srvs/srv/SetBool` 			| {`true`: start return home, `false`: stop return home} trigger navigate home 			|
| `home/set` 				| [`anafi_ros_interfaces/srv/Location`](details.md#location) 	| set the custom home location 									|
| `POI/start` 				| [`anafi_ros_interfaces/srv/PilotedPOI`](details.md#pilotedpoi) 	| start the piloted point of interest 								|
| `POI/stop` 				| `std_srvs/srv/Trigger` 			| stop the piloted point of interest 								|
| `skycontroller/discover_drones` 	| `std_srvs/srv/Trigger` 			| find all visible drones 									|
| `skycontroller/forget_drone` 		| `std_srvs/srv/Trigger` 			| forget the connected drone 									|
| `skycontroller/offboard` 		| `std_srvs/srv/SetBool` 			| {`true`: switch to offboard control, `false`: switch to manual control} change control mode 	|
| `storage/download` 			| `std_srvs/srv/SetBool` 			| {`true`: delete media after download, `false`: otherwise} download media from the drone 		|
| `storage/format` 			| `std_srvs/srv/Trigger` 			| format removable storage 									|

## Parameters

| Parameter name 		| Type 		| Default value 	| Values set/range 								| Parameter description 		| Units 	|
| ----------------------------- | ------------- | --------------------- | ----------------------------------------------------------------------------- | ------------------------------------- | ------------- |
| `camera/autorecord` 		| `bool` 		| `false` 		| {`true`: enabled, `false`: disabled} 						| auto record at take-off 		| 		|
| `camera/disparity_image` 	| `bool` 		| `false` 		| {`true`: stream disparity map image, `false`: stream RGB image} 			| 					| 		|
| `camera/ev_compensation` 	| `int` 		| `9` 			| {`0`: -3.00, `3`: -2.00, `6`: -1.00, `9`: 0.00, `12`: 1.00, `15`: 2.00, `18`: 3.00} 	| camera exposure compensation 		| EV 		|
| `camera/hdr` 			| `bool` 		| `true` 			| {`true`: enabled, `false`: disabled} 						| high dynamic range (HDR) mode 	| 		|
| `camera/max_zoom_speed` 	| `float`		| `10.0` 			| [`0.1`, `10.0`] 									| maximum zoom speed 			| tan(º)/s	|
| `camera/mode` 			| `int` 		| `0` 			| {`0`: camera in recording mode, `1`: camera in photo mode} 			| camera mode 				| 		|
| `camera/stereo/disparity_map` 	| `bool` 		| `false` 			| {`true`: enabled, `false`: disabled} 			| stream disparity map image (supported only by ANAFI Ai) 				| 		|
| `camera/stereo/obstacle_avoidance` 	| `bool` 		| `true` 			| {`true`: enabled, `false`: disabled} 			| obstacle avoidance (supported only by ANAFI Ai) 				| 		|
| `camera/streaming`		| `int` 		| `0` 			| {`0`: minimize latency with average reliability (best for piloting), `1`: maximize reliability with an average latency, `2`: maximize reliability using a frame-rate decimation} 	| streaming mode 	| |
| `camera/style` 			| `int` 		| `0` 			| {`0`: natural look, `1`: flat and desaturated images, best for post-processing, `2`: intense - bright colors, warm shade, high contrast, `3`: pastel - soft colors, cold shade, low contrast} | images style 		| |
| `camera/thermal/rendering` 		| `int` 		| `0` 		| {`0`: visible, `1`: thermal, `2`: blended} 						| thermal image rendering mode (supported only by ANAFI Thermal and ANAFI USA) 						| |
| `device/ip` 	| `string` 	| `"192.168.53.1"`			| 										| IP address of the connected device 		| 		|
| `drone/banked_turn` 		| `bool` 		| `false` 			| {`true`: enabled, `false`: disabled} 						| banked turn 				| 		|
| `drone/camera_operated` 		| `bool` 		| `false` 		| {`true`: commands relative to the camera pitch, `false`: otherwise} 		| 					| 		| 
| `drone/max_altitude` 		| `float`		| `2.0` 			| [`0.5`, `4000.0`] 								| maximum altitude 			| m 		|
| `drone/max_distance` 		| `float` 	| `10.0` 			| [`10.0`, `4000.0`] 								| maximum distance 			| m 		| 
| `drone/max_horizontal_speed` 	| `float` 	| `1.0` 			| [`0.1`, `15.0`] 									| maximum horizontal speed 		| m/s 		| 
| `drone/max_pitch_roll` 		| `float` 	| `10.0` 			| [`1.0`, `40.0`] 									| maximum pitch and roll angle 		| º 		| 
| `drone/max_pitch_roll_rate` 	| `float` 	| `300.0` 		| [`40.0`, `300.0`] 								| maximum pitch and roll rotation speed | º/s 		| 
| `drone/max_vertical_speed` 	| `float` 	| `1.0` 			| [`0.1`, `4.0`] 									| maximum vertical speed 		| m/s 		| 
| `drone/max_yaw_rate` 		| `float` 	| `180.0` 		| [`3.0`, `200.0`] 									| maximum yaw rotation speed 		| º/s 		| 
| `drone/model` 			| `string` 	| 			| {`"4k"`, `"thermal"`, `"usa"`, `"ai"`} 					| drone's model 			| 		|
| `drone/name` 			| `string` 	| 			| `"[drone/model]_[device/ip]"` 					| drone's name 			| 		|
| `drone/offboard` 			| `bool` 	| `false`			| {`true`: offboard control, `false`: manual control} 					| enable offboard control mode 			| 		|
| `drone/serial` 			| `string` 	| 			|  					| drone's serial number 			| 		|
| `gimbal/max_speed` 		| `float` 	| `180.0` 		| [`1.0`, `180.0`] 									| maximum gimbal speed 			| º/s 		| 
| `home/autotrigger` 		| `bool` 		| `true` 			| {`true`: enabled, `false`: disabled} 						| auto trigger return-to-home 		| 		|
| `home/ending_behavior` 		| `int` 		| `1` 			| {`0`: land, `1`: hover} 								| return-to-home ending behavior 	| 		|
| `home/hovering_altitude` 	| `float` 	| `10.0` 			| [`1.0`, `10.0`] 									| return-to-home ending hovering altitude | m 		| 
| `home/min_altitude` 		| `float` 	| `20.0` 			| [`20.0`, `100.0`] 								| return-to-home minimum altitude 	| m 		| 
| `home/precise` 			| `bool` 		| `true` 			| {`true`: enabled, `false`: disabled} 						| precise return-to-home 		| 		|
| `home/type` 			| `int` 		| `4` 			| {`1`: take-off location, `3`: user-set custom location, `4`: pilot location} 	| home type for return-to-home 		| 		|
| `link/wifi_key` 			| `string` 	| 			|  					| WiFi key for the drone's access point (for direct connection to the drone) 			| 		|
| `storage/download_folder` 	| `string` 	| `"~/Pictures/Anafi"` 	| 										| path to the download folder 		| 		|
| `storage/rest_api_version` 	| `int` 	| `1` 	| 										| version of the REST API 		| 		|

## Custom messages

#### CameraCommand
- *Header* **header**: header of the message
- *uint8* **mode**: control mode {`0`: level, `1`: velocity}
- *float32* **zoom**: zoom command (x | x/s)
#### GimbalCommand
- *Header* **header**: header of the message
- *uint8* **mode**: control mode {`0`: position, `1`: velocity}
- *uint8* **frame**: gimbal's frame of reference {`0`: none, `1`: relative, `2`: absolute}
- *float32* **roll**: roll command (º | º/s)
- *float32* **pitch**: pitch command (º | º/s)
- *float32* **yaw**: pitch command (º | º/s)
#### MoveByCommand
- *Header* **header**: header of the message
- *float32* **dx**: x displacement (m)
- *float32* **dy**: y displacement (m)
- *float32* **dz**: z displacement (m)
- *float32* **dyaw**: yaw displacement (º)
#### MoveToCommand
- *Header* **header**: header of the message
- *float64* **latitude**: latitude (º)
- *float64* **longitude**: longitude (º)
- *float64* **altitude**: altitude (m)
- *float32* **heading**: heading w.r.t. North (º)
- *uint8* **orientatio_mode**: orientation mode {`0`: none, `1`: to target, `2`: heading start, `3`: heading during}
#### PilotingCommand
- *Header* **header**: header of the message
- *float32* **roll**: roll angle (º)
- *float32* **pitch**: pitch angle (º)
- *float32* **yaw**: yaw rate (º/s)
- *float32* **gaz**: vertical velocity (m/s)
#### SkycontrollerCommand
- *Header* **header**: header of the message
- *int8* **x**: x-axis [`-100`, `100`] (%)
- *int8* **y**: y-axis [`-100`, `100`] (%)
- *int8* **z**: z-axis [`-100`, `100`] (%)
- *int8* **yaw**: yaw-axis [`-100`, `100`] (%)
- *int8* **camera**: camera-axis [`-100`, `100`] (%)
- *int8* **zoom**: zoom-axis [`-100`, `100`] (%)
- *bool* **return_home**: return-to-home (front top) button {`true`: pressed, `false`: not pressed}
- *bool* **takeoff_land**: take-off/land (front bottom) button {`true`: pressed, `false`: not pressed}
- *bool* **reset_camera**: reset camera (back left) button {`true`: pressed, `false`: not pressed}
- *bool* **reset_zoom**: reset zoom (back right) button {`true`: pressed, `false`: not pressed}
#### TargetTrajectory
- *Header* **header**: header of the message
- *float64* **latitude**: target latitude (º)
- *float64* **longitude**: target longitude (º)
- *float32* **altitude**: target altitude (m)
- *float32* **north_speed**: target north speed (m/s)
- *float32* **east_speed**: target east speed (m/s)
- *float32* **down_speed**: target down speed (m/s)

## Custom services

#### FlightPlan
- *string* **file** path to the flight plan file on local computer
- *string* **uid** flight plan UID in drone's directory
#### FollowMe
- *uint8* **mode** follow me mode {`0`: none, `1`: look at, `2`: geographic, `3`: relative, `4`: leash}
- *int8* **horizontal** horizontal position in the video from left to right (%)
- *int8* **vertical** vertical position in the video from bottom to top (%)
- *float32* **target_azimuth** horizontal north-drone-target angle (rad)
- *float32* **target_elevation** vertical angle horizon-drone-target (rad)
- *float32* **change_of_scale** normalized relative radial speed (1/s)
- *uint8* **confidence_index** confidence index of the detection [`0`: worst, `255`: best]
- *bool* **is_new_selection** selection is new {`true`: new selection, `false`: old selection}
#### Location
- *float64* **latitude** latitude (º)
- *float64* **longitude** longitude (º)
- *float64* **altitude** altitude (m)
#### PilotedPOI
- *float64* **latitude** latitude to look at (º)
- *float64* **longitude** longitude to look at (º)
- *float64* **altitude** altitude to look at (m)
- *bool* **locked_gimbal** gimbal is locked {`true`: gimbal is locked on the point of interest, `false`: gimbal is freely controllable}
#### Photo
- *uint8* **mode** photo mode {`0`: single shot, `1`: bracketing - burst of frames with a different exposure, `2`: burst of frames, `3`: time-lapse - frames at a regular time interval, `4`: GPS-lapse - frames at a regular GPS position interval}
- *uint8* **photo_format** photo format {`0`: full resolution, not dewarped, `1`: rectilinear projection, dewarped}
- *uint8* **file_format** file format {`0`: jpeg, `1`: dng, `2`: jpeg and dng}
- *string* **media_id** media id
#### Recording
- *uint8* **mode** video recording mode {`0`: standard, `1`: hyperlapse, `2`: slow motion, `3`: high-framerate}
- *string* **media_id** media id
