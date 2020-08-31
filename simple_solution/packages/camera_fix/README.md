# Relay node
Made as a workaround for converting Gazebo generated image messages (Z-Y frame) into ros conventions (X-Y frame)
- Creates a new frame according to ros conventions
- Takes the sensor_msgs::Image and the sensor_msgs::CameraInfo, changes the frame_id to be the new frame, and publishes them at `name/prefix/fixed/`

Usage `roslaunch camera_fix camera_tf_fix.launch name:="X1" prefix:="front"`
