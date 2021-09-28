# README
This is a simple ROS package that allows starting and stopping rosbag recordings via service calls, which is useful for performing repeated data collection.

## How to set config
The config for the recording is in config/parameter.yaml.
Configure the output directory for the rosbag files and topics to be recorded (refer to http://wiki.ros.org/rosbag/Commandline).
Rosbag files are saved with the current date and time as a filename.

## How to run node
`roslaunch bag_recorder run.launch`

The package creates four services, which can be called with a `std_srvs.srv.Trigger` and `bag_recorder.srv.saveDirectory` message:
* /bag_recorder/start\_recording
* /bag_recorder/stop\_recording
* /bag_recorder/toggle\_recording
* /bag_recorder/save_directory "path"
