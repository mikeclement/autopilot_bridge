# autopilot_bridge : ROS bridges to autopilot protocols

Currently supports mavlink-based autopilots

## Requirements

You will need mavlink (and pymavlink) installed:
https://github.com/mavlink/mavlink

For a user-specific install, run these commands on a clone:

	cd mavlink/pymavlink
	python setup.py build install --user

Note that some features in this code rely on custom MAVLink messages.
However, the code should always run with stock versions as well.

## Installation

From the base workspace folder, run

	catkin_make

## Usage

To run the bridge, first start roscore, then run

	rosrun autopilot_bridge mavlink.py

You can specify --help at the end of the command to see all options.

To run over serial device /dev/ttyS3 at 57600 baud:

	rosrun autopilot_bridge mavlink.py --device /dev/ttyS3 --baudrate 57600

To run over a TCP connection to a Simulation-In-The-Loop (SITL) instance:

	rosrun autopilot_bridge mavlink.py --device tcp:127.0.0.1:5762

