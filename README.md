# autopilot\_bridge : ROS bridges to autopilot protocols

Currently supports mavlink-based autopilots

## Requirements

First, you will need a working ROS installation. See here for details:
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

Note: currently, this software is developed using ROS Hydro.

While the full desktop version of ROS is recommended, the "base" install
should work for most cases. Some additional packages may be required,
such as sensor messages (ros-hydro-sensor-msgs on an Ubuntu-esque system).

Second, you will need autopilot protocol-specific software.

For MAVLink-based autopilots, mavlink (and pymavlink) is required:
https://github.com/mavlink/mavlink

For a user-specific install, run these commands on a clone:

```bash
cd mavlink/pymavlink
python setup.py build install --user
```

Note that features in some modules (e.g. acs) rely on customized branches
of MAVLink. However, the base code should always run with the master branch.

## Installation

You will want to follow the typical steps for creating a ROS workspace
(see http://wiki.ros.org/catkin/Tutorials/create_a_workspace for details).

In summary:

```bash
mkdir -p ~/my_workspace/src
cd ~/my_workspace/src
catkin_init_workspace
cd ..
catkin_make
source devel/setup.bash  # You may wish to add this to ~/.bashrc
```
        
Then, clone this software into the workspace:

```bash
cd my_workspace/src
git clone url_to_autopilot_bridge
```

Finally:

```bash
cd ~/my_workspace
catkin_make
```

## Usage

To run the MAVLink bridge (mavbridge.py), first start roscore in a separate terminal, then run:

```bash
rosrun autopilot_bridge mavbridge.py [options ...]
```

You can specify --help at the end of the command to see all options.
Generally, you will want to specify the device or network connection using -d or --device.
If omitted, mavbridge will attempt to autodetect a serial device.

To run over serial device /dev/ttyS3 at 57600 baud:

```bash
rosrun autopilot_bridge mavbridge.py --device /dev/ttyS3 --baudrate 57600
```

To run over a TCP connection to a Simulation-In-The-Loop (SITL) instance running
locally on port 5762:

```bash
rosrun autopilot_bridge mavbridge.py --device tcp:127.0.0.1:5762
```

Some additional options:

* `--ros-basename BASENAME` - Use a ROS basename other than "autopilot"
* `--gps-time-hack` - After the bridge first connects, attempt to set the system clock from the autopilot. Requires root privileges.
* `--track-time-delta` - Use autopilot time messages to maintain a "delta" between the local and autopilot clocks; use this when publishing ROS messages (see MAVLinkBridge.py:project\_ap\_time()).
* `--spam-mavlink` - Print every _received_ MAVLink message in semi-friendly form to stdout.

## ROS Elements in mavbridge.py

When run as described above, mavbridge.py provides the following ROS elements:

* Publishers
  * `/autopilot/gps` - GPS as a sensor\_msgs/NavSatFix message
  * `/autopilot/gps_odom` - GPS as a nav\_msgs/Odometry message
  * `/autopilot/imu` - IMU / INS as sensor\_msgs/Imu message
* Subscribers
  * `/autopilot/arm` - Arm or disarm the throttle (boolean)
  * `/autopilot/guided_goto` - Go to a lat/lon/alt in GUIDED mode (all floats)
  * `/autopilot/mode` - Change mode (string)
  * `/autopilot/waypoint_goto` - Go to a programmed waypoint number (integer)

There are additional modules that provide extra functionality (see below for more details).
Of interest are the 'param' (Parameter), 'wp' (Waypoint), and 'slave' modules.
Modules can be loaded with the -m or --module option. For instance, to run mavbridge.py
with the 'param' module:

```bash
rosrun autopilot_bridge mavbridge.py --device /dev/ttyS3 --baudrate 57600 --module param
```

mavbridge.py will report which modules are loaded successfully (or not).

The 'param' module provides the following additional ROS elements:

* Services
  * `/autopilot/param_get` - Get a specific parameter by name
  * `/autopilot/param_getlist` - Get a set of parameters by name (array)
  * `/autopilot/param_set` - Set a parameter by name and value (string, float)

The 'wp' module provides the following additional ROS elements:

* Services
  * `/autopilot/wp_getall` - Get all waypoints in the autopilot
  * `/autopilot/wp_setall` - Clear and set all waypoints in the autopilot

The 'slave' module allows connecting slave MAVLink applications (e.g., MAVProxy)
to mavbridge. Slave channels are configurable at runtime using the
`/autopilot/slave_setup` service, which takes a boolean value ('enable')
and a string describing the channel (e.g., "/dev/ttyUSB0,57600" or
"udp:127.0.0.1:1234").

See the service and message definitions in autopilot\_bridge for details
on specifying service requests and using responses.

## Extensibility in mavbridge.py

mavbridge supports a module system, which attempts to make adding ROS publishers,
subscribers, and services easy. It is intentionally designed to allow the writer
to choose either a procedural or an object-oriented style for each module.

Each module must reside in its own file, such that module 'foo' is in mavbridge\_foo.py.
Inside the module, there must exist the following function:

```python
def init(bridge):
    ...
```

where 'bridge' is an instance of the MAVLinkBridge class in MAVLinkBridge.py.
init() can use this instance to register ROS subscriber and service handlers,
MAVLink message handlers, and timed event handlers. See MAVLinkBridge.py for
details, and the included modules for examples.
Note that (by convention only) publishers, subscribers, and services ought
to be prefixed with the module name; hence module 'foo' publisher 'bar'
would become 'foo\_bar'.

MAVLinkBridge also offers a number of 'helper' functions for modules, such
as managing the creation of ROS publishers (a publisher to a common topic
and type may be shared across modules), projection of time stamps, and 
parsing autopilot sensor status.

## Acknowledgements

This software was originally inspired by roscopter:
https://github.com/cberzan/roscopter

Its implementation of several MAVLink commands and its modular extensions are heavily influenced by MAVProxy:
https://github.com/tridge/MAVProxy

