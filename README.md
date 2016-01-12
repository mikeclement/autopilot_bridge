# autopilot\_bridge : ROS bridge to autopilot protocols

Currently supports MAVLink-speaking autopilots,
particularly those running ArduPlane and the ardupilotmega message set.

## License

Written in 2015 by the Advanced Robotic Systems Engineering Laboratory at the U.S. Naval Postgraduate School, Monterey, California, USA.

Pursuant to 17 USC 105, this work is not subject to copyright in the United States and is in the public domain.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

## Requirements

First, you will need a working ROS installation. See here for details:
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

_Note: currently, this software is developed using ROS Indigo
and tested under both Indigo and Hydro._

While the full desktop version of ROS is recommended, the "base" install
should work for most cases. Some additional packages may be required,
such as sensor messages (ros-\*-sensor-msgs on an Ubuntu-esque system).

Second, you will need autopilot protocol-specific software.

For MAVLink-based autopilots, pymavlink (included in mavlink) is required:
https://github.com/mavlink/mavlink

For a user-specific install, run these commands:

```bash
git clone https://github.com/mavlink/mavlink.git
cd mavlink/pymavlink
python setup.py build install --user
```

Note that features in some modules (e.g., 'acs') rely on customized branches
of mavlink. However, the base code should always run with the master branch.

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

To run the MAVLink bridge (`mavbridge.py`), first start roscore in a separate terminal, then run:

```bash
rosrun autopilot_bridge mavbridge.py [options ...]
```

You can specify --help at the end of the command to see all options.
Generally, you will want to specify the device or network connection using `-d` or `--device`.
You can also specify the serial link speed using `-b` or `--baudrate` (the default is 57600 baud).
The following are all examples of legal device and baudrate syntax:

* `-d /dev/ttyS3 -b 115200`
* `-d /dev/ttyUSB*` (use the first matching device, if found, at 57600 baud)
* `-d /dev/ttyUSB1,115200` (overrides `-b`)
* `-d tcp:127.0.0.1:5762` (ignores `-b`)

If `-d` is omitted, mavbridge will attempt to find a serial port and use it.

If an appropriate device cannot be found on startup, mavbridge will fail and shut down. However, once up and running, mavbridge will detect a device disconnect or failure and attempt to reconnect using the same specification given at startup. Hence, if `-d /dev/ttyUSB*` is specified and mavbridge finds `/dev/ttyUSB0`, and then the USB serial device is momentarily disconnected and reappears as `/dev/ttyUSB1`, mavbridge _should_ detect and start using this. Of course, there are plenty of edge cases that are not handled; use wildcards with care!

Some additional options:

* `--ros-basename BASENAME` - Use a ROS basename other than "autopilot"
* `--looprate HZ` - Run the internal message-handling loop at `HZ` Hz (default 50)
* `--gps-time-hack` - Attempt to set the system clock from the autopilot. Requires root privileges
* `--serial-relief SIZE` - Don't get serial backlog grow past `SIZE` bytes
* `--spam-mavlink` - Print every _received_ MAVLink message in semi-friendly form to stdout.

## ROS Elements in mavbridge.py

When run as described above, `mavbridge.py` provides the following ROS elements:

* Publishers
  * `autopilot/gps` - GPS as a sensor\_msgs/NavSatFix message
  * `autopilot/gps_odom` - GPS as a nav\_msgs/Odometry message (position {x, y, z} replaced with {lat, lon, alt})
  * `autopilot/imu` - IMU / INS as sensor\_msgs/Imu message
* Subscribers
  * `autopilot/arm` - Arm or disarm the throttle (boolean)
  * `autopilot/guided_goto` - Go to a lat/lon/alt in GUIDED mode (all floats)
  * `autopilot/mode` - Change mode (string)
  * `autopilot/waypoint_goto` - Go to a programmed waypoint number (integer)

`mavbridge.py` supports modules for defining additional functionality.
Modules can be loaded with the `-m` or `--module` option.
For instance, to run mavbridge.py with the 'wp' module:

```bash
rosrun autopilot_bridge mavbridge.py -d /dev/ttyS3 -b 57600 -m wp
```

The `-m` option can be used multiple times, and `mavbridge.py` will report to stdout
whether each module is loaded successfully.

Following are brief descriptions of some of the commonly-used modules included in this package:

### wp - Waypoints

The 'wp' module provides the following additional ROS elements:

* Services
  * `autopilot/wp_getall` - Get all waypoints in the autopilot
  * `autopilot/wp_getrange` - Get a range of waypoints, by numeric index
  * `autopilot/wp_getlast` - Get the highest-indexed waypoint
  * `autopilot/wp_setall` - Clear and set all waypoints in the autopilot

### param - Parameters

The 'param' module provides the following additional ROS elements:

* Services
  * `autopilot/param_get` - Get a specific parameter by name (string)
  * `autopilot/param_getlist` - Get a set of parameters by name (array)
  * `autopilot/param_set` - Set a parameter by name and value (string, float)

**Note**: this module predates the 'fpr' module. It attempts to provide a more
complete implementation of the parameter fetching protocol; however, you may
alternatively use the newer `fpr_param_*` services described below.

### fpr - Fences, Parameters, and Rally points

The 'fpr' module provides a common framework for getting and setting parameters,
lists of fence points, and lists of rally points. It can be used in place of the
'param' module. It provides the following additional ROS elements:

* Services
 * `autopilot/fpr_fence_getall` - Get all fence points in the autopilot
 * `autopilot/fpr_fence_setall` - Clear and set all fence points in the autopilot
 * `autopilot/fpr_param_get` - Get a specific parameter by name
 * `autopilot/fpr_param_getlist` - Get a list of parameters by name
 * `autopilot/fpr_param_set` - Set a parameter by name and value (string, float)
 * `autopilot/fpr_param_setlist` - Set a list of parameters by name and value
 * `autopilot/fpr_rally_getall` - Get all rally points in the autopilot
 * `autopilot/fpr_rally_setall` - Clear and set all rally points in the autopilot

### file - Load configurations from files

The 'file' module provides services for loading files in the same formats used
by pymavlink and MAVProxy, with some limitations. It provides the following
additional ROS elements:

* Services
 * `autopilot/load_fence` - Load a fence file
 * `autopilot/load_param` - Load a parameter file
 * `autopilot/load_rally` - Load a rally file
 * `autopilot/load_wp` - Load a waypoint file

### slave - Ground station slaving

The 'slave' module allows "slaving" or "chaining" other MAVLink-speaking applications
(e.g., Mission Planner or MAVProxy) to the autopilot via mavbridge. This is similar to
MAVProxy's `--out` option, though slave channels in `mavbridge.py` can be configured
at runtime.

* Services
 * `autopilot/slave_setup` - Enable or disable a slave channel described by a string

Strings are those allowable in pymavlink (e.g., "/dev/ttyUSB0,57600" or "udp:127.0.0.1:1234").

See the service and message definitions in autopilot\_bridge for details
on specifying service requests and using responses.

### ap\_msg\_queue

The 'ap\_msg\_queue' module prvoides a means for storing messages such as STATUSTEXT from the autopilot for later forwarding.  The primary intent is to forward previously received autopilot messages to a ground control station, but only when requested.

* Services
 * `autopilot/ap_msg_queue_last_n` - Returns the last N message in the queue.  A second parameter is also provided: "since\_seq" which indicates the requestor only would like messages since the indicated sequence number, providing a rudimentary means to prevent forwarding messages the requestor already posseses.

## Extensibility in mavbridge.py

`mavbridge.py` supports a module system, which attempts to make adding ROS publishers,
subscribers, and services easy. It is intentionally designed to allow the writer
to choose either a procedural or an object-oriented style for each module.

Each module must reside in its own file, such that module 'foo' is in `mavbridge_foo.py`.
Inside the module, there must exist the following function:

```python
def init(bridge):
    ...
```

where 'bridge' is an instance of the MAVLinkBridge class in `MAVLinkBridge.py`.
`init()` can use this instance to register ROS subscriber and service handlers,
MAVLink message handlers, and timed event handlers. See `MAVLinkBridge.py` for
details, and the included modules for examples.
`MAVLinkBridge.py` also offers a number of 'helper' functions for modules, such
as managing the creation of ROS publishers (a publisher to a common topic
and type may be shared across modules), projection of autopilot time,
parsing autopilot sensor status bits, and running event handlers in their
own threads.

Note that (by convention only) publisher, subscriber, and service should
be prefixed with the module name; hence module 'foo' publisher 'bar'
should be registered as 'foo\_bar', and will show up in ROS as `autopilot/foo_bar`.

## Acknowledgements

This software was originally inspired by roscopter:
https://github.com/cberzan/roscopter

It relies heavily on pymavlink, the Python bindings for the MAVLink protocol:
https://github.com/mavlink/mavlink

Its implementation of several MAVLink commands and its modular extensions are heavily influenced by MAVProxy:
https://github.com/tridge/MAVProxy
