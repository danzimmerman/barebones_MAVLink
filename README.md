# A Minimal QGroundControl MAVLink Example

[QGroundControl](http://qgroundcontrol.com/) is an open-source
"ground control station" for uncrewed aerial vehicles. Among many other features, it's an all-in-one center for displaying live-streaming inbound UDP video, attractive display of vehicle telemetry, and manual joystick remote control. It conveniently communicates with the vehicle controller using the [open MAVLink protocol](https://mavlink.io/). These features make it attractive for remote human-piloted teleoperation of simple robots. 

However, I had a hard time finding clear documentation or a simple example of how to establish a MAVLink "connection" between a vehicle and [QGroundControl](http://qgroundcontrol.com/) outside of the context of a complicated UAV or UUV autopilot framework. 

This Arduino sketch uses a small subset of [common MAVLink messages](https://mavlink.io/en/messages/common.html) and auto-generated C headers (added to this project as a submodule) to negotiate a serial-port connection with QGroundControl. The Arduino will then send some simple telemetry to QGroundControl and respond to manual control packets from QGC if a [supported joystick](https://docs.qgroundcontrol.com/en/SetupView/Joystick.html#supported-joysticks) is connected to the ground control computer. I tested this with the Logitech F710.

This sketch is not necessarily that useful by itself, nor is it robust and well-tested. If you are working with a quadcopter, boat, or submarine, or wheeled rover, using one of the appropriate autopilot software offerings would probably be more appropriate than working with MAVLink messages directly. I worked this out because I wanted to tack on MAVLink manual control and telemetry to a previously-developed controller for a robot that's very different from vehicles covered by existing autopilots.

## This is a work in progress.
