# A Minimal QGroundControl Connection Example

QGroundControl (http://qgroundcontrol.com/) is an open-source
"ground control station" for uncrewed aerial vehicles. Among many other features, it can display live-streaming inbound UDP video, provides attractive display of telemetry, and allows manual joystick remote control. It conveniently communicates with the vehicle controller using the [open MAVLink protocol](https://mavlink.io/).

However, I had a hard time finding clear documentation or a simple example of how to establish a MAVLink "connection" between a vehicle and [QGroundControl](http://qgroundcontrol.com/) outside of the context of a complicated UAV or UUV autopilot framework. 

This Arduino sketch uses a small subset of [common MAVLink messages](https://mavlink.io/en/messages/common.html) and auto-generated C headers (added to this project as a submodule) to negotiate a serial-port connection with QGroundControl. It will then send some simple telemetry to QGroundControl and allow manual control using a [supported joystick](https://docs.qgroundcontrol.com/en/SetupView/Joystick.html#supported-joysticks) connected to the ground control computer. I tested this with the Logitech F710.

This sketch is not necessarily that useful by itself, nor is it robust and well-tested. If you are working with a quadcopter, boat, or submarine, or wheeled rover, I wouldn't reinvent the wheel, using one of the appropriate autopilot software offerings would probably be more appropriate than working with MAVLink messages directly. I worked this out because I wanted to add MAVLink manual control and telemetry to an robot prototype that's very different from any vehicle covered by the typical autopilots.

## This is a work in progress.
