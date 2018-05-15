# A Minimal QGroundControl Connection Example

QGroundControl (http://qgroundcontrol.com/) is an open-source
"ground control station" for uncrewed aerial vehicles. Among many other features, it can display live-streaming inbound UDP video, provides attractive display of telemetry, and allows manual joystick remote control. It conveniently communicates with the vehicle controller using the [open MAVLink protocol](https://mavlink.io/).

However, I had a hard time finding clear documentation or a simple example of how to establish a MAVLink "connection" between a vehicle and [QGroundControl](http://qgroundcontrol.com/) outside of the context of a complicated UAV or UUV autopilot framework. 

This Arduino sketch uses a small subset of [common MAVLink messages](https://mavlink.io/en/messages/common.html) and auto-generated C headers (added to this project as a submodule) to negotiate a serial-port connection with QGroundControl.

This sketch is not necessarily useful by itself, but demonstrates the minimal MAVLink message handling to connect, send some status data to QGroundControl, and recieve control input from a [supported joystick](https://docs.qgroundcontrol.com/en/SetupView/Joystick.html#supported-joysticks) connected to the ground control computer. I tested this with the Logitech F710.
