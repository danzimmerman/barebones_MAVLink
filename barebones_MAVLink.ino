/*  minimal_MAVLink_QGC_teleop.ino

   QGroundControl (http://qgroundcontrol.com/) is an open-source
   "ground control station" for uncrewed aerial vehicles.

   Among many other features, it can display live-streaming inbound UDP video,
   provides attractive display of telemetry, and allows manual joystick
   remote control. It conveniently communicates with the
   vehicle controller using the open MAVLink protocol (https://mavlink.io/).

   MAVLink libraries for many programming languages are readily available, and
   can be custom-generated using the mavgenerate.py software tool.

   These features make QGroundControl an attractive interface for purely teleoperated
   robots and simple remote-controlled vehicles. However, QGroundControl and MAVLink 
   are generally used as parts of a full software stack for UAVs, and the commonly-used
   vehicle-side software is a complex autopilot.

   Many features of the QGroundControl interface are completely inaccessible without
   a "connection" to the vehicle and proper vehicle configuration. For example, 
   QGroundControl will only present a joystick interface if the vehicle properly 
   informs QGroundControl that it has manual control capability.
   
   I had a hard time finding clear documentation or a simple example on how
   to establish a MAVLink "connection" between a vehicle and QGroundControl
   outside of the context of an existing autopilot. Over time, I worked it out
   and wanted to contribute a clear, bare-bones example.
 
   This code uses a ~very~ small subset of common MAVLink messages to negotiate a 
   serial-port connection with QGroundControl.

   QGroundControl is overkill for basic remote control, so this sketch running on an Arduino is probably not very useful
   on its own. But the addition of single-board computer running MAVProxy can handle message 
   forwarding from a serial port to a Wifi and at the same time can be set up with gstreamer to stream live
   video from an inexpensive USB webcam. Furthermore, the handling of recieved MAVLink messages can be used 
   to add auxiliary functionality to an existing vehicle that uses a QGroundControl/MAVLink/autopilot stack.

   If you wish to expand this code, the rest of the common MAVLink messages are explained at https://mavlink.io/en/messages/common.html
   
   This code licensed under the MIT license: https://opensource.org/licenses/MIT

   Copyright 2018 Daniel Zimmerman

   Permission is hereby granted, free of charge, to any person obtaining a copy of this software
   and associated documentation files (the "Software"), to deal in the Software without restriction,
   including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
   do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all copies
   or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
   INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
   PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
   FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
   TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
   OTHER DEALINGS IN THE SOFTWARE.
*/

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
