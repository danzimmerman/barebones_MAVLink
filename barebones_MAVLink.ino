/*  barebones_MAVLink.ino

  A Simple QGroundControl MAVLink Example
  
  MIT License
  
  Copyright (c) 2018 Daniel S. Zimmerman
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
  
 
*/

#include "./mavlink_c_v2/common/mavlink.h"
#include "math.h"

mavlink_message_t mvl_tx_message; //A special MAVLink message data structure. 
mavlink_message_t mvl_rx_message;
mavlink_status_t mvl_rx_status;
const uint8_t mvl_compid = 1; //Component ID and System ID identify us to QGroundControl
const uint8_t mvl_sysid = 1;
const uint8_t mvl_chan = MAVLINK_COMM_1;  //MAVLink channel 1 appears to be required at least for Blue Robotics QGC

//In my actual code for the Cypress PSoC, I use timer interrupts to schedule all the robot tasks.
const uint32_t hb_interval = 1000; //heartbeat interval in milliseconds - 1 second
uint32_t t_last_hb = 0;
const uint32_t sys_stat_interval = 100; //10 system status messages per second
uint32_t t_last_sys_stat =0;
int16_t sys_stat_count = 0;
uint8_t mvl_armed = 0;
uint8_t mvl_packet_recieved = 0;

/*==============================================================
 * Message-handling and transmitting functions 
 * used in the main loop are defined below.
 *==============================================================*/
 
void MVL_Transmit_Message(mavlink_message_t* mvl_msg_ptr)
{
  uint8_t tx_byte_buffer[512]={0}; //A byte buffer that will be sent from the serial port.
  uint16_t tx_buflen = mavlink_msg_to_send_buffer(tx_byte_buffer,mvl_msg_ptr);
  Serial.write(tx_byte_buffer,tx_buflen);
}

void MVL_Handle_Manual_Control(mavlink_message_t* mvl_msg_ptr)
{
  mavlink_manual_control_t mvl_joy; //manual control data structure into which we decode the message
  mavlink_msg_manual_control_decode(mvl_msg_ptr,&mvl_joy);
  //For now, let's just retransmit the manual control message to see it in MAVLink Inspector
  mavlink_msg_manual_control_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                         &mvl_tx_message,&mvl_joy);
  MVL_Transmit_Message(&mvl_tx_message);
}

void MVL_Handle_Param_Request_List(mavlink_message_t* mvl_msg_ptr)
{
  mavlink_param_value_t mvl_param;
  
  mvl_param.param_id[0] = 'a'; //a parameter ID string, less than 16 characters.
  mvl_param.param_value = 123.456; //the parameter value as a float
  mvl_param.param_type = MAV_PARAM_TYPE_REAL32; //https://mavlink.io/en/messages/common.html#MAV_PARAM_TYPE
  mvl_param.param_count = 1; //We have just one parameter to send. 
  mvl_param.param_index = 0; 
  mavlink_msg_param_value_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                      &mvl_tx_message,&mvl_param);
  MVL_Transmit_Message(&mvl_tx_message);
  
}

void MVL_Handle_Command_Long(mavlink_message_t* mvl_msg_ptr)
{
  mavlink_command_long_t mvl_cmd;
  mavlink_msg_command_long_decode(mvl_msg_ptr,&mvl_cmd);
  switch (mvl_cmd.command)
  {
    case (MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES):
    {
      if (1==mvl_cmd.param1)
      {
        mavlink_autopilot_version_t mvl_apv; https://mavlink.io/en/messages/common.html#AUTOPILOT_VERSION
        mvl_apv.flight_sw_version = 1;
        mvl_apv.middleware_sw_version = 1;
        mvl_apv.board_version = 1;
        mvl_apv.vendor_id = 10101;
        mvl_apv.product_id = 20202;
        mvl_apv.uid = 0;
        mvl_apv.capabilities = 0; //See: https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY
        mvl_apv.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET; //Just as an example, code does not support! https://mavlink.io/en/messages/common.html#MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET
        mavlink_msg_autopilot_version_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                                  &mvl_tx_message,&mvl_apv);
        MVL_Transmit_Message(&mvl_tx_message);
      }
      break;
    }//end handling of autopilot capabilities request
    case (MAV_CMD_COMPONENT_ARM_DISARM):
    {
      if (1==mvl_cmd.param1)
      {
        mvl_armed = 1;
      }
      else
      {
        mvl_armed = 0;
      }
      //Acknowledge the arm/disarm command.
      mavlink_command_ack_t mvl_ack; //https://mavlink.io/en/messages/common.html#COMMAND_ACK
      mvl_ack.command = MAV_CMD_COMPONENT_ARM_DISARM; //https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM
      mvl_ack.result = MAV_RESULT_ACCEPTED; //https://mavlink.io/en/messages/common.html#MAV_RESULT
      mavlink_msg_command_ack_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                          &mvl_tx_message,&mvl_ack);
      //skipped setting several fields here, with unknown consequences.
      MVL_Transmit_Message(&mvl_tx_message);
      break;
    }//end handling of arm/disarm command
  }//end switch/case
}//end MVL_Handle_Command_Long()

void MVL_Handle_Mission_Request_List(mavlink_message_t* mvl_msg_ptr)
{
  mavlink_mission_count_t mvl_mc;
  mvl_mc.target_system = mvl_sysid;
  mvl_mc.target_component = mvl_compid;
  mvl_mc.count = 0;
  mavlink_msg_mission_count_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                        &mvl_tx_message,&mvl_mc);
  MVL_Transmit_Message(&mvl_tx_message);
}

//=========================== Main program is below ======================================

void setup() 
{
  Serial.begin(115200); //start the serial port for the MAVLink packets.
}

void loop() 
{
  /* ======================== Serial MAVLink Message Reception ====================
   * If bytes come in on the serial port, we feed them to mavlink_parse_char().
   * This helper function keeps track of incoming bytes and alerts us when it's 
   * recieved a complete, valid MAVLink message.
   * See https://github.com/mavlink/c_library_v2/blob/master/mavlink_helpers.h#L966
   *==============================================================================*/
   
  while (Serial.available())
  {
    uint8_t rxbyte = Serial.read();
    mvl_packet_recieved = mavlink_parse_char(mvl_chan,rxbyte, 
                                             &mvl_rx_message,
                                             &mvl_rx_status);
  }
  
  /* ====================== Recieved MAVLink Message Handling ==================
   *  If a full incoming MAVLink message is recieved, AND the message 
   *  came from the GCS (System ID 255), we handle it here. 
   *  
   *  In this code we: 
   *  -Respond to initial messages sent from QGC to the vehicle when it first connects, including:
   *    -A parameter list request. We send an arbitary float parameter.
   *    -A request for "mission items." We say we have none.
   *    -A request for autopilot capabilities and version. We make some things up.
   *  -Take action on manual control joystick messages -- retransmitted back to QGC for viewing in MAVLink inspector
   *  -Arm or disarm the vehicle and acknowledge arm/disarm status based on incoming command messages.
   *  There may be more messages you want to handle. 
   *  The Message IDs below are defined in individual message's .h files.
   *  For example: https://github.com/mavlink/c_library_v2/blob/master/common/mavlink_msg_manual_control.h#L4
   *  You can find message numbers and field descriptions at https://mavlink.io/en/messages/common.html 
   * ==================================================================== */
  if ((mvl_packet_recieved) && (255==mvl_rx_message.sysid)) 
  {
    mvl_packet_recieved = 0; //reset the "packet recieved" flag
    switch (mvl_rx_message.msgid)
    {
      case MAVLINK_MSG_ID_MANUAL_CONTROL: //#69 https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
      {
        MVL_Handle_Manual_Control(&mvl_rx_message);
        break;
      }
      case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: //#21 https://mavlink.io/en/messages/common.html#PARAM_REQUEST_LIST
      {
        MVL_Handle_Param_Request_List(&mvl_rx_message);
        break;
      }
      case MAVLINK_MSG_ID_COMMAND_LONG: //#76 https://mavlink.io/en/messages/common.html#COMMAND_LONG
      {
        MVL_Handle_Command_Long(&mvl_rx_message);
        break;
      }
      case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: //#43 https://mavlink.io/en/messages/common.html#MISSION_REQUEST_LIST
      {
        MVL_Handle_Mission_Request_List(&mvl_rx_message);
        break;
      }
      
    }
    
   
  }

  /*=========================== Periodic Telemetry Transmissions ======================
   * To let QGroundControl know a vehicle is present, we send a heartbeat every 1s.
   * Here I'm using the millis() function to schedule outbound messages. 
   * You don't want to use blocking delay() calls in code like this, because you want
   * to read the incoming serial port as often as possible.
   * 
   * In my actual target system, a Cypress PSoC, I use timer interrupts for scheduling.
   * ==============================================================================*/
   
  if ((millis()-t_last_hb)>hb_interval)
  { //#0 HEARTBEAT https://mavlink.io/en/messages/common.html#HEARTBEAT
    mavlink_heartbeat_t mvl_hb; //struct with user fields: uint32_t custom_mode, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint8_t system_status;
    mvl_hb.type = MAV_TYPE_SUBMARINE; //My vehicle is an underwater ROV. Change as appropriate. See: https://github.com/mavlink/c_library_v2/blob/748192f661d0df3763501cfc432861d981952921/common/common.h#L69
    mvl_hb.autopilot = MAV_AUTOPILOT_GENERIC; //See https://github.com/mavlink/c_library_v2/blob/748192f661d0df3763501cfc432861d981952921/common/common.h#L40
    mvl_hb.system_status = MAV_STATE_ACTIVE;
    if (mvl_armed) 
    {
      mvl_hb.base_mode = MAV_MODE_MANUAL_ARMED;
    }
    else 
    { 
      mvl_hb.base_mode = MAV_MODE_MANUAL_DISARMED;
    }
    mvl_hb.base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED; //I always use CUSTOM_MODE_ENABLED
    mvl_hb.custom_mode=0xABBA; //custom mode, can be anything, I guess
    mavlink_msg_heartbeat_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                      &mvl_tx_message,&mvl_hb);
    MVL_Transmit_Message(&mvl_tx_message);
    t_last_hb = millis();  
  }

  if ((millis()-t_last_sys_stat)>sys_stat_interval)
  {
    
    //We'll make up some fake periodic data to feed to the QGroundControl widget
    float pfreq = 0.2; //slowly varying
    float phaserad = 2*PI*pfreq*millis()/1000.0;
    //Send the current phase angle in degrees times 100 as the voltage.
    //Voltage will ramp from 10000 to 16,000 mV over 60 sys_stat_intervals... 10-16V on the QGC widget
    int16_t angle_as_cV = sys_stat_count*100+10000;
    sys_stat_count+=1;
    sys_stat_count%=60;
    //I often use the mavlink_<message name>_pack_chan() functions that 
    //accept each field as an argument instead of the mavlink_<message name>_encode() that
    //accepts a struct. They should save some memory to skip the extra
    //message struct, but I think setting each field by name in a demo code is easier to follow.
    
    mavlink_sys_status_t mvl_sys_stat; //#1 SYS_STATUS https://mavlink.io/en/messages/common.html#SYS_STATUS
    mvl_sys_stat.onboard_control_sensors_present = 0; //To set these, consult https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR
    mvl_sys_stat.onboard_control_sensors_enabled = 0;
    mvl_sys_stat.onboard_control_sensors_health = 0;
    mvl_sys_stat.load = 0;
    mvl_sys_stat.voltage_battery = angle_as_cV; //the only non-trivial telemetry we're sending, shows up several places in QGC
    mvl_sys_stat.current_battery = -1;
    mvl_sys_stat.battery_remaining = -1;
    mvl_sys_stat.drop_rate_comm = 0;
    mvl_sys_stat.errors_comm = 0;
    mvl_sys_stat.errors_count1 = 0;
    mvl_sys_stat.errors_count2 = 0;
    mvl_sys_stat.errors_count3 = 0;
    mvl_sys_stat.errors_count4 = 0;
    
    //We'll also send an attitude quaternion to display something in the QGC
    //roll/pitch widget and in the compass.
    //The code below results in a gentle spherical rocking in the QGC roll/pitch widget
    //and a continuous rotation of the displayed heading.
    
    float maxang = 0.0873; // about five degrees
    float roll = maxang*sin(phaserad);
    float pitch = maxang*cos(phaserad);
    float yaw = phaserad;
    
    //Quaternion conversion taken from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
  
    mavlink_attitude_quaternion_t mvl_att_quat; //#31 ATTITUDE_QUATERNION https://mavlink.io/en/messages/common.html#ATTITUDE_QUATERNION
    mvl_att_quat.time_boot_ms = millis();
    mvl_att_quat.q1 = cy * cr * cp + sy * sr * sp; //https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code
    mvl_att_quat.q2 = cy * sr * cp - sy * cr * sp;
    mvl_att_quat.q3 = cy * cr * sp + sy * sr * cp;
    mvl_att_quat.q4 = sy * cr * cp - cy * sr * sp;
    
    mvl_att_quat.rollspeed = 2*PI*pfreq*maxang*cos(phaserad); //d/dt A*sin(2*pi*f*t) = 2*pi*f*A*cos(2*pi*f*t)
    mvl_att_quat.pitchspeed = -2*PI*pfreq*maxang*sin(phaserad);
    mvl_att_quat.yawspeed = 2*PI*pfreq; 
    
    mavlink_msg_sys_status_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                      &mvl_tx_message,&mvl_sys_stat);
    MVL_Transmit_Message(&mvl_tx_message);
    mavlink_msg_attitude_quaternion_encode_chan(mvl_sysid,mvl_compid,mvl_chan,
                                      &mvl_tx_message,&mvl_att_quat);
    MVL_Transmit_Message(&mvl_tx_message);
    t_last_sys_stat = millis();  
  }


} //end main loop


