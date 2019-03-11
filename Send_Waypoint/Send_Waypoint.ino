#include <mavlink.h>
#include <SoftwareSerial.h>

uint8_t system_id = 2;
uint8_t component_id = 0;
uint8_t target_system = 0;
uint8_t target_component = 0;

SoftwareSerial SerialMAV(19, 18);

void setup() {
  Serial.begin(9600); // debugging
  //Serial1.begin(57600); // communication with PixHawk   19(RX), 18(TX)
  SerialMAV.begin(57600);
}

void loop() {
  //MAV_MISSION_CLEAR();
  //comm_receive();
  
  //MAV_MISSION_COUNT();
  //MAV_SET_WP(100.0,10.0,10.0);

  //delay(1000);

  //setMode();
  //setGuided();
  //setWP();

  //mav_set_mode();
  mav_arm_pack(1);
}

void MAV_MISSION_CLEAR() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_clear_all_pack(system_id, component_id, &msg, target_system, target_component);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  Serial.println(SerialMAV.available());
  while (SerialMAV.available() > 0) {
    uint8_t c = SerialMAV.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message
      Serial.println("Receiving message id: " + msg.msgid);
      switch (msg.msgid) {
        case  47: //MISSION_ACK
          {
            mavlink_mission_ack_t mission_ack;
            mavlink_msg_mission_ack_decode(&msg, &mission_ack);

            Serial.print("target system:");
            Serial.println(mission_ack.target_system);
            Serial.print("target component:");
            Serial.println(mission_ack.target_component);
            Serial.print("type:");
            Serial.println(mission_ack.type);
          }
          break;

        case  40: //MISSION_REQUEST
          {
            Serial.println("req");
          }
          break;
        default:
          break;
      }
    }
  }
}

void MAV_MISSION_COUNT() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t count = 1;

  mavlink_msg_mission_count_pack(system_id, component_id, &msg, target_system, target_component, count);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

// implement this to see if arduino is requesting
/*
  void MAV_MISSION_REQUEST() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint8_t count = 1;

  mavlink_msg_mission_request_int_decode(&msg, );

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
  }
*/

void MAV_SET_WP(float x, float y, float z)
{

  uint16_t seq = 0; // Sequence is always set to 0
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_WAYPOINT; // 16
  uint8_t current = 2; // indicates a guided mode "go-to" message
  uint8_t autocontinue = 0; // Always 0
  float param1 = 0; // Loiter time
  float param2 = 1; // Acceptable range from target - radius in meters
  float param3 = 0; // Pass through waypoint
  float param4 = 0; // Desired yaw angle


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  // NOTE : _target_systen & _target_component are defined in setup
  // found by searching mission item .h from mavlink library
  mavlink_msg_mission_item_pack(system_id, component_id, &msg, target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z);
  //mavlink_msg_command_long_pack(system_id, component_id, &msg, target_system, target_component, command, 1, 10, 10, 10, 0, x, y, z);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void setMode() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(system_id, component_id, &msg, target_system, target_component, MAV_CMD_DO_SET_MODE, 1, 1, 0, 0, 0, 0, 0, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void setWP() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(system_id, component_id, &msg, target_system, target_component, MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 100, 100, 100);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void setGuided() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(system_id, component_id, &msg, target_system, target_component, 92, 1, 1, 0, 0, 0, 0, 0, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void mav_set_mode() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  //Set flight mode 'Stabilize'
  mavlink_msg_set_mode_pack(0xFF, 0xBE, &msg, 1, 209, 3); // last parameter = flight mode
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}

void mav_arm_pack(boolean state) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  //Arm the drone
  //400 stands for MAV_CMD_COMPONENT_ARM_DISARM
  // 1 an 8'th argument is for ARM (0 for DISARM)
  if (state) {
    //ARM
    //mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 400, 1,1.0,0,0,0,0,0,0);
    mavlink_msg_command_long_pack(system_id, component_id, &msg, target_system, target_component, 400, 1, 1.0, 0, 0, 0, 0, 0, 0);
  } else {
    //DISARM
    mavlink_msg_command_long_pack(0xFF, 0xBE, &msg, 1, 1, 400, 1, 0.0, 0, 0, 0, 0, 0, 0);
  }
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  SerialMAV.write(buf, len);
}
