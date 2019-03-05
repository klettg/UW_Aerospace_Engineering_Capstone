#include <mavlink.h>

uint8_t system_id = 2;
uint8_t component_id = 200;
uint8_t target_system = 1;
uint8_t target_component = 0;

void setup() {
  Serial.begin(9600); // debugging
  Serial1.begin(57600); // communication with PixHawk   19(RX), 18(TX)
}

void loop() {
  MAV_SET_WP(1.0,2.0,3.0);
  setMode(0);
  delay(5000);
}

void MAV_SET_WP(float x, float y, float z)
{
  
  uint16_t seq = 0; // Sequence is always set to 0
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_WAYPOINT; // Specific command for APM as it uses different paramters than the standard MAVLINK implementation
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

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  delay(1000);
  Serial.println("sending waypoint");
  Serial1.write(buf, len);
}

void setMode(int mode)
{
 
  uint16_t command = MAV_CMD_DO_SET_MODE; // Specific command for APM as it uses different paramters than the standard MAVLINK implementation
  uint8_t confirmation = 0; // First transmission of this command
  float param1 = 0; // mode = stabilize
  float param2 = 0;
  float param3 = 0;
  float param4 = 0;
  float param5 = 0;
  float param6 = 0;
  float param7 = 0;


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  // NOTE : _target_systen & _target_component are defined in setup
  // found by searching mission item .h from mavlink library
  mavlink_msg_command_long_pack(system_id, component_id, &msg, target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  delay(1000);
  Serial.println("setting mode");
  Serial1.write(buf, len);
}
