/*
   TODO:
   -implement timeouts per: https://mavlink.io/en/services/mission.html
*/


#include <mavlink.h> // must have mavlink.h in same directory

unsigned long previousMillisMAVLink = 0;
unsigned long next_interval_MAVLink = 1000; // interval between heartbeats

uint8_t system_id = 255;
uint8_t component_id = 190;
uint8_t target_system = 1;
uint8_t target_component = 1;

int type = MAV_TYPE_FIXED_WING;
uint8_t autopilot_type = MAV_AUTOPILOT_INVALID; // No valid autopilot, e.g. a GCS or other MAVLink component
uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

void setup() {
  Serial1.begin(57600); // communication with PixHawk   19(RX), 18(TX)
  Serial.begin(9600); // debugging output
  Serial.println("starting up");
}

void loop() {
  //MISSION_CLEAR();
  //COMM_RECEIVE(MAVLINK_MSG_ID_MISSION_ACK);

  SEND_MISSION_COUNT(4); // send count
  COMM_RECEIVE(MAVLINK_MSG_ID_MISSION_REQUEST); // listen for request
  SET_WP(0, 0, 0, 0); // set home waypoint
  COMM_RECEIVE(MAVLINK_MSG_ID_MISSION_REQUEST); // listen for request
  SET_WP(1, 30, 50, 20);
  COMM_RECEIVE(MAVLINK_MSG_ID_MISSION_REQUEST); // listen for request
  SET_WP(2, 10, 40, 20);
  COMM_RECEIVE(MAVLINK_MSG_ID_MISSION_REQUEST); // listen for request
  SET_WP(3, -80, 150, 20);
  COMM_RECEIVE(MAVLINK_MSG_ID_MISSION_ACK); // listen for acknowledgement
  ACK_PACK();
  
  delay(1000);
}

void MISSION_CLEAR() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_clear_all_pack(system_id, component_id, &msg, target_system, target_component);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void SEND_MISSION_COUNT(int count) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_count_pack(system_id, component_id, &msg, target_system, target_component, count);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void TAKEOFF(int seq, float alt) {
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_TAKEOFF;
  uint8_t current = 0; // indicates a guided mode "go-to" message
  uint8_t autocontinue = 0; // Always 0
  float param1 = 0; // Loiter time
  float param2 = 0; // Acceptable range from target - radius in meters
  float param3 = 0; // Pass through waypoint
  float param4 = 0; // Desired yaw angle

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_item_pack(system_id, component_id, &msg, target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, 0, 0, alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

// lat:[-90,90] lon:[-180,180]
void SET_WP(uint16_t seq, float lat, float lon, float alt) {
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  uint16_t command = MAV_CMD_NAV_WAYPOINT;
  uint8_t current = 0; //2; // indicates a guided mode "go-to" message
  uint8_t autocontinue = 0;
  float param1 = 0; // Loiter time
  float param2 = 0; // Acceptable range from target - radius in meters
  float param3 = 0; // Pass through waypoint
  float param4 = 0; // Desired yaw angle

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_item_pack(system_id, component_id, &msg, target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, lat, lon, alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void RTL(int seq) {
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; // Set target frame to global default
  uint16_t command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
  uint8_t current = 0; // indicates a guided mode "go-to" message
  uint8_t autocontinue = 0; // Always 0
  float param1 = 0; // Loiter time
  float param2 = 0; // Acceptable range from target - radius in meters
  float param3 = 0; // Pass through waypoint
  float param4 = 0; // Desired yaw angle

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_item_pack(system_id, component_id, &msg, target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, 0, 0, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void ACK_PACK() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  mavlink_msg_mission_ack_pack(system_id, component_id, &msg, target_system, target_component, 0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void COMM_RECEIVE(int desiredID) {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();

    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      uint8_t receivedID = msg.msgid;

      //Serial.println(receivedID);

      //if (receivedID == desiredID) {
      switch (receivedID) {
        case MAVLINK_MSG_ID_HEARTBEAT:
          {
            mavlink_heartbeat_t hbt;
            mavlink_msg_heartbeat_decode(&msg, &hbt);

            //Serial.print("type: ");
            //Serial.println(hbt.type);
          }
          break;
        case MAVLINK_MSG_ID_MISSION_REQUEST:
          {
            mavlink_mission_request_t request;
            mavlink_msg_mission_request_decode(&msg, &request);

            Serial.println("--------MISSION_REQUEST--------");
            Serial.print("Sequence: ");
            Serial.println(request.seq);
            Serial.print("Target System: ");
            Serial.println(request.target_system);
            Serial.print("Target Component: ");
            Serial.println(request.target_component);
            Serial.println();
          }
          break;
        case MAVLINK_MSG_ID_MISSION_ACK: //MAVLINK_MSG_ID_MISSION_REQUEST_INT:
          {
            mavlink_mission_ack_t ack;
            mavlink_msg_mission_ack_decode(&msg, &ack);

            Serial.println("--------MISSION_ACK--------");
            Serial.print("Target System: ");
            Serial.println(ack.target_system);
            Serial.print("Target Component: ");
            Serial.println(ack.target_component);
            Serial.print("Type: ");
            Serial.println(ack.type);
            Serial.println();
          }
          break;
        default:
          break;
      }
      //}
    }
  }
}


void MAV_SEND_HEARTBEAT() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_heartbeat_pack(system_id, component_id, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}
