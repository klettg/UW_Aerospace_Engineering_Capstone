/*
    TODO:
    -are autocontinue and current set correctly?
    -verify altitude readings
    -implement message timeouts per: https://mavlink.io/en/services/mission.html
    -implement timeout for commands
*/

#include <mavlink.h> // must have mavlink.h in same directory

uint8_t system_id = 255;
uint8_t component_id = 190;
uint8_t target_system = 1;
uint8_t target_component = 1;

float curr_lat = 0;
float curr_lon = 0;
float curr_alt = 0; // user defined alt instead?

#define loiter_radius 30 // radius at which to loiter (meters)
#define loiter_time 15 // time to loiter while waiting for next waypoint (seconds)

// see defines.h
#define AUTO 10
#define STABILIZE 2

void setup() {
  Serial1.begin(57600); // communication with PixHawk   19(RX), 18(TX)
  Serial.begin(9600); // debugging output
  Serial.println("starting up");
}

void loop() {
  if (!update_gps_pos()) {
    // failsafe
  }

  uint32_t mode = get_flight_mode();
  while (mode != AUTO) {
    if (mode == 100) {
      // failsafe
    }
    if (!update_gps_pos()) {
      // failsafe
    }
    send_next_mission_item(curr_lat, curr_lon, curr_alt);
    if (mode == STABILIZE) {
      //start data collection if not already started
    }
    mode = get_flight_mode();
  }

  // we are in auto -> start data collection

  // listen for item completed message (finished first orbit)
  // we are assumming that the plane sends the loiter item when it
  // finishes the loiter. it may send it when it begins it in
  // which case we want to check for seq = 2, meaning it has moved
  // on to the timed "waiting" loiter
  // maybe change autocontinue/current?
  while (get_flight_mode() == AUTO) {
    unsigned long curr_time = millis();
    while (get_reached_item() != 1) { // seq = 1
      // wait, collect data
      // implement failsafe
    }
    // send next waypoint
  }

  // not in auto -> end data collection

}

uint32_t get_flight_mode() {
  mavlink_message_t msg;
  mavlink_status_t status;

  unsigned long curr_time = millis();
  while (millis() - curr_time < 3000) {
    while (Serial1.available() > 0) {
      uint8_t c = Serial1.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          mavlink_heartbeat_t hbt;
          mavlink_msg_heartbeat_decode(&msg, &hbt);
          return hbt.custom_mode;
        }
      }
    }
  }
  return 100; // failed
}

bool update_gps_pos() {
  request_data();

  mavlink_message_t msg;
  mavlink_status_t status;

  unsigned long curr_time = millis();
  while (millis() - curr_time < 3000) {
    while (Serial1.available() > 0) {
      uint8_t c = Serial1.read();

      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
          mavlink_global_position_int_t gps;
          mavlink_msg_global_position_int_decode(&msg, &gps);

          // check this math
          curr_lat = (float) (gps.lat * pow(10.0, -7.0));
          curr_lon = (float) (gps.lon * pow(10.0, -7.0));
          curr_alt = (float) (gps.relative_alt / 1000.0); // mm -> m

          // check how the cast rounds
          Serial.print("Raw lat: "); Serial.println(gps.lat);
          Serial.print("Casted lat: "); Serial.println(curr_lat, 6);
          // check alt
          Serial.print("Relative alt: "); Serial.println(curr_alt);

          return true;
        }
      }
    }
  }
  return false; //failed
}

void request_data() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  /*
    const uint8_t MAVStream = 0;//MAV_DATA_STREAM_RAW_SENSORS; // Enables IMU_RAW, GPS_RAW, GPS_STATUS packets. May need to change to MAV_DATA_STREAM_EXTENDED_STATUS
    const uint16_t MAVRate = 0x02;
    const int  maxStreams = 2;
    mavlink_msg_request_data_stream_pack(system_id, component_id, &msg, target_system, target_component, MAVStream, MAVRate, 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  */

  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS, MAV_DATA_STREAM_EXTENDED_STATUS}; // GPS and SYS_STATUS respectively
  const uint16_t MAVRates[maxStreams] = {0x02, 0x05};

  for (int i = 0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(system_id, component_id, &msg, target_system, target_component, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}

uint16_t get_reached_item() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();

    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM_REACHED) {
        mavlink_mission_item_reached_t item;
        mavlink_msg_mission_item_reached_decode(&msg, &item);

        return item.seq;
      }
    }
  }
  return 100; // failed
}

void send_next_mission_item(float lat, float lon, float alt) {
  send_mission_count(3);
  comm_receive(); // request
  send_waypoint(0, 0, 0, 0); // home waypoint
  comm_receive(); // request
  send_loiter_turns(1, lat, lon, alt); // data collection waypoint
  comm_receive(); // request
  send_loiter_time(2, lat, lon, alt); // waiting waypoint
  com_receive(); // acknowledgement
  acknowledge();
}

void send_mission_count(int count) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_count_pack(system_id, component_id, &msg, target_system, target_component, count);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

// lat:[-90,90] lon:[-180,180]
void send_waypoint(uint16_t seq, float lat, float lon, float alt) {
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  uint16_t command = MAV_CMD_NAV_WAYPOINT;
  uint8_t current = 0;
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

// lat:[-90,90] lon:[-180,180]
// xtrack location: 0 for center of loiter wp, 1 for exit location
// if rad positive, clockwise turn. if rad negative, counter-clockwise turn
void send_loiter_turns(uint16_t seq, float lat, float lon, float alt) {
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  uint16_t command = MAV_CMD_NAV_LOITER_TURNS;
  uint8_t current = 0;
  uint8_t autocontinue = 0;
  uint8_t turns = 1;
  uint8_t rad = loiter_radius;
  uint8_t xtrack_loc = 0;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_item_pack(system_id, component_id, &msg, target_system, target_component, seq, frame, command, current, autocontinue, turns, 0, rad, xtrack_loc, lat, lon, alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

// lat:[-90,90] lon:[-180,180]
// xtrack location: 0 for center of loiter wp, 1 for exit location
// if rad positive, clockwise turn. if rad negative, counter-clockwise turn
void send_loiter_time(uint16_t seq, float lat, float lon, float alt) {
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  uint16_t command = MAV_CMD_NAV_LOITER_TIME;
  uint8_t current = 0;
  uint8_t autocontinue = 0;
  uint8_t tim = loiter_time;
  uint8_t rad = loiter_radius;
  uint8_t xtrack_loc = 0;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_item_pack(system_id, component_id, &msg, target_system, target_component, seq, frame, command, current, autocontinue, tim, 0, rad, xtrack_loc, lat, lon, alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_MISSION_REQUEST:
          {
            mavlink_mission_request_t request;
            mavlink_msg_mission_request_decode(&msg, &request);

            Serial.println("MISSION_REQUEST sequence: "); Serial.println(request.seq);
            Serial.println();
          }
          break;
        case MAVLINK_MSG_ID_MISSION_ACK:
          {
            mavlink_mission_ack_t ack;
            mavlink_msg_mission_ack_decode(&msg, &ack);

            Serial.println("MISSION ACK type: "); Serial.println(ack.type);
            Serial.println();
          }
          break;
        default:
          break;
      }
    }
  }
}

void acknowledge() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_ack_pack(system_id, component_id, &msg, target_system, target_component, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}
