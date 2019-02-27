#include </Users/dylanhoff/Documents/School/AerosPACE/mavlink/mavlink.h>

unsigned long previousMillisMAVLink = 0;
unsigned long next_interval_MAVLink = 1000; // interval between heartbeats

void setup() {
  Serial1.begin(57600); // communication with PixHawk   19(RX), 18(TX)
  Serial.begin(9600); // debugging output
  Serial.println("starting up");
}

void loop() {
  int sysid = 1; // 1 for PX
  int compid = 0; // (doesn't matter?)
  int type = MAV_TYPE_FIXED_WING;

  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID; // No valid autopilot, e.g. a GCS or other MAVLink component

  uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
  uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
  uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
  mavlink_msg_heartbeat_pack(sysid, compid, &msg, type, autopilot_type, system_mode, custom_mode, system_state);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  // only requesting info once a second
  unsigned long currentMillisMAVLink = millis();
  if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
    // Timing variables
    previousMillisMAVLink = currentMillisMAVLink;

    Serial1.write(buf, len);
    Mav_Request_Data();
  }

  comm_receive();
}

void Mav_Request_Data() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  const uint8_t MAVStream = MAV_DATA_STREAM_RAW_SENSORS; // Enables IMU_RAW, GPS_RAW, GPS_STATUS packets. May need to change to MAV_DATA_STREAM_EXTENDED_STATUS
  const uint16_t MAVRate = 0x02; // change?

  mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStream, MAVRate, 1); // 2 and 200?
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  Serial1.write(buf, len); // request data
}

void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();

    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message
      switch (msg.msgid) {
        case MAVLINK_MSG_ID_GPS_RAW_INT:
          {
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(&msg, &gps);
            
            // print gps.time_usec;
            //Serial1.println(gps.time_usec);
            
            Serial.print("MAVLINK_MSG_ID_GPS_RAW_INT lat:");
            Serial.println(gps.lat);
            
            Serial.print("MAVLINK_MSG_ID_GPS_RAW_INT lon:");
            Serial.println(gps.lon);
          }
          break;
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          {
            mavlink_global_position_int_t gps33;
            mavlink_msg_global_position_int_decode(&msg, &gps33);

            Serial.println("-----------GLOBAL_POSITION_INT----------");
            Serial.print("GLOBAL_POSITION_INT lat:");
            Serial.println(gps33.lat);
            Serial.print("GLOBAL_POSITION_INT lon:");
            Serial.println(gps33.lon);
            Serial.print("GLOBAL_POSITION_INT alt:");
            Serial.println(gps33.alt);
            Serial.print("GLOBAL_POSITION_INT relalt:");
            Serial.println(gps33.relative_alt);
            Serial.println("-----------END----------");
            
          }
          break;
        default:
          break;   
      }
    } 
  }
}

