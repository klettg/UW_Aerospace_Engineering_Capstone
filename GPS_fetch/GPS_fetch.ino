#include <mavlink.h> // must have mavlink.h in same directory

unsigned long previousMillisMAVLink = 0;
unsigned long next_interval_MAVLink = 1000; // interval between heartbeats

uint8_t system_id = 2;
uint8_t component_id = 200;
uint8_t target_system = 1;
uint8_t target_component = 0;

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

  //const uint8_t MAVStream = MAV_DATA_STREAM_RAW_SENSORS; // Enables IMU_RAW, GPS_RAW, GPS_STATUS packets. May need to change to MAV_DATA_STREAM_EXTENDED_STATUS
  //const uint16_t MAVRate = 0x02;

  const int  maxStreams = 2;
  const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS, MAV_DATA_STREAM_EXTENDED_STATUS}; // GPS and SYS_STATUS respectively
  const uint16_t MAVRates[maxStreams] = {0x02, 0x05};

  for (int i = 0; i < maxStreams; i++) {
    mavlink_msg_request_data_stream_pack(system_id, component_id, &msg, target_system, target_component, MAVStreams[i], MAVRates[i], 1);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
  }
}

void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;

  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();

    // Try to get a new message
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      // Handle message
      //Serial.println(msg.msgid);
      switch (msg.msgid) {
        /*
          case MAVLINK_MSG_ID_GPS_RAW_INT:
          {
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(&msg, &gps);

            Serial.print("MAVLINK_MSG_ID_GPS_RAW_INT time:");

            uint64_t num = gps.time_usec;

            uint32_t low = num % 0xFFFFFFFF;
            uint32_t high = (num >> 32) % 0xFFFFFFFF;

            Serial.print(low);
            Serial.println(high);

            Serial.print("MAVLINK_MSG_ID_GPS_RAW_INT lat:");
            Serial.println(gps.lat);

            Serial.print("MAVLINK_MSG_ID_GPS_RAW_INT lon:");
            Serial.println(gps.lon);
          }
          break;
        */
        case MAVLINK_MSG_ID_HEARTBEAT:
          {

            Serial.println("heartbeat");

          }
          break;
        // the filtered global position (e.g. fused GPS and accelerometers)
        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
          {
            mavlink_global_position_int_t gps;
            mavlink_msg_global_position_int_decode(&msg, &gps);

            Serial.println("-----------GLOBAL_POSITION_INT----------");
            Serial.print("GLOBAL_POSITION_INT lat:");
            Serial.println(gps.lat);
            Serial.print("GLOBAL_POSITION_INT lon:");
            Serial.println(gps.lon);
            Serial.println("-----------END----------");
            Serial.println();

          }
          break;
        case MAVLINK_MSG_ID_SYSTEM_TIME:
          {
            mavlink_system_time_t tim;
            mavlink_msg_system_time_decode(&msg, &tim);

            uint64_t t_unix = tim.time_unix_usec;

            uint32_t low = t_unix % 0xFFFFFFFF;
            uint32_t high = (t_unix >> 32) % 0xFFFFFFFF;

            Serial.print("SYSTEM_TIME time:");
            Serial.print(low);
            Serial.println(high);
            Serial.print("SYTEM_TIME since last boot:");
            Serial.println(tim.time_boot_ms);
            Serial.println();
          }
          break;
        default:
          break;
      }
    }
  }
}
