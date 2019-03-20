/*
    TODO:
    -are autocontinue and current set correctly?
    -verify altitude readings
    -implement message timeouts per: https://mavlink.io/en/services/mission.html
    -implement timeout for commands
    -adjust timeout values
    -convert system time from unix (RTClib?)
    -implement SD card failsafe/warning, acknowldegment?
    -implement beeper messages
    -verify serial2 works in place of software serial
    -improve calculate_score() -> email WSDE
    -verify math in calculate_next_mission()
    -split into classes
*/

#include <mavlink.h> // must have mavlink.h in same directory
#include <math.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

#define loiter_radius 30 // radius at which to loiter (meters)
#define loiter_time 15 // time to loiter while waiting for next waypoint (seconds)

#define AUTO 10 // see defines.h
#define STABILIZE 2 // see defines.h

#define system_id 255
#define component_id 190
#define target_system 1
#define target_component 1

#define ozone_address 0x50 // = 80
#define CO_address 0x51 // = 81
#define CS_pin 10 // pin 10 is chip select line

#define orbit_points_size 300
#define data_wait_time 5000 // time to wait between data points (ms)

bool auto_mode = false;

float curr_lat = 0;
float curr_lon = 0;
float curr_alt = 0;

float curr_wp_lat = 0;
float curr_wp_lon = 0;
float curr_wp_alt = 0;

File file; // file object to store data
char filename[] = "DATA00.CSV"; // name of file to create and write to

// structure to store PM2.5 sensor data
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};
struct pms5003data PM_data;

typedef struct {
  uint16_t score;
  float lat;
  float lon;
} score_point;

score_point orbit_points[orbit_points_size];
uint8_t orbit_index = 0;

void setup() {
  Serial.begin(115200); // debugging output
  Serial.println("starting up");

  Serial1.begin(57600); // communication with PixHawk [19(RX), 18(TX)]
  Serial2.begin(9600); // communication with PM2.5 sensor [16(RX), 17(TX)]
  Wire.begin(); // I2C communication with ozone and CO sensors

  // set up SD card
  pinMode(CS_pin, OUTPUT);
  if (SD.begin(CS_pin)) {
    Serial.println("SD connected");
  } else {
    Serial.println("SD error: could not connect");
  }

  // create new file name
  for (uint8_t i = 0; i < 255; i++) {
    filename[4] = i / 10 + '0';
    filename[5] = i % 10 + '0';
    if (!SD.exists(filename)) {
      break;
    }
  }
  Serial.print("Logging to: "); Serial.println(filename);

  // write data header
  // SCU = standard concentration units, ECU = environmental concentration units
  file = SD.open(filename, FILE_WRITE);
  file.println("Date, Time, latitude, longitude, altitude (m), SCU PM 1.0, SCU PM 2.5, SCU PM 10, ECU PM 1.0, ECU PM 2.5, ECU PM 10, 0.3um, 0.5um, 1.0um, 2.5um, 5.0um, 10.0um, Ozone ppm, CO ppm");
  file.close();
}

void loop() {
  update_gps_pos();
  uint32_t mode = get_flight_mode();
  
  while (mode != AUTO) {
    update_gps_pos();
    send_mission(curr_lat, curr_lon, curr_alt);

    unsigned long prev_time = millis();    
    while (mode == STABILIZE) { // collect data if in stabilize
      // collect data
      if (millis() - prev_time >= data_wait_time) {
        rec_data_point();
        prev_time = millis();
      }
      // continue sending wp's in case we go from stabilize to auto
      update_gps_pos();
      send_mission(curr_lat, curr_lon, curr_alt);
    }
    mode = get_flight_mode();
  }

  auto_mode = true;

  // listen for item completed message (finished first orbit)
  // we are assumming that the plane sends the loiter item when it
  // finishes the loiter. it may send it when it begins it in
  // which case we want to check for seq = 2, meaning it has moved
  // on to the timed "waiting" loiter
  // maybe change autocontinue/current?
  while (get_flight_mode() == AUTO) {
    unsigned long prev_time = millis();
    while (get_reached_item() != 1) { // seq = 1
      // wait, collect data
      if (millis() - prev_time >= data_wait_time) {
        rec_data_point();
        prev_time = millis();
      }
    }
    // orbit completed
    calculate_next_mission();
  }
  
  auto_mode = false;
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
  // FAILSAFE
  return 100; // failed
}

void update_gps_pos() {
  request_data(MAV_DATA_STREAM_RAW_SENSORS);

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

          return;
        }
      }
    }
  }
  // FAILSAFE
}

uint32_t get_time() {
  request_data(MAV_DATA_STREAM_EXTENDED_STATUS);

  mavlink_message_t msg;
  mavlink_status_t status;

  unsigned long curr_time = millis();
  while (millis() - curr_time < 3000) {
    while (Serial1.available() > 0) {
      uint8_t c = Serial1.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_SYSTEM_TIME) {
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

          return 1;
        }
      }
    }
  }
  // FAILSAFE
  return 0; // failed
}

void request_data(uint8_t stream) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  uint8_t MAVStream = stream;
  uint16_t MAVRate = 0x02;

  mavlink_msg_request_data_stream_pack(system_id, component_id, &msg, target_system, target_component, MAVStream, MAVRate, 1);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
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

void send_mission(float lat, float lon, float alt) {
  curr_wp_lat = lat;
  curr_wp_lon = lon;
  curr_wp_alt = alt;

  send_mission_count(3);
  comm_receive(); // request
  send_waypoint(0, 0, 0, 0); // home waypoint
  comm_receive(); // request
  send_loiter_turns(1, lat, lon, alt); // data collection waypoint
  comm_receive(); // request
  send_loiter_time(2, lat, lon, alt); // waiting waypoint
  comm_receive(); // acknowledgement
  send_acknowledgment();
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

void send_acknowledgment() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_ack_pack(system_id, component_id, &msg, target_system, target_component, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void rec_data_point() {
  file = SD.open(filename, FILE_WRITE); // open file for writing
  file.print(get_time());

  unsigned long curr_time = millis();
  while (millis() - curr_time < 10000) { // timeout
    // wait for successful PM reading
    if (!read_PM_data(&Serial2)) {
      update_gps_pos();
      write_gps();
      record_PM();
      record_ozone();
      record_CO();

      if (auto_mode) {
        record_score_point();
      }
    }
  }
  
  file.println();
  file.close(); // close file to save
}

void calculate_next_mission() {
  // point to determine direction
  double dir_x = 0;
  double dir_y = 0;
  for (uint8_t i = 0; i < orbit_index; i++) {
    score_point point = orbit_points[i];
    double lat_offset = (double)(point.lat - curr_wp_lat);
    double lon_offset = (double)(point.lon - curr_wp_lon);
    double theta = atan2(lat_offset, lon_offset);
    if (lon_offset >= 0) { // quadrant I and IV check atan(1/0))
      dir_x += point.score * cos(theta);
      dir_y += point.score * sin(theta);
    } else { // quadrant II and III
      dir_x += point.score * -cos(theta);
      dir_y += point.score * -sin(theta);
    }
  }
  orbit_index = 0; // reset array

  double new_lat;
  double new_lon;
  double phi = atan2(dir_y, dir_x); // phi = direction of worst air quality
  if (dir_x >= 0) { // quadrant I and IV
    new_lat = curr_wp_lat + loiter_radius * sin(phi);
    new_lon = curr_wp_lon + loiter_radius * cos(phi);
  } else { // quadrant II and III
    new_lat = curr_wp_lat + loiter_radius * -sin(phi);
    new_lon = curr_wp_lon + loiter_radius * -cos(phi);
  }
  send_mission(new_lat, new_lon, curr_wp_alt); // update position, maintain same altitude
}

void record_score_point() {
  score_point point;
  point.score = calculate_score();
  point.lat = curr_lat;
  point.lon = curr_lon;

  if (orbit_index < orbit_points_size) {
    orbit_points[orbit_index] = point;
    orbit_index++;
  } else {
    // send message that full
  }
}

// change
uint16_t calculate_score() {
  return PM_data.particles_25um;
}

void write_gps() {
  file.print(curr_lat); file.print(',');
  file.print(curr_lon); file.print(',');
  file.print(curr_alt); file.print(',');
}

void record_PM() {
  // PM data updated, print
  Serial.println("---------------------------------------");
  Serial.println("Concentration Units (standard)");
  Serial.print("PM 1.0: "); Serial.print(PM_data.pm10_standard); file.print(PM_data.pm10_standard); file.print(',');
  Serial.print("\t\tPM 2.5: "); Serial.print(PM_data.pm25_standard); file.print(PM_data.pm25_standard); file.print(',');
  Serial.print("\t\tPM 10: "); Serial.println(PM_data.pm100_standard); file.print(PM_data.pm100_standard); file.print(',');
  Serial.println("---------------------------------------");
  Serial.println("Concentration Units (environmental)");
  Serial.print("PM 1.0: "); Serial.print(PM_data.pm10_env); file.print(PM_data.pm10_env); file.print(',');
  Serial.print("\t\tPM 2.5: "); Serial.print(PM_data.pm25_env); file.print(PM_data.pm25_env); file.print(',');
  Serial.print("\t\tPM 10: "); Serial.println(PM_data.pm100_env); file.print(PM_data.pm100_env); file.print(',');
  Serial.println("---------------------------------------");
  Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(PM_data.particles_03um); file.print(PM_data.particles_03um); file.print(',');
  Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(PM_data.particles_05um); file.print(PM_data.particles_05um); file.print(',');
  Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(PM_data.particles_10um); file.print(PM_data.particles_10um); file.print(',');
  Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(PM_data.particles_25um); file.print(PM_data.particles_25um); file.print(',');
  Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(PM_data.particles_50um); file.print(PM_data.particles_50um); file.print(',');
  Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(PM_data.particles_100um); file.print(PM_data.particles_100um); file.print(',');
  Serial.println("---------------------------------------");
  Serial.println();
}

void record_ozone() {
  Wire.beginTransmission(ozone_address);
  Wire.write(0x00); // set register number to read/write from (= 0 for conversion result)
  Wire.endTransmission();

  Wire.requestFrom(ozone_address, 2);
  unsigned int ozone_data[2];
  if (Wire.available() == 2)
  {
    ozone_data[0] = Wire.read(); // most significant bit
    ozone_data[1] = Wire.read(); // least significant bit

    // convert the data to 12-bits
    int raw_adc = ((ozone_data[0] & 0x0F) * 256) + ozone_data[1];
    float ozone_PPM = (1.99 * raw_adc) / 4095.0 + 0.01;

    Serial.print("Ozone parts per million: "); Serial.println(ozone_PPM);
    file.print(ozone_PPM); file.print(',');
  } else {
    // PRINT ERROR MESSAGE
  }
}

void record_CO() {
  Wire.beginTransmission(CO_address);
  Wire.write(0x00); // set register number to read/write from (= 0 for conversion result)
  Wire.endTransmission();

  Wire.requestFrom(CO_address, 2);
  unsigned int CO_data[2];
  if (Wire.available() == 2)
  {
    CO_data[0] = Wire.read(); // most significant bit
    CO_data[1] = Wire.read(); // least significant bit

    // convert the data to 12-bits
    int raw_adc = ((CO_data[0] & 0x0F) * 256) + CO_data[1];
    float CO_PPM = (1.99 * raw_adc) / 4095.0 + 0.01;

    Serial.print("CO parts per million: "); Serial.println(CO_PPM);
    file.print(CO_PPM); file.print(',');
  } else {
    // PRINT ERROR MESSAGE
  }
}

boolean read_PM_data(Stream *s) {
  if (! s->available()) {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i = 0; i < 30; i++) {
    sum += buffer[i];
  }

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put data into the data struct
  memcpy((void *)&PM_data, (void *)buffer_u16, 30);
  if (sum != PM_data.checksum) {
    Serial.println("PM checksum failure");
    return false;
  }
  return true;
}
