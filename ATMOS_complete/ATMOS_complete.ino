/*
    TODO:
    -are autocontinue and current set correctly?
    -verify altitude readings
    -verify failsafe
    -test test_failsafe_message()
    -verify update_gps_pos()
    -functionality testing -> make written test plan for normal and edge cases
    -verify comm_receive() desiredID if statement
    -test timeouts by checking realistic times

    -adjust timeout values. will they cause problems?
    -convert system time from unix in get_time() (RTClib?)
    -improve calculate_score()
    -verify math in calculate_next_mission()
    -communication with GCS?
    -implement failsafe as full mission
    -implement protocol for maximum score points in add_score_point()
*/

#include <mavlink.h> // must have mavlink.h in same directory
#include <math.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>

#define loiter_radius 30 // radius at which to loiter (meters)
#define loiter_time 15 // time to loiter while waiting for next waypoint (seconds)

#define MANUAL 0 // see defines.h
#define STABILIZE 2 // see defines.h
#define AUTO 10 // see defines.h
#define RTL 11 // see defines.h

#define system_id 255
#define component_id 190
#define target_system 1
#define target_component 1

#define ozone_address 0x50 // = 80
#define CO_address 0x51 // = 81
#define CS_pin 10 // chip select line
#define beeper_pin 2

#define orbit_points_size 256 // if increased past 256, orbit_index must be changed to type uint16_t
#define data_wait_time 5000 // time to wait between data points (ms)

bool auto_mode = false;

uint8_t 

float curr_lat = 0;
float curr_lon = 0;
float curr_alt = 0;

float curr_wp_lat = 0;
float curr_wp_lon = 0;
float curr_wp_alt = 0;

File file; // file object to store data
char filename[] = "DATA00.CSV"; // name of file to create and write to

SoftwareSerial pmsSerial(11, 10); // serial communication for PM sensor [11(RX), 10(TX)]

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

  pinMode(beeper_pin, OUTPUT); beep(2, 100); // set up beeper

  Serial1.begin(57600); // communication with PixHawk [19(RX), 18(TX)]
  pmsSerial.begin(9600); // communication with PM sensor
  Wire.begin(); // I2C communication with ozone and CO sensors

  // set up SD card
  pinMode(CS_pin, OUTPUT);
  if (SD.begin(CS_pin)) {
    Serial.println("SD connected");
  } else {
    Serial.println("SD error: could not connect");
    beep(10, 1000);
  }

  // create new file name
  for (uint8_t i = 0; i < 255; i++) {
    filename[4] = i / 10 + '0';
    filename[5] = i % 10 + '0';
    if (!SD.exists(filename)) {
      break;
    }
    if (i == 255) {
      Serial.println("SD error: all filenames used. please clear card. overwriting DATA255.CSV");
      beep(5, 1000);
    }
  }
  Serial.print("Logging to: "); Serial.println(filename);

  // write data header
  // SCU = standard concentration units, ECU = environmental concentration units
  file = SD.open(filename, FILE_WRITE);
  file.println("Date, Time, latitude, longitude, altitude (m), SCU PM 1.0, SCU PM 2.5, SCU PM 10, ECU PM 1.0, ECU PM 2.5, ECU PM 10, 0.3um, 0.5um, 1.0um, 2.5um, 5.0um, 10.0um, Ozone ppm, CO ppm, Auto Mode");
  file.close();

  // FOR TESTING. DELETE AFTER TESTING COMPLETE
  test_failsafe_message()
}

void loop() {
  update_gps_pos();
  uint32_t mode = get_flight_mode();

  while (mode != AUTO) {
    if (mode == MANUAL) { // manual = safe mode. stop payload interaction fully
      halt();
    }
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
      // include this? too many calls of get_flight_mode()?
      if (get_flight_mode() != AUTO) {
        break;
      }
    }
    // orbit completed
    calculate_next_mission();
  }

  auto_mode = false;
}

// TEST TO SEE IF THIS COMMAND WILL TRIGGER A MESSAGE SENT TO THE GCS.
// IF SO, IMPLEMENT THIS TO THE FAILSAFE INSTEAD OF RTL
// ALSO CHECK IF COMMANDS THEMSELF SHOW UP IN MISSION PLANNER
void test_failsafe_message() {
  send_mission_count(2);
  comm_receive(MAVLINK_MSG_ID_MISSION_REQUEST);
  send_waypoint(0, 0, 0, 0);
  comm_receive(MAVLINK_MSG_ID_MISSION_REQUEST);
  
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  uint16_t command = MAV_CMD_NAV_PAYLOAD_PLACE;
  uint8_t current = 0;
  uint8_t autocontinue = 0;
  float param1 = 1010.10; // value displayed to GCS
  float param2 = 0;
  float param3 = 0;
  float param4 = 0;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_item_pack(system_id, component_id, &msg, target_system, target_component, 1, frame, command, current, autocontinue, param1, param2, param3, param4, curr_lat, curr_lon, curr_alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);

  comm_receive(MAVLINK_MSG_ID_MISSION_ACK);
  send_acknowledgment();
}

void beep(uint8_t count, uint8_t interval) {
  for (uint8_t i = 1; i <= count; i++) {
    analogWrite(beeper_pin, 50);
    delay(interval);
    analogWrite(beeper_pin, 0);
    delay(interval);
  }
}

uint32_t get_flight_mode() {
  mavlink_message_t msg;
  mavlink_status_t status;

  unsigned long curr_time = millis();
  while (millis() - curr_time < 1500) {
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
  Serial.println("get_flight_mode() failsafe");
  failsafe();
  return 100; // failed
}

void update_gps_pos() {
  request_data(MAV_DATA_STREAM_RAW_SENSORS);

  mavlink_message_t msg;
  mavlink_status_t status;

  unsigned long curr_time = millis();
  while (millis() - curr_time < 1500) {
    while (Serial1.available() > 0) {
      uint8_t c = Serial1.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
          mavlink_global_position_int_t gps;
          mavlink_msg_global_position_int_decode(&msg, &gps);

          curr_lat = (float) (gps.lat * pow(10.0, -7.0));
          curr_lon = (float) (gps.lon * pow(10.0, -7.0));
          curr_alt = (float) (gps.relative_alt / 1000.0); // mm -> m

          // TESTS. DELETE AFTER TESTING
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
  Serial.println("update_gps_pos() failed. entering failsafe");
  failsafe();
}

uint32_t get_time() {
  request_data(MAV_DATA_STREAM_EXTENDED_STATUS);

  mavlink_message_t msg;
  mavlink_status_t status;

  unsigned long curr_time = millis();
  while (millis() - curr_time < 1500) {
    while (Serial1.available() > 0) {
      uint8_t c = Serial1.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_SYSTEM_TIME) {
          mavlink_system_time_t tim;
          mavlink_msg_system_time_decode(&msg, &tim);

          uint64_t t_unix = tim.time_unix_usec;

          // REPLACE WITH CONVERSION TO READABLE TIME AND RETURN (CHANGE TYPE TO STRING?)
          uint32_t low = t_unix % 0xFFFFFFFF;
          uint32_t high = (t_unix >> 32) % 0xFFFFFFFF;

          Serial.print("SYSTEM_TIME time:");
          Serial.print(low);
          Serial.println(high);
          Serial.print("SYTEM_TIME since last boot:");
          Serial.println(tim.time_boot_ms);
          Serial.println();

          return 1;
          // END REPLACE
        }
      }
    }
  }
  Serial.println("get_time() failed");
  return 0; // failed
}

/* unix_convert()
  // converts unix time to a string representing the current time in ISO 8601
  String unix_convert(uint64_t unix_time) {
  String yr = 1970 + unix_time/31556926;
  uint64_t yr_remainder = unix_time%31556926;
  String mnth = 1 + yr_remainder/2629743;
  uint64_t mnth_remainder = yr_remainder%2629743;
  String dy = 1 + mnth_remainder/86400;
  uint64_t dy_remainder = mnth_remainder%86400;
  String hr = dy_remainder/3600;
  uint64_t hr_remainder = dy_remainder%3600;
  String mnt = hr_remainder/60;
  uint64_t mnt_remainder = hr_remainder%60;
  String sec = mnt_remainder/1;
  return yr + '-' + mnth + '-' + dy + 'T' + hr + ':' + mnt + ':' + sec;
  }
*/

void request_data(uint8_t stream) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_request_data_stream_pack(system_id, component_id, &msg, target_system, target_component, stream, 0x02, 1); // MAVRate = 0x02

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

uint16_t get_reached_item() {
  mavlink_message_t msg;
  mavlink_status_t status;

  // items will not be completed frequently. no need for timeout
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

// timeouts implemented per https://mavlink.io/en/services/mission.html
void send_mission(float lat, float lon, float alt) {
  curr_wp_lat = lat;
  curr_wp_lon = lon;
  curr_wp_alt = alt;

  send_mission_count(3);
  bool received = false;
  for (uint8_t i = 1; i <= 5; i++) {
    if (comm_receive(MAVLINK_MSG_ID_MISSION_REQUEST)) {
      // success
      received = true;
      break;
    } else {
      // failed, retry maximum 5 times
      send_mission_count(3);
    }
  }
  if (!received) {
    Serial.println("send_mission() failed. enter failsafe");
    failsafe();
  }
  
  send_waypoint(0, 0, 0, 0); // home waypoint
  received = false;
  for (uint8_t i = 1; i <= 5; i++) {
    if (comm_receive(MAVLINK_MSG_ID_MISSION_REQUEST)) {
      // success
      received = true;
      break;
    } else {
      // failed, retry maximum 5 times
      send_waypoint(0, 0, 0, 0);
    }
  }
  if (!received) {
    Serial.println("send_mission() failed. entering failsafe");
    failsafe();
  }
  
  send_loiter_turns(1, lat, lon, alt); // data collection waypoint
  received = false;
  for (uint8_t i = 1; i <= 5; i++) {
    if (comm_receive(MAVLINK_MSG_ID_MISSION_REQUEST)) {
      // success
      received = true;
      break;
    } else {
      // failed, retry maximum 5 times
      send_loiter_turns(1, lat, lon, alt);
    }
  }
  if (!received) {
    Serial.println("send_mission() failed. entering failsafe");
    failsafe();
  }

  send_loiter_time(2, lat, lon, alt); // waiting waypoint
  received = false;
  for (uint8_t i = 1; i <= 5; i++) {
    if (comm_receive(MAVLINK_MSG_ID_MISSION_ACK)) {
      // success
      received = true;
      break;
    } else {
      // failed, retry maximum 5 times
      send_loiter_time(2, lat, lon, alt);
    }
  }
  if (!received) {
    Serial.println("send_mission() failed. entering failsafe");
    failsafe();
  }

  send_acknowledgment();
}

void failsafe() {
  beep(5, 100);

  send_mission_count(2);
  bool received = false;
  for (uint8_t i = 1; i <= 5; i++) {
    if (comm_receive(MAVLINK_MSG_ID_MISSION_REQUEST)) {
      // success
      received = true;
      break;
    } else {
      // failed, retry maximum 5 times
      send_mission_count(2);
    }
  }
  if (!received) {
    Serial.println("failsafe() failed. halting program execution");
    halt();
  }
  
  send_waypoint(0, 0, 0, 0);
  received = false;
  for (uint8_t i = 1; i <= 5; i++) {
    if (comm_receive(MAVLINK_MSG_ID_MISSION_REQUEST)) {
      // success
      received = true;
      break;
    } else {
      // failed, retry maximum 5 times
      send_waypoint(0, 0, 0, 0);
    }
  }
  if (!received) {
    Serial.println("failsafe() failed. halting program execution");
    halt();
  }
  
  set_mode(RTL);
  received = false;
  for (uint8_t i = 1; i <= 5; i++) {
    if (comm_receive(MAVLINK_MSG_ID_MISSION_ACK)) {
      // success
      received = true;
      break;
    } else {
      // failed, retry maximum 5 times
      set_mode(RTL);
    }
  }
  if (!received) {
    Serial.println("failsafe() failed. halting program execution");
    halt();
  }

  send_acknowledgment();
}

void halt() {
  while (1) {
    beep(1, 3000);
  }
}

void set_mode(float mode) {
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
    uint16_t command = MAV_CMD_DO_SET_MODE;
    uint8_t current = 0;
    uint8_t autocontinue = 0;
    float param2 = 0; // Custom mode
    float param3 = 0; // Custom sub mode

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_mission_item_pack(system_id, component_id, &msg, target_system, target_component, seq, frame, command, current, autocontinue, mode, param2, param3, 0, 0, 0, 0);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    Serial1.write(buf, len);
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

bool comm_receive(uint8_t desired_ID) {
  mavlink_message_t msg;
  mavlink_status_t status;

  unsigned long curr_time = millis();
  while (millis() - curr_time < 1500) { // timeout
    while (Serial1.available() > 0) {
      uint8_t c = Serial1.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        uint8_t received_ID = msg.msgid;
        if (received_ID == desired_ID) {
          switch (received_ID) {
            case MAVLINK_MSG_ID_MISSION_REQUEST:
              {
                mavlink_mission_request_t request;
                mavlink_msg_mission_request_decode(&msg, &request);

                Serial.println("MISSION_REQUEST sequence: "); Serial.println(request.seq);

                return true;
              }
              break;
            case MAVLINK_MSG_ID_MISSION_ACK:
              {
                mavlink_mission_ack_t ack;
                mavlink_msg_mission_ack_decode(&msg, &ack);

                Serial.println("MISSION ACK type: "); Serial.println(ack.type);

                return true;
              }
              break;
            default:
              break;
          }
        }
      }
    }
  }
  return false; // failed
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

  bool success = false;
  unsigned long curr_time = millis();
  while (millis() - curr_time < 2000) { // timeout
    if (!read_PM_data(&pmsSerial)) { // wait for successful PM reading. should be once every second
      success = true;
      
      update_gps_pos();
      write_gps();
      record_PM();
      record_ozone();
      record_CO();

      if (auto_mode) {
        add_score_point();
      }
    }
  }
  
  if (success) {
    file.print(auto_mode); file.print(',');
  } else {
    Serial.println("rec_data_point() failed due to failed PM reading");
    file.print("failed data point"); file.print(',');
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
    if (lon_offset >= 0) { // quadrants I and IV
      dir_x += point.score * cos(theta);
      dir_y += point.score * sin(theta);
    } else { // quadrants II and III
      dir_x += point.score * -cos(theta);
      dir_y += point.score * -sin(theta);
    }
  }
  orbit_index = 0; // reset array

  double new_lat;
  double new_lon;
  double phi = atan2(dir_y, dir_x); // phi angle from x axis to direction of worst air quality
  if (dir_x >= 0) { // quadrants I and IV
    new_lat = curr_wp_lat + 2*loiter_radius * sin(phi);
    new_lon = curr_wp_lon + 2*loiter_radius * cos(phi);
  } else { // quadrants II and III
    new_lat = curr_wp_lat + 2*loiter_radius * -sin(phi);
    new_lon = curr_wp_lon + 2*loiter_radius * -cos(phi);
  }
  send_mission(new_lat, new_lon, curr_wp_alt); // update position, maintain same altitude
}

void add_score_point() {
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

// change in order to influence the data used in the mission calculation algorithm
uint16_t calculate_score() {
  return PM_data.pm25_env;
}

void write_gps() {
  file.print(curr_lat); file.print(',');
  file.print(curr_lon); file.print(',');
  file.print(curr_alt); file.print(',');
}

void record_PM() {
  // print current PM data
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
    Serial.print("ozone reading failed");
    file.print("failed"); file.print(',');
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
    Serial.print("CO reading failed");
    file.print("failed"); file.print(',');
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
