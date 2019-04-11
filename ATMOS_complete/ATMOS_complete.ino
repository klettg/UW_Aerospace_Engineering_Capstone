#include <mavlink.h> // must have mavlink.h in same directory
#include <math.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include "RTClib.h"

#define print_data_to_console 0 // 0 to supress data printing to console

#define loiter_radius 2500 // radius at which to loiter (meters)
#define loiter_tolerance 5 // tolerance of aircraft distance to loiter radius (meters)
#define loiter_time 30 // time to loiter while waiting for next waypoint (seconds)

// see defines.h
#define MANUAL 0 // safe mode. stops payload operation
#define STABILIZE 2 // data collection mode. flys normally but collects data during flight
#define TRAINING 3 // recovery mode. continues payload operation after halted
#define AUTO 10 // autonomous mode. collects data and uses it to determine future waypoints

#define system_id 255
#define component_id 190
#define target_system 1
#define target_component 1
#define autocontinue 0

#define ozone_address 0x50 // = 80
#define CO_address 0x51 // = 81
#define CS_pin 10 // chip select line
#define beeper_pin 2

#define timeout 2000 // ms
#define data_wait_time 3000 // time to wait between data points (ms)

#define pi 3.1415926535

uint32_t flight_mode = 99;

double curr_lat = 0;
double curr_lon = 0;
double curr_alt = 0;

double curr_wp_lat = 0;
double curr_wp_lon = 0;
double curr_wp_alt = 0;

bool orbit_completed = false;

// point to determine next waypoint direction based on air quality data
double dir_x = 0;
double dir_y = 0;

File file; // file object to store data
char filename[] = "DATA00.CSV"; // name of file to create and write to

RTC_PCF8523 rtc; // real time clock (RTC) object
bool RTC_found;

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
  double lat;
  double lon;
} score_point;

void setup() {
  Serial.begin(115200); // debugging output
  Serial.println("starting up");

  pinMode(beeper_pin, OUTPUT); beep(2, 100); // set up beeper

  Serial1.begin(57600); // communication with PixHawk [19(RX), 18(TX)]
  pmsSerial.begin(9600); // communication with PM sensor
  Wire.begin(); // I2C communication with ozone and CO sensors

  unsigned long curr_time = millis();
  while (flight_mode == 99) {
    update_flight_mode();
    if (millis() - curr_time > 5000) {
      Serial.println("ERROR: no hearbeat packets received. ending execution");
      while (1) {
        beep(1, 100);
      }
    }
  }

  // connect to RTC and check connection
  if (rtc.begin()) {
    RTC_found = true;
  } else {
    RTC_found = false;
    Serial.println("RTC not found");
    beep(3, 1000);
  }

  // set up SD card
  pinMode(CS_pin, OUTPUT);
  if (SD.begin(CS_pin)) {
    Serial.println("SD connected");
  } else {
    Serial.println("SD ERROR: could not connect");
    beep(10, 1000);
  }

  // create new file name
  for (uint8_t i = 0; i < 100; i++) {
    filename[4] = i / 10 + '0';
    filename[5] = i % 10 + '0';
    if (!SD.exists(filename)) {
      break;
    }
    if (i == 99) {
      Serial.println("SD ERROR: all filenames used. please clear card");
      beep(5, 1000);
    }
  }
  Serial.print("Logging to: "); Serial.println(filename);

  // write data header
  // SCU = standard concentration units, ECU = environmental concentration units
  file = SD.open(filename, FILE_WRITE);
  file.println("Date, Time, latitude, longitude, altitude (m), SCU PM 1.0, SCU PM 2.5, SCU PM 10, ECU PM 1.0, ECU PM 2.5, ECU PM 10, 0.3um, 0.5um, 1.0um, 2.5um, 5.0um, 10.0um, Ozone ppm, CO ppm, Flight Mode");
  file.close();
}

void loop() {
  unsigned long prev_time = millis() - data_wait_time;
  while (flight_mode != AUTO) {
    update_gps_pos();
    send_mission(curr_lat, curr_lon, curr_alt);
    if (flight_mode == MANUAL) { // manual = safe mode. stop payload interaction fully
      halt();
    }
    if (flight_mode == STABILIZE && millis() - prev_time >= data_wait_time) { // collect data if in stabilize
      rec_data_point();
      prev_time = millis();
    }
    update_flight_mode();
  }

  while (flight_mode == AUTO) {
    // new orbit is beginning. reset direction point and orbit completed state
    orbit_completed = false;
    dir_x = 0;
    dir_y = 0;

    Serial.println("starting orbit");

    prev_time = millis() - data_wait_time;
    while (!orbit_completed) {
      // collect data, wait for orbit completion
      if (within_orbit_rad() && millis() - prev_time >= data_wait_time) {
        rec_data_point();
        prev_time = millis();
      }
      update_flight_mode();
      if (flight_mode != AUTO) {
        // if taken out of auto mid orbit, exit
        break;
      }
      get_reached_item();
    }
    // mission completed

    if (flight_mode == AUTO) { // only calculate next mission if still in auto mode
      Serial.println("orbit completed");
      calculate_next_mission();
      set_mission_current();
    }
    update_flight_mode();
  }
}

void rec_data_point() {
  file = SD.open(filename, FILE_WRITE); // open file for writing
  file.print(' ');

  write_date_time();

  bool success = false;
  unsigned long curr_time = millis();
  while (millis() - curr_time < 2000) { // timeout
    if (read_PM_data(&pmsSerial)) { // wait for successful PM reading. should be once every second
      success = true;

      update_gps_pos();
      write_gps();
      record_PM();
      record_ozone();
      record_CO();

      file.print(flight_mode); file.print(',');

      // if in auto mode, use data point to update direction point
      if (flight_mode == AUTO) {
        score_point scr_point;
        scr_point.score = calculate_score();
        scr_point.lat = curr_lat;
        scr_point.lon = curr_lon;

        update_direction_point(scr_point);
      }

      if (!print_data_to_console) {
        Serial.println("data point recorded");
      }

      break;
    }
  }

  if (!success) {
    Serial.println("rec_data_point() failed due to failed PM reading");
    file.print("failed data point"); file.print(',');
  }

  file.println();
  file.close(); // close file to save
}

bool within_orbit_rad() {
  double dist = sqrt(pow((curr_wp_lat - curr_lat) * 111111, 2) + pow((curr_wp_lon -  curr_lon) * 111111 * cos(curr_wp_lat * (pi / 180)), 2));
  return dist <= loiter_radius + loiter_tolerance;
}

/*
  change based on what type(s) of data is/are most important
*/
uint16_t calculate_score() {
  return PM_data.pm25_env;
}

void update_direction_point(score_point scr_point) {
  double lat_offset = (scr_point.lat - curr_wp_lat);
  double lon_offset = (scr_point.lon - curr_wp_lon);
  double theta = atan2(lat_offset, lon_offset); // theta = angle from x axis (current waypoint centered) to current location

  if (lon_offset >= 0) { // quadrants I and IV
    dir_x += scr_point.score * cos(theta);
    dir_y += scr_point.score * sin(theta);
  } else { // quadrants II and III
    dir_x += scr_point.score * -cos(theta + pi);
    dir_y += scr_point.score * -sin(theta + pi);
  }
}

void calculate_next_mission() {
  double new_lat;
  double new_lon;
  double phi = atan2(dir_y, dir_x); // phi = angle from x axis (current waypoint centered) to direction of worst air quality

  new_lat = curr_wp_lat + 2 * loiter_radius / 111111.0 * sin(phi);
  new_lon = curr_wp_lon + 2 * loiter_radius / (111111 * cos(curr_wp_lat * (pi / 180))) * cos(phi);

  send_mission(new_lat, new_lon, curr_wp_alt); // update position, maintain same altitude
}

boolean read_PM_data(Stream *s) {
  if (! s->available()) {
    return false;
  }

  // listen for '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

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

  // endianness
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put data into the data struct
  memcpy((void *)&PM_data, (void *)buffer_u16, 30);
  if (sum != PM_data.checksum) {
    return false;
  }
  return true;
}

void record_PM() {
  // write current PM data
  file.print(PM_data.pm10_standard); file.print(',');
  file.print(PM_data.pm25_standard); file.print(',');
  file.print(PM_data.pm100_standard); file.print(',');
  file.print(PM_data.pm10_env); file.print(',');
  file.print(PM_data.pm25_env); file.print(',');
  file.print(PM_data.pm100_env); file.print(',');
  file.print(PM_data.particles_03um); file.print(',');
  file.print(PM_data.particles_05um); file.print(',');
  file.print(PM_data.particles_10um); file.print(',');
  file.print(PM_data.particles_25um); file.print(',');
  file.print(PM_data.particles_50um); file.print(',');
  file.print(PM_data.particles_100um); file.print(',');

  if (print_data_to_console) {
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: "); Serial.print(PM_data.pm10_standard);
    Serial.print("\t\tPM 2.5: "); Serial.print(PM_data.pm25_standard);
    Serial.print("\t\tPM 10: "); Serial.println(PM_data.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: "); Serial.print(PM_data.pm10_env);
    Serial.print("\t\tPM 2.5: "); Serial.print(PM_data.pm25_env);
    Serial.print("\t\tPM 10: "); Serial.println(PM_data.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(PM_data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(PM_data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(PM_data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(PM_data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(PM_data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(PM_data.particles_100um);
    Serial.println("---------------------------------------");
    Serial.println();
  }
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

    if (print_data_to_console) {
      Serial.print("Ozone parts per million: "); Serial.println(ozone_PPM);
    }
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

    if (print_data_to_console) {
      Serial.print("CO parts per million: "); Serial.println(CO_PPM);
    }
    file.print(CO_PPM); file.print(',');
  } else {
    Serial.print("CO reading failed");
    file.print("failed"); file.print(',');
  }
}

void write_date_time() {
  if (RTC_found) {
    DateTime now = rtc.now();

    file.print(now.year(), DEC);
    file.print('/');
    file.print(now.month(), DEC);
    file.print('/');
    file.print(now.day(), DEC);
    file.print(',');

    file.print(now.hour(), DEC);
    file.print(':');
    file.print(now.minute(), DEC);
    file.print(':');
    file.print(now.second(), DEC);
    file.print(',');

    if (print_data_to_console) {
      Serial.print(now.year(), DEC);
      Serial.print('/');
      Serial.print(now.month(), DEC);
      Serial.print('/');
      Serial.print(now.day(), DEC);
      Serial.print(' ');

      Serial.print(now.hour(), DEC);
      Serial.print(':');
      Serial.print(now.minute(), DEC);
      Serial.print(':');
      Serial.print(now.second(), DEC);
      Serial.println();
    }
  } else {
    file.print("error"); file.print(',');
    file.print("error"); file.print(',');
  }
}

void write_gps() {
  file.print(curr_lat, 6); file.print(',');
  file.print(curr_lon, 6); file.print(',');
  file.print(curr_alt); file.print(',');
}

void set_mission_current() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t seq = 1;
  mavlink_msg_mission_set_current_pack(system_id, component_id, &msg, target_system, target_component, seq);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

/*
  timeouts implemented per https://mavlink.io/en/services/mission.html
*/
void send_mission(double lat, double lon, double alt) {
  curr_wp_lat = lat;
  curr_wp_lon = lon;
  curr_wp_alt = alt;

  send_mission_count(3);
  bool received = false;
  for (uint8_t i = 1; i <= 5; i++) {
    if (receive_req()) {
      // success
      received = true;
      break;
    } else {
      // failed, retry maximum 5 times
      send_mission_count(3);
    }
  }
  if (!received) {
    Serial.println("send_mission() failed. entering failsafe");
    failsafe(99);
  }

  send_waypoint(0, 0, 0, 0); // home waypoint
  received = false;
  for (uint8_t i = 1; i <= 5; i++) {
    if (receive_req()) {
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
    failsafe(99);
  }

  send_loiter_turns(1, lat, lon, alt); // data collection waypoint
  received = false;
  for (uint8_t i = 1; i <= 5; i++) {
    if (receive_req()) {
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
    failsafe(99);
  }

  send_loiter_time(2, lat, lon, alt); // waiting waypoint
  received = false;
  for (uint8_t i = 1; i <= 5; i++) {
    if (receive_ack()) {
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
    failsafe(99);
  }

  send_acknowledgment();
}

void send_mission_count(int count) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_count_pack(system_id, component_id, &msg, target_system, target_component, count);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

/*
  lat:[-90,90] lon:[-180,180]
*/
void send_waypoint(uint16_t seq, float lat, float lon, float alt) {
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  uint16_t command = MAV_CMD_NAV_WAYPOINT;
  uint8_t current = 0;
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

/*
  lat:[-90,90] lon:[-180,180]
  xtrack location: 0 for center of loiter wp, 1 for exit location
  if rad positive, clockwise turn. if rad negative, counter-clockwise turn
*/
void send_loiter_turns(uint16_t seq, float lat, float lon, float alt) {
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  uint16_t command = MAV_CMD_NAV_LOITER_TURNS;
  uint8_t current = 1;
  uint8_t turns = 1;
  uint8_t rad = loiter_radius;
  uint8_t xtrack_loc = 0;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_item_pack(system_id, component_id, &msg, target_system, target_component, seq, frame, command, current, autocontinue, turns, 0, rad, xtrack_loc, lat, lon, alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

/*
  lat:[-90,90] lon:[-180,180]
  xtrack location: 0 for center of loiter wp, 1 for exit location
  if rad positive, clockwise turn. if rad negative, counter-clockwise turn
*/
void send_loiter_time(uint16_t seq, float lat, float lon, float alt) {
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  uint16_t command = MAV_CMD_NAV_LOITER_TIME;
  uint8_t current = 0;
  uint8_t tim = loiter_time;
  uint8_t rad = loiter_radius;
  uint8_t xtrack_loc = 0;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_item_pack(system_id, component_id, &msg, target_system, target_component, seq, frame, command, current, autocontinue, tim, 0, rad, xtrack_loc, lat, lon, alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void send_acknowledgment() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_ack_pack(system_id, component_id, &msg, target_system, target_component, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void request_data(uint8_t stream) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_request_data_stream_pack(system_id, component_id, &msg, target_system, target_component, stream, 0x02, 1); // MAVRate = 0x02

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

bool comm_receive(uint8_t desired_ID) {
  mavlink_message_t msg;
  mavlink_status_t status;

  unsigned long curr_time = millis();
  while (millis() - curr_time < timeout) {
    while (Serial1.available() > 0) {
      uint8_t c = Serial1.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        uint8_t received_ID = msg.msgid;
        switch (received_ID) {
          case MAVLINK_MSG_ID_HEARTBEAT: // update flight mode
            {
              mavlink_heartbeat_t hbt;
              mavlink_msg_heartbeat_decode(&msg, &hbt);
              flight_mode = hbt.custom_mode;

              if (desired_ID == MAVLINK_MSG_ID_HEARTBEAT) {
                return true;
              }
            }
            break;
          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // update position
            {
              mavlink_global_position_int_t gps;
              mavlink_msg_global_position_int_decode(&msg, &gps);

              curr_lat = (gps.lat * pow(10.0, -7.0));
              curr_lon = (gps.lon * pow(10.0, -7.0));
              curr_alt = (gps.relative_alt / 1000.0); // mm -> m

              // Serial.print("Relative alt: "); Serial.println(curr_alt);

              if (desired_ID == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                return true;
              }
            }
            break;
          case MAVLINK_MSG_ID_MISSION_REQUEST: // receive request
            {
              /*
                mavlink_mission_request_t request;
                mavlink_msg_mission_request_decode(&msg, &request);
                Serial.println("MISSION_REQUEST sequence: "); Serial.println(request.seq);
              */

              if (desired_ID == MAVLINK_MSG_ID_MISSION_REQUEST) {
                return true;
              }
            }
            break;
          case MAVLINK_MSG_ID_MISSION_ACK: // receive acknowledgement
            {
              /*
                mavlink_mission_ack_t ack;
                mavlink_msg_mission_ack_decode(&msg, &ack);
                Serial.println("MISSION ACK type: "); Serial.println(ack.type);
              */

              if (desired_ID == MAVLINK_MSG_ID_MISSION_ACK) {
                return true;
              }
            }
            break;
          case MAVLINK_MSG_ID_MISSION_ITEM_REACHED: // get reached item
            {
              mavlink_mission_item_reached_t item;
              mavlink_msg_mission_item_reached_decode(&msg, &item);

              //Serial.print("Completed sequence: "); Serial.println(item.seq);

              if (item.seq == 1) {
                orbit_completed = true;
              }

              if (desired_ID == MAVLINK_MSG_ID_MISSION_ITEM_REACHED) {
                return true;
              }
            }
            break;
          case MAVLINK_MSG_ID_MISSION_CURRENT: // get current item
            {
              mavlink_mission_current_t curr;
              mavlink_msg_mission_current_decode(&msg, &curr);

              //Serial.print("Current sequence: "); Serial.println(curr.seq);

              if (curr.seq != 1) {
                orbit_completed = true;
              }

              if (desired_ID == MAVLINK_MSG_ID_MISSION_CURRENT) {
                return true;
              }
            }
          default:
            break;
        }
      }
    }
  }
  return false; // failed
}

bool update_flight_mode() {
  comm_receive(MAVLINK_MSG_ID_HEARTBEAT);
}

void update_gps_pos() {
  request_data(MAV_DATA_STREAM_RAW_SENSORS);
  if (!comm_receive(MAVLINK_MSG_ID_GLOBAL_POSITION_INT)) {
    Serial.println("update_gps_pos() failed. entering failsafe");
    failsafe(88);
  }
}

bool receive_req() {
  return comm_receive(MAVLINK_MSG_ID_MISSION_REQUEST);
}

bool receive_ack() {
  return comm_receive(MAVLINK_MSG_ID_MISSION_ACK);
}

void get_reached_item() {
  comm_receive(MAVLINK_MSG_ID_MISSION_ITEM_REACHED);
  comm_receive(MAVLINK_MSG_ID_MISSION_CURRENT);
}

/*
  tells aircraft to return to launch. sends message telling what went wrong
*/
void failsafe(float ID) {
  beep(5, 100);

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

  return_to_launch(1);
  received = false;
  for (uint8_t i = 1; i <= 5; i++) {
    if (comm_receive(MAVLINK_MSG_ID_MISSION_REQUEST)) {
      // success
      received = true;
      break;
    } else {
      // failed, retry maximum 5 times
      return_to_launch(1);
    }
  }
  if (!received) {
    Serial.println("failsafe() failed. halting program execution");
    halt();
  }

  failsafe_message(2, ID);
  received = false;
  for (uint8_t i = 1; i <= 5; i++) {
    if (comm_receive(MAVLINK_MSG_ID_MISSION_ACK)) {
      // success
      received = true;
      break;
    } else {
      // failed, retry maximum 5 times
      failsafe_message(2, ID);
    }
  }
  if (!received) {
    Serial.println("failsafe() failed. halting program execution");
    halt();
  }

  send_acknowledgment();

  halt();
}

void failsafe_message(uint16_t seq, float ID) {
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  uint16_t command = 208; // =MAV_CMD_DO_PARACHUTE;
  uint8_t current = 0;
  //uint8_t autocontinue = 1;
  float param2 = 0;
  float param3 = 0;
  float param4 = 0;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_item_pack(system_id, component_id, &msg, target_system, target_component, seq, frame, command, current, autocontinue, ID, param2, param3, param4, curr_lat, curr_lon, curr_alt);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void return_to_launch(uint16_t seq) {
  uint8_t frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
  uint16_t command = MAV_CMD_NAV_RETURN_TO_LAUNCH;
  uint8_t current = 1;
  //uint8_t autocontinue = 1;

  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_item_pack(system_id, component_id, &msg, target_system, target_component, seq, frame, command, current, autocontinue, 0, 0, 0, 0, 0, 0, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf, len);
}

void beep(uint8_t count, uint8_t interval) {
  for (uint8_t i = 1; i <= count; i++) {
    analogWrite(beeper_pin, 50);
    delay(interval);
    analogWrite(beeper_pin, 0);
    delay(interval);
  }
}

void halt() {
  Serial.println("halting");
  while (flight_mode != TRAINING) { // training is recovery mode. put into training to continue payload operation
    update_flight_mode();
    beep(1, 3000);
  }
  Serial.println("continuing from halt");
}
