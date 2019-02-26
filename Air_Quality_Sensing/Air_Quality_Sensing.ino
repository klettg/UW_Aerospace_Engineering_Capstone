#include </Users/dylanhoff/Documents/School/AerosPACE/mavlink/mavlink.h>

#include <SoftwareSerial.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include "RTClib.h"

#define ozoneAddr 0x50 // = 80
#define coAddr 0x51 // = 81
#define SD_Check 1 // if not set to 0, will stop program if SD card fails
#define CS_Pin 10 // pin 10 is chip select line

// software serial for PM2.5 sensor
// receives to pin 2, transmits from pin 3
SoftwareSerial pmsSerial(2, 3); // TODO: change to real serial

RTC_PCF8523 rtc; // real time clock (RTC) object
bool RTC_found; // represents if the RTC has been found

File data; // file object to store data
char filename[] = "DATA00.CSV"; // name of file to create and write to

void setup() {
  // debugging output to IDE at baud rate 115200
  Serial.begin(115200);

  // begin communication with PM 2.5 sensor at baud rate 9600
  pmsSerial.begin(9600);

  // begin I2C communication
  // for use with ozone and CO sensors and RTC
  Wire.begin();

  // connect to RTC and check connection
  if (rtc.begin()) {
    RTC_found = true;
  } else { 
    RTC_found = false;
    Serial.println("RTC not found");
  }

  // set up pin for data logging
  pinMode(CS_Pin, OUTPUT);

  // connect to SD card and check connection
  // if connection fails and SD_Check is enabled, stop program and blink LED
  if (!SD.begin(CS_Pin)) {
    Serial.println("SD error: could not connect");
#if SD_Check
    while (1) {
      // flash led (insert code here)
      // beep beeper?
      // send fail message to GCS
      Serial.println("SD error");
    }
#endif
  }

  // increase the number in the filename until it is not found on the SD card.
  // This causes a new file to be written each time the Arduino is powered off.
  // Up to 256 files from "DATA00.CSV" to "DATA255.CSV"
  for (uint8_t i = 0; i < 255; i++) {
    filename[4] = i / 10 + '0';
    filename[5] = i % 10 + '0';
    if (!SD.exists(filename)) {
      break;
    }
  }

  Serial.print("Logging to: ");
  Serial.println(filename);
  // send message to GCS

/*
  // write data header
  // SCU = standard concentration units, ECU = environmental concentration units
  data = SD.open(filename, FILE_WRITE); // open file for writing
  data.println("date, time, SCU PM 1.0, SCU PM 2.5, SCU PM 10, ECU PM 1.0, ECU PM 2.5, ECU PM 10, 0.3um, 0.5um, 1.0um, 2.5um, 5.0um, 10.0um, Ozone ppm, CO ppm");
  data.close();
*/

}

// stores PM 2.5 sensor data
struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data PMdata;

void loop() {
// prints all data only if data from the PM 2.5 sensor is successfully read
  if (readPMSdata(&pmsSerial)) {
    data = SD.open(filename, FILE_WRITE); // open file for writing
    data.print(" ");

    Serial.println();
    Serial.println();
    Serial.println("***************************************");
    RTC();
    Serial.println();
    PM_sensing();
    ozone_sensing();
    Serial.println("---------------------------------------");
    CO_sensing();
    Serial.println();
    Serial.println("***************************************");
    Serial.println();
    Serial.println();

    data.println();
    data.close(); // close file to ensure data is saved
  }
}

// prints PM 2.5 data
void PM_sensing() {
  Serial.println("---------------------------------------");
  Serial.println("Concentration Units (standard)");
  Serial.print("PM 1.0: "); Serial.print(PMdata.pm10_standard); data.print(PMdata.pm10_standard); data.print(',');
  Serial.print("\t\tPM 2.5: "); Serial.print(PMdata.pm25_standard); data.print(PMdata.pm25_standard); data.print(',');
  Serial.print("\t\tPM 10: "); Serial.println(PMdata.pm100_standard); data.print(PMdata.pm100_standard); data.print(',');
  Serial.println("---------------------------------------");
  Serial.println("Concentration Units (environmental)");
  Serial.print("PM 1.0: "); Serial.print(PMdata.pm10_env); data.print(PMdata.pm10_env); data.print(',');
  Serial.print("\t\tPM 2.5: "); Serial.print(PMdata.pm25_env); data.print(PMdata.pm25_env); data.print(',');
  Serial.print("\t\tPM 10: "); Serial.println(PMdata.pm100_env); data.print(PMdata.pm100_env); data.print(',');
  Serial.println("---------------------------------------");
  Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(PMdata.particles_03um); data.print(PMdata.particles_03um); data.print(',');
  Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(PMdata.particles_05um); data.print(PMdata.particles_05um); data.print(',');
  Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(PMdata.particles_10um); data.print(PMdata.particles_10um); data.print(',');
  Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(PMdata.particles_25um); data.print(PMdata.particles_25um); data.print(',');
  Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(PMdata.particles_50um); data.print(PMdata.particles_50um); data.print(',');
  Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(PMdata.particles_100um); data.print(PMdata.particles_100um); data.print(',');
  Serial.println("---------------------------------------");
  Serial.println();
}

// reads ozone levels from sensor in parts per million (ppm) and prints the data
void ozone_sensing() {
  // set up transmission with Ozone sensor
  Wire.beginTransmission(ozoneAddr);
  Wire.write(0x00); // set register number to read/write from (= 0 for conversion result)
  Wire.endTransmission();

  // request and read 2 bytes of data
  Wire.requestFrom(ozoneAddr, 2);
  unsigned int ozone_data[2];
  if (Wire.available() == 2)
  {
    ozone_data[0] = Wire.read(); // most significant bit
    ozone_data[1] = Wire.read(); // least significant bit

    // convert the data to 12-bits
    int raw_adc = ((ozone_data[0] & 0x0F) * 256) + ozone_data[1];
    float ozone_PPM = (1.99 * raw_adc) / 4095.0 + 0.01;

    Serial.print("Ozone parts per million: "); Serial.println(ozone_PPM);
    data.print(ozone_PPM); data.print(',');
  } else {
    // send error message
  }

}

// reads carbon monoxide levels from sensor in parts per million (ppm) and prints the data
void CO_sensing() {
  // set up transmission with Ozone sensor
  Wire.beginTransmission(coAddr);
  Wire.write(0x00); // set register number to read/write from (= 0 for conversion result)
  Wire.endTransmission();

  // request and read 2 bytes of data
  Wire.requestFrom(coAddr, 2);
  unsigned int CO_data[2];
  
  if (Wire.available() == 2)
  {
    CO_data[0] = Wire.read(); // most significant bit
    CO_data[1] = Wire.read(); // least significant bit

    // convert the data to 12-bits
    int raw_adc = ((CO_data[0] & 0x0F) * 256) + CO_data[1];
    float CO_PPM = (1.99 * raw_adc) / 4095.0 + 0.01;

    Serial.print("CO parts per million: "); Serial.println(CO_PPM);
    data.print(CO_PPM); data.print(',');
  } else {
    // print error message
  }
}


// takes in a stream representing the PM 2.5 serial commmunication
// reads a set of PM 2.5 data and returns true if successful, false otherwise
boolean readPMSdata(Stream *s) {
  // exit if no bytes are available for reading
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

  /* debugging
    for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
    }
    Serial.println();
  */

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i = 0; i < 15; i++) {
    buffer_u16[i] = buffer[2 + i * 2 + 1];
    buffer_u16[i] += (buffer[2 + i * 2] << 8);
  }

  // put data into the data struct
  memcpy((void *)&PMdata, (void *)buffer_u16, 30);

  if (sum != PMdata.checksum) {
    Serial.println("Checksum failure");
    return false;
  }

  // success!
  return true;
}

// if the real time clock has been found, prints date and time
// uses A4 and A5 pins for I2C communication
// to reset clock, use File > Examples > RTClib > pcf8523
void RTC() {
  if (RTC_found) {
    DateTime now = rtc.now();

    data.print(now.year(), DEC);
    data.print('/');
    data.print(now.month(), DEC);
    data.print('/');
    data.print(now.day(), DEC);
    data.print(',');

    data.print(now.hour(), DEC);
    data.print(':');
    data.print(now.minute(), DEC);
    data.print(':');
    data.print(now.second(), DEC);
    data.print(',');

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
  } else {
    Serial.println("RTC error");
  }
}

