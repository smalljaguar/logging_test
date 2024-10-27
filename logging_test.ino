/* for sensors */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* for logging */
#include <SD.h>
#include <SPI.h>

/* for bluetooth */
#include <SoftwareSerial.h>


/* 
  Stolen from adafruit examples
  need to add logging (write to flash/microsd)
  add bluetooth (also works over serial)
  To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).

   Sensor Connections
   ===========
   Connect SCL to analog 18 (default 5)
   Connect SDA to analog 19 (default 4)
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground
   SD Connections
   ===========
   SD card attached to SPI bus as follows:
   MOSI - pin 11
   MISO - pin 12
   CLK - pin 13
   CS - pin 4
  Bluetooth Connections
  ===========
  RX - 7
  TX - 8
*/

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);


const int chipSelect = BUILTIN_SDCARD; // will this work?


SoftwareSerial BTSerial(7, 8); // RX | TX
bool is_logging = false;
unsigned long startTime = 0;
unsigned long TIME_TO_CHUTE = 5*60*1000; // 5 minutes to stay on safe side

void setup(void)
{
  Serial.begin(115200);

  while (!Serial) delay(10);  /* wait for serial port to open */

  Serial.println("Orientation Sensor Initialising"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }



  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }
  Serial.println("card initialized.");


  BTSerial.begin(38400); /* HC-05 default speed in AT command more */


  delay(1000);
}

void loop(void)
{
  String dataBuf = "";

  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER); 
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  dataBuf += printEventToStr(&orientationData) + '\n';
  dataBuf += printEventToStr(&angVelocityData) + '\n';
  dataBuf += printEventToStr(&linearAccelData) + '\n';
  dataBuf += printEventToStr(&magnetometerData) + '\n'; // surely we don't care about this? comment out if so
  dataBuf += printEventToStr(&accelerometerData) + '\n'; // redundant with linear accel
  dataBuf += printEventToStr(&gravityData) + '\n';  // really shouldn't be varying as far as I can tell probably can also remove


  // don't know if we want this? remember it only changes at 1Hz
  /*
  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  dataBuf += "\nTemp: ";
  dataBuf += String(boardTemp) + '\n';
  */

  /*
  defines calibration level where 0 is uncalibrated, 3 is fully calibrated
  calibration is done automatically, so not too much we can do about it
  might not want to log every cycle, feels a bit noisy, perhaps only if it changes and/or if it's not 3?
  */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  if (system < 3) {
    dataBuf += "SysCal: ";
    dataBuf += String(system);
  }
  if (gyro < 3) {
    dataBuf += " GyroCal: ";
    dataBuf += String(gyro);
  }
  if (accel < 3) {
    dataBuf += " AccelCal: ";
    dataBuf += String(accel);
  }
  if (mag < 3) {
    dataBuf += " MagCal: ";
    dataBuf += String(mag);
  }
  dataBuf += '\n';
  // Serial.println();
  // Serial.print("Calibration: Sys=");
  // Serial.print(system);
  // Serial.print(" Gyro=");
  // Serial.print(gyro);
  // Serial.print(" Accel=");
  // Serial.print(accel);
  // Serial.print(" Mag=");
  // Serial.println(mag);

  Serial.println("--");
  dataBuf += "--\n";


  if (is_logging && (millis() - startTime) < TIME_TO_CHUTE) {
    File dataFile = SD.open("datalog.txt", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataBuf);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataBuf);
    } else {
      // if the file isn't open, pop up an error:
      Serial.println("error opening datalog.txt");
    }
  }

  if (BTSerial.available()) {
    char c = BTSerial.read();
    if (c == 'l') { 
      is_logging = !is_logging;
      if (is_logging)
        {startTime = millis();}
      Serial.print("Logging: ");
      Serial.println(is_logging);
      BTSerial.write("Logging: ");
      BTSerial.write(is_logging);
      BTSerial.write('\n');
    }
  }


  delay(BNO055_SAMPLERATE_DELAY_MS); 
}


String printEventToStr(sensors_event_t* event) {
  String dataBuf = "";
  double x = NAN, y = NAN , z = NAN; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    dataBuf += "Accl:";
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    dataBuf += "Orient:";
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    dataBuf += "Mag:";
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    dataBuf += "Gyro:";
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    dataBuf += "Rot:";
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    dataBuf += "Linear:";
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    dataBuf += "Gravity:";
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    dataBuf += "Unk:"; // unknown
  }


  dataBuf += "\tx= ";
  dataBuf += String(x); 
  dataBuf += " |\ty= ";
  dataBuf += String(y);
  dataBuf += " |\tz= ";
  dataBuf += String(z);
  dataBuf += '\n';
  
  
  return dataBuf;

}


void printEvent(sensors_event_t* event) {
  Serial.print(printEventToStr(event));
}