/* for sensors */
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_DPS310.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include <stdio.h>

/* for logging */
// #define NOWRITE
#ifdef NANDFLASH
#include <LittleFS.h>
#else
#include <SD.h>
#include <SPI.h>
#endif

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
uint16_t BNO055_SAMPLERATE_DELAY_MS = 1000;
float BNO055_SAMPLERATE_DELAY_S = BNO055_SAMPLERATE_DELAY_MS / 1000;

//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp;
Adafruit_Sensor *dps_pressure;

#ifdef NANDFLASH
LittleFS_SPINAND myfs;
const int chipSelect = 4;
#else
const int chipSelect = BUILTIN_SDCARD;  // hopefully works
#endif

File dataFile;

bool isArmed = false; // can change for debug purposes, NEEDS to be false for launch
bool hasLaunched = false;
unsigned long startTime = 0;
unsigned long TIME_TO_CHUTE = 5 * 60 * 1000;  // 5 minutes to stay on safe side, can change to an hr to prevent false alarms?
unsigned long cnt = 0;
float sqLen(float v[3]) {
  return v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
}

float len(float v[3]) {
  return sqrt(sqLen(v));
}

void setup(void) {
    Serial.begin(115200);

    // while (!Serial)
    //     delay(10); /* wait for serial port to open */
    // waiting is actually unnecessary, as when the board is used in flight
    // the serial port will not be connected, so we can just start the loop


    Serial.println("Orientation Sensor Initialising");
    Serial.println("");

    /* Initialise the sensor */
    if (!bno.begin()) {
        Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    }

    dps_temp = dps.getTemperatureSensor();
    dps_pressure = dps.getPressureSensor();

    if (! dps.begin_I2C(0x77, &Wire)) {
      Serial.println("Bayes DPS not detected");
      while (1);
    }
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);


    Serial2.begin(9600);
    startTime = millis();
    while (!Serial2) {
        if (millis() - startTime > 5000) {
            Serial.println("Failed to connect to Serial2!");
            // blink board LED
            while (1) {
                digitalWrite(LED_BUILTIN, HIGH);
                delay(100);
                digitalWrite(LED_BUILTIN, LOW);
                delay(100);
            }
            break;
        }
        delay(10);
    };
    Serial.println("Bluetooth connected");


#ifdef NANDFLASH
    // SPI port param is optional
    if (!myfs.begin(chipSelect, SPI)) {
        Serial.printf("Error starting %s\n", "SPI FLASH");
        while (1) {
            // Flash error, so don't do anything more - stay stuck here
        }
    }
    Serial.println("Flash initialized.");
#else
    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        #ifndef NOWRITE
        while (1);
        #endif
    }
    // File dataFile = SD.open("datalog.txt", FILE_WRITE);
    // dataFile.println("\nSTART NEW LOGGING RUN");
    Serial.println("card initialized.");
#endif

    char baseFilename[16] = "datalog-";
    char filename[16];
    int fileCnt = 0;
  #ifdef NANDFLASH
        // TODO
        do {
            sprintf(filename, "%s%d.txt", baseFilename, fileCnt);
            fileCnt++;
        }  while (myfs.exists(filename) && fileCnt < 50);
        dataFile = myfs.open(filename, FILE_WRITE);
  #else
        do {
            sprintf(filename, "%s%d.txt", baseFilename, fileCnt);
            fileCnt++;
        }  while (SD.exists(filename) && fileCnt < 50);
        dataFile = SD.open(filename, FILE_WRITE);
  #endif
  if (fileCnt > 50){
    Serial.println("Are you sure you're okay with this many files?");
  }
  Serial.println("file opened, fileCnt=");
  Serial.println(fileCnt);
  Serial.print("filename=");
  Serial.println(filename);


    delay(1000);
}

void loop(void) {
    
    String dataBuf = "";
    dataBuf += "time (seconds since setup):";
    dataBuf += String(BNO055_SAMPLERATE_DELAY_S*cnt) + '\n';
    cnt++;
    // could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
    sensors_event_t orientationData, angVelocityData, linearAccelData,
    magnetometerData, accelerometerData, gravityData, pressureData, tempData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    dataBuf += printEventToStr(&orientationData) + '\n';
    dataBuf += printEventToStr(&angVelocityData) + '\n';
    dataBuf += printEventToStr(&linearAccelData) + '\n';
    dataBuf += printEventToStr(&magnetometerData) + '\n';   // surely we don't care about this? comment out if so
    dataBuf += printEventToStr(&accelerometerData) + '\n';  // redundant with linear accel
    dataBuf += printEventToStr(&gravityData) + '\n';        // really shouldn't be varying as far as I can tell probably can also remove

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
    String calib = "";
    calib += "Calibration:\n";
    calib += "System:";
    calib += String(system);
    calib += "  Gyro:";
    calib += String(gyro);
    calib += "  Accel:";
    calib += String(accel);
    calib += "  Mag:";
    calib += String(mag);
    if (!hasLaunched) {
    Serial2.println(calib);
    Serial.println(calib);
    }

    dataBuf += calib + '\n';
    float temperature = 0;
    float pressure = 0;
    if (dps.temperatureAvailable()) {
        dps_temp->getEvent(&tempData);
        temperature = tempData.temperature;
    }
    if (dps.pressureAvailable()) {
        dps_pressure->getEvent(&pressureData);
        pressure = pressureData.pressure;
    }
    else{
      // Serial.println("pressure not available :(");
    }

    dataBuf += "temp: ";
    dataBuf += String(temperature);
    dataBuf += "\tPressure:";
    dataBuf += String(pressure);
    dataBuf += "\n";

    float *acceleration = linearAccelData.acceleration.v;
    Serial2.print("Accel magnitude: ");
    Serial.print("Accel magnitude: ");
    Serial2.println(len(acceleration));
    Serial.println(len(acceleration));

    dataBuf += "\n--\n";

    if (!hasLaunched && isArmed) {
      float *accel = linearAccelData.acceleration.v;
      if (len(accel) > 2) { // units are SI (ms^-2)
        hasLaunched = true;
        Serial2.println("We have liftoff!");
        startTime = millis();
      }
    }

    if (hasLaunched && (millis() - startTime < TIME_TO_CHUTE)) {
      Serial.println(dataBuf);
#ifndef NOWRITE
        // if the file is available, write to it:
        if (dataFile) {
            dataFile.println(dataBuf);
            dataFile.flush();
            // print to the serial port too:
        } else {
            // if the file isn't open, pop up an error:
            Serial.println("error opening file");
        }
#endif
    }

    if (Serial2.available()) {
        char c = Serial2.read();
        Serial.print(c);
        if (c == 'a') { 
            isArmed = !isArmed;
            if (isArmed){
              Serial.println("Armed!");
              Serial2.println("Armed!");
            }
            else{
              hasLaunched = false;
              Serial.println("Disarmed!");
              Serial2.println("Disarmed!");
            }
        }
    }

    delay(BNO055_SAMPLERATE_DELAY_MS);
}

String printEventToStr(sensors_event_t *event) {
    String dataBuf = "";
    double x = NAN, y = NAN, z = NAN;  // dumb values, easy to spot problem
    if (event->type == SENSOR_TYPE_ACCELEROMETER) {
        dataBuf += "Accl:";
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    } else if (event->type == SENSOR_TYPE_ORIENTATION) {
        dataBuf += "Orient:";
        x = event->orientation.x;
        y = event->orientation.y;
        z = event->orientation.z;
    } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
        dataBuf += "Mag:";
        x = event->magnetic.x;
        y = event->magnetic.y;
        z = event->magnetic.z;
    } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
        dataBuf += "Gyro:";
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
        dataBuf += "Rot:";
        x = event->gyro.x;
        y = event->gyro.y;
        z = event->gyro.z;
    } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
        dataBuf += "Linear:";
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    } else if (event->type == SENSOR_TYPE_GRAVITY) {
        dataBuf += "Gravity:";
        x = event->acceleration.x;
        y = event->acceleration.y;
        z = event->acceleration.z;
    } else {
        dataBuf += "Unk:";  // unknown
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

void printEvent(sensors_event_t *event) {
    Serial.print(printEventToStr(event));
}