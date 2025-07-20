#include <stdio.h>

#include "serialisation.hpp"
#include "sensors.hpp"
#include "utils.hpp"
#include "pid.hpp"
#include "servos.hpp"

#include <SD.h>
#include <SPI.h>


// IMU: clock 19 and data 18 

bool servosEnabled;
bool hasLaunched;
const int chipSelect = BUILTIN_SDCARD;
File dataFile;

unsigned long MIN_STORAGE_BYTES = 1 << 24;  // 16MB

bool checkStorage() {
    if (SD.totalSize() - SD.usedSize() < MIN_STORAGE_BYTES) {
        return false;
    }
    return true;
}

void setup(void) {
  
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    Serial.println("Orientation Sensor Initialising\n");

    /* Initialise the sensor */
    if (!bno.begin()) {
        Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    }

    dps_temp = dps.getTemperatureSensor();
    dps_pressure = dps.getPressureSensor();

    if (!dps.begin_I2C(0x77, &Wire)) {
        Serial.println("DPS not detected");
        while (1);
    }
    // TODO: optimise this maybe
    dps.configurePressure(DPS310_128HZ, DPS310_16SAMPLES);
    dps.configureTemperature(DPS310_128HZ, DPS310_2SAMPLES);
    dps.setMode(DPS310_CONT_PRESTEMP); // think this is default but ... who knows


    if (!bmp.begin(0x76, 88)) {
        Serial.println("Could not find a valid BMP280 sensor!");
        while (1) delay(10);
    }

    /* settings from datasheet for handheld-device low power, n.b. default settings were returning stale data */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X4,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
    

    if (!SD.begin(chipSelect)) {
        Serial.println("Card failed, or not present");
        while (1);
    }
    Serial.println("card initialized.");

    if (!checkStorage()) {
        Serial.println("Not enough storage space!!");
        while (1);
            // Flash error, so don't do anything more - stay stuck here
    }

    char baseFilename[16] = "datalog-";
    char filename[16];
    unsigned int fileCnt = 0;
    unsigned int maxFileCnt = 256;

    do {
        sprintf(filename, "%s%d.json", baseFilename, fileCnt);
        fileCnt++;
    } while (SD.exists(filename) && fileCnt < maxFileCnt);
    dataFile = SD.open(filename, FILE_WRITE);

    Serial.println("file opened, fileCnt=");
    Serial.println(fileCnt);
    Serial.print("filename=");
    Serial.println(filename);

    
    servosEnabled = true;
    hasLaunched = false;

    // servo test
    testServo(servo1,1);
    testServo(servo2,2);
    testServo(servo3,3);
    testServo(servo4,4);



    delay(1000);
}

void loop(void) {
    /*
    defines calibration level where 0 is uncalibrated, 3 is fully calibrated
    calibration is done automatically, so not too much we can do about it
    might not want to log every cycle, feels a bit noisy, perhaps only if it changes and/or if it's not 3?
    */
    int currTime = millis();
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    char calib[128];
    sprintf(calib,"Calibration:\nSystem: %d  Gyro: %d  Accel: %d  Mag: %d", system, gyro, accel, mag);


    if (!hasLaunched){
        // check
        FlightData flight_data = {0};
        collectSensorData(flight_data);
        if (magnitude(flight_data.bno055.linear_acceleration) > 20){ // more than 2gs of acceleration
            hasLaunched = true;
        }
    }

    if (servosEnabled && hasLaunched){
        FlightData flight_data = {0};
        collectSensorData(flight_data);
        if (isUnsafe(flight_data)){
            servosEnabled = false;
        }
        float *angles = findCanardAngles(flight_data);
        writeServos(angles);
    }
    
    if ((currTime - millis()) > SAMPLERATE_DELAY_MS){
        currTime = millis();
        
        FlightData flight_data = {0};
        collectSensorData(flight_data);
        
        String JSONData = serialiseToJSON(flight_data);
        Serial.println(JSONData);
        // if the file is available, write to it:
        if (dataFile) {
            dataFile.println(JSONData);
            dataFile.flush();
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            // if the file isn't open, pop up an error:
            Serial.println("error opening file");
        }
    }
    
    
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
}
