#include <stdio.h>

#include "serialisation.hpp"
#include "sensors.hpp"
#include "utils.hpp"
#include "pid.hpp"
#include "servos.hpp"

/* for logging */
#define NOWRITE
#include <SD.h>
#include <SPI.h>


// IMU: clock 19 and data 18 

const int chipSelect = BUILTIN_SDCARD;

File dataFile;

bool hasLaunched = true;

unsigned long MIN_STORAGE_BYTES = 1 << 24;  // 16MB

bool checkStorage() {
    if (SD.totalSize() - SD.usedSize() < MIN_STORAGE_BYTES) {
        return false;
    }
    return true;
}

// Adafruit_BNO055 bno = Adafruit_BNO055();

void setup(void) {
  
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

    // waiting is actually unnecessary, as when the board is used in flight
    // the serial port will not be connected, so we can just start the loop

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
    // TODO: configure this maybe
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

    delay(1000);
}

void loop(void) {
    /*
    defines calibration level where 0 is uncalibrated, 3 is fully calibrated
    calibration is done automatically, so not too much we can do about it
    might not want to log every cycle, feels a bit noisy, perhaps only if it changes and/or if it's not 3?
    */
    int loopStart = millis();
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    char calib[128];
    sprintf(calib,"Calibration:\nSystem: %d  Gyro: %d  Accel: %d  Mag: %d", system, gyro, accel, mag);
    if (!hasLaunched) {
        Serial.println(calib);
    }
    sensors_event_t linearAccelData;
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    float *acceleration = linearAccelData.acceleration.v;
    Serial.print("Accel magnitude: ");
    Serial.println(magnitude(acceleration));
    if (hasLaunched) {
        FlightData flight_data = {0};
        Serial.println("about to collect sensor data");
        collectSensorData(flight_data);
        Serial.println("sensor data collected");
        String dataBuf = serialiseFlightData(flight_data);
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
    
    int delay_time = SAMPLERATE_DELAY_MS-(millis()-loopStart);
    delay(delay_time);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
}
