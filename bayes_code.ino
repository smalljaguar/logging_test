#include <stdio.h>

#include "serialisation.hpp"
#include "sensors.hpp"
#include "utils.hpp"

/* for logging */
// #define NOWRITE
#ifdef NANDFLASH
#include <LittleFS.h>
#else
#include <SD.h>
#include <SPI.h>
#endif


#ifdef NANDFLASH
LittleFS_SPINAND myfs;
const int chipSelect = 4;
#else
const int chipSelect = BUILTIN_SDCARD;  // hopefully works
#endif

File dataFile;

bool isArmed = true;  // can change for debug purposes, NEEDS to be false for launch
bool hasLaunched = true;
unsigned long launchTime = 0;
unsigned long TIME_TO_CHUTE = 5 * 60 * 1000;  // 5 minutes to stay on safe side, can change to an hr to prevent false alarms?

unsigned long MIN_STORAGE_BYTES = 1 << 24;  // 16MB

bool checkStorage() {
#ifdef NANDFLASH
    if (myfs.totalSize() - myfs.usedSize() < MIN_STORAGE_BYTES) {
        return false;
    }
#else
    if (SD.totalSize() - SD.usedSize() < MIN_STORAGE_BYTES) {
        return false;
    }
#endif
    return true;
}


void setup(void) {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);

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

    if (!dps.begin_I2C(0x77, &Wire)) {
        Serial.println("Bayes DPS not detected");
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

    // int startTime = millis();
    
    Serial2.begin(9600);  // attempt to set this to 115200 (should work automagically?)
    /*
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
  */
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

    if (!checkStorage()) {
        Serial.println("Not enough storage space!!");
        Serial2.println("Not enough storage space!!");
        while (1) {
            // Flash error, so don't do anything more - stay stuck here
        }
    }

    char baseFilename[16] = "datalog-";
    char filename[16];
    unsigned int fileCnt = 0;
    unsigned int maxFileCnt = 256;
#ifdef NANDFLASH
    do {
        sprintf(filename, "%s%d.json", baseFilename, fileCnt);
        fileCnt++;
    } while (myfs.exists(filename) && fileCnt < maxFileCnt);
    dataFile = myfs.open(filename, FILE_WRITE);
#else
    do {
        sprintf(filename, "%s%d.json", baseFilename, fileCnt);
        fileCnt++;
    } while (SD.exists(filename) && fileCnt < maxFileCnt);
    dataFile = SD.open(filename, FILE_WRITE);
#endif

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
    Serial.println("Line 169");
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
    sensors_event_t linearAccelData;
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    float *acceleration = linearAccelData.acceleration.v;
    Serial2.print("Accel magnitude: ");
    Serial.print("Accel magnitude: ");
    Serial2.println(magnitude(acceleration));
    Serial.println(magnitude(acceleration));
    if (!hasLaunched && isArmed) {
        float *accel = linearAccelData.acceleration.v;
        if (magnitude(accel) > 10 && magnitude(accel) < 200) {  // units are SI (ms^-2), 150 is sanity check
            hasLaunched = true;
            Serial2.println("We have liftoff!");
            launchTime = millis();
            if (dataFile) {
                // dataFile.println(generateCSVHeader());
            }
        }
    }
    if (hasLaunched) {
        FlightData flight_data = {0};
        Serial.println("about to collect sensor data");
        collectSensorData(flight_data);
        Serial.println("sensor data collected");
        // MAKE SURE TO DISABLE FOR LAUNCH!!
        String dataBuf = serialiseFlightData(flight_data);
        Serial.println(dataBuf);
        // Serial2.println(dataBuf);
        // String csvData = serialiseToCSV(flight_data);
        String JSONData = serialiseToJSON(flight_data);
#ifndef NOWRITE
        // if the file is available, write to it:
        if (dataFile) {
            dataFile.println(JSONData);
            dataFile.flush();
            digitalWrite(LED_BUILTIN, HIGH);
            // print to the serial port too:
        } else {
            // if the file isn't open, pop up an error:
            Serial.println("error opening file");
        }
#endif
    }

    
    // if (Serial2.available()) {
    //     char c = Serial2.read();
    //     Serial.print(c);
    //     if (c == 'a') {
    //         isArmed = !isArmed;
    //         if (isArmed) {
    //             Serial.println("Armed!");
    //             Serial2.println("Armed!");
    //             FlightData flight_data = {0};
    //             collectSensorData(flight_data);
    //             seaLevelPressure = bmp.seaLevelForAltitude(250, flight_data.bmp280.pressure); /* fairley moor height is 250m */
    //         } else {
    //             hasLaunched = false;
    //             Serial.println("Disarmed!");
    //             Serial2.println("Disarmed!");
    //         }
    //     }
    // }
    

    delay(SAMPLERATE_DELAY_MS/2);
    digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
    delay(SAMPLERATE_DELAY_MS/2);

}
