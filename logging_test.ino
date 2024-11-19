/* for sensors */
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <stdio.h>
#include <utility/imumaths.h>

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

Adafruit_BMP280 bmp;  // I2C

#ifdef NANDFLASH
LittleFS_SPINAND myfs;
const int chipSelect = 4;
#else
const int chipSelect = BUILTIN_SDCARD;  // hopefully works
#endif

File dataFile;

bool isArmed = false;  // can change for debug purposes, NEEDS to be false for launch
bool hasLaunched = false;
unsigned long launchTime = 0;
unsigned long TIME_TO_CHUTE = 5 * 60 * 1000;  // 5 minutes to stay on safe side, can change to an hr to prevent false alarms?
unsigned long cnt = 0;
float sqLen(float v[3]) {
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

float len(float v[3]) {
    return sqrt(sqLen(v));
}

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

struct FlightData {
    unsigned long timestamp;  // milliseconds since start
    
    // BNO055 Sensor Data (9-axis IMU)
    struct {
        float orientation[3];    // Euler angles (x, y, z)
        float angular_velocity[3]; // Gyroscope data (x, y, z)
        float linear_acceleration[3]; // Linear acceleration (x, y, z)
        float magnetometer[3];   // Magnetometer readings (x, y, z)
        float accelerometer[3];  // Raw accelerometer data (x, y, z)
        float gravity[3];        // Gravity vector (x, y, z)
        uint8_t calibration[4];  // Calibration levels [system, gyro, accel, mag]
        int8_t board_temperature; // BNO055 board temperature
    } bno055;
    
    // DPS310 Sensor Data
    struct {
        float temperature;  // Temperature reading
        float pressure;     // Pressure reading
    } dps310;
    
    // BMP280 Sensor Data
    struct {
        float temperature;  // Temperature reading
        float pressure;     // Pressure reading
        float altitude;     // Calculated altitude
    } bmp280;
};

void collectSensorData(FlightData& data) {
    // Timestamp
    data.timestamp = millis();
    
    // BNO055 Sensor Readings
    sensors_event_t orientationData, angVelocityData, linearAccelData,
                    magnetometerData, accelerometerData, gravityData;
    
    // Orientation (Euler angles)
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    data.bno055.orientation[0] = orientationData.orientation.x;
    data.bno055.orientation[1] = orientationData.orientation.y;
    data.bno055.orientation[2] = orientationData.orientation.z;
    
    // Angular Velocity (Gyroscope)
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    data.bno055.angular_velocity[0] = angVelocityData.gyro.x;
    data.bno055.angular_velocity[1] = angVelocityData.gyro.y;
    data.bno055.angular_velocity[2] = angVelocityData.gyro.z;
    
    // Linear Acceleration
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    data.bno055.linear_acceleration[0] = linearAccelData.acceleration.x;
    data.bno055.linear_acceleration[1] = linearAccelData.acceleration.y;
    data.bno055.linear_acceleration[2] = linearAccelData.acceleration.z;
    
    // Magnetometer
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    data.bno055.magnetometer[0] = magnetometerData.magnetic.x;
    data.bno055.magnetometer[1] = magnetometerData.magnetic.y;
    data.bno055.magnetometer[2] = magnetometerData.magnetic.z;
    
    // Raw Accelerometer
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    data.bno055.accelerometer[0] = accelerometerData.acceleration.x;
    data.bno055.accelerometer[1] = accelerometerData.acceleration.y;
    data.bno055.accelerometer[2] = accelerometerData.acceleration.z;
    
    // Gravity Vector
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    data.bno055.gravity[0] = gravityData.acceleration.x;
    data.bno055.gravity[1] = gravityData.acceleration.y;
    data.bno055.gravity[2] = gravityData.acceleration.z;
    
    // Calibration Levels
    bno.getCalibration(&data.bno055.calibration[0], 
                       &data.bno055.calibration[1], 
                       &data.bno055.calibration[2], 
                       &data.bno055.calibration[3]);
    
    // Board Temperature (optional)
    data.bno055.board_temperature = bno.getTemp();
    
    // DPS310 Sensor Readings
    sensors_event_t tempData, pressureData;
    
    if (dps.temperatureAvailable()) {
        dps_temp->getEvent(&tempData);
        data.dps310.temperature = tempData.temperature;
    }
    
    if (dps.pressureAvailable()) {
        dps_pressure->getEvent(&pressureData);
        data.dps310.pressure = pressureData.pressure;
    }
    
    // BMP280 Sensor Readings
    if (bmp.takeForcedMeasurement()) {
        data.bmp280.temperature = bmp.readTemperature();
        data.bmp280.pressure = bmp.readPressure();
        data.bmp280.altitude = bmp.readAltitude(1013.25); // Adjusted to local forecast
    }
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
    // default accelerometer range is 4G, can be changed with setAccelerometerRange

    dps_temp = dps.getTemperatureSensor();
    dps_pressure = dps.getPressureSensor();

    if (!dps.begin_I2C(0x77, &Wire)) {
        Serial.println("Bayes DPS not detected");
        while (1);
    }
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP280 sensor!");
        while (1) delay(10);
    }

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

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

    if (!checkStorage()) {
        Serial.println("Not enough storage space!!");
        Serial2.println("Not enough storage space!!");
        while (1) {
            // Flash error, so don't do anything more - stay stuck here
        }
    }
    
    char baseFilename[16] = "datalog-";
    char filename[16];
    int fileCnt = 0;
#ifdef NANDFLASH
    // TODO
    do {
        sprintf(filename, "%s%d.txt", baseFilename, fileCnt);
        fileCnt++;
    } while (myfs.exists(filename) && fileCnt < 50);
    dataFile = myfs.open(filename, FILE_WRITE);
#else
    do {
        sprintf(filename, "%s%d.txt", baseFilename, fileCnt);
        fileCnt++;
    } while (SD.exists(filename) && fileCnt < 50);
    dataFile = SD.open(filename, FILE_WRITE);
#endif
    if (fileCnt > 50) {
        Serial.println("Are you sure you're okay with this many files?");
    }
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

    float *acceleration = linearAccelData.acceleration.v;
    Serial2.print("Accel magnitude: ");
    Serial.print("Accel magnitude: ");
    Serial2.println(len(acceleration));
    Serial.println(len(acceleration));

    if (!hasLaunched && isArmed) {
        float *accel = linearAccelData.acceleration.v;
        if (len(accel) > 2 && len(accel) < 150) {  // units are SI (ms^-2), 150 is sanity check
            hasLaunched = true;
            Serial2.println("We have liftoff!");
            launchTime = millis();

            if (dataFile) {
                dataFile.println(generateCSVHeader());
            }
        }
    }

    if (hasLaunched && (millis() - launchTime < TIME_TO_CHUTE)) {
        FlightData data;
        collectSensorData(data);
        String dataBuf = serializeFlightData(data);
        Serial.println(dataBuf);
        String csvData = serializeToCSV(data);
#ifndef NOWRITE
        // if the file is available, write to it:
        if (dataFile) {
            dataFile.println(csvData);
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
            if (isArmed) {
                Serial.println("Armed!");
                Serial2.println("Armed!");
            } else {
                hasLaunched = false;
                Serial.println("Disarmed!");
                Serial2.println("Disarmed!");
            }
        }
    }

    delay(BNO055_SAMPLERATE_DELAY_MS);
}


String serializeFlightData(const FlightData& data) {
    String output = "";
    
    // Timestamp
    output += "Timestamp: " + String(data.timestamp) + "\n";
    
    // BNO055 Data
    output += "BNO055 Orientation: " 
             + String(data.bno055.orientation[0]) + "," 
             + String(data.bno055.orientation[1]) + "," 
             + String(data.bno055.orientation[2]) + "\n";
    
    output += "BNO055 Linear Acceleration: " 
             + String(data.bno055.linear_acceleration[0]) + "," 
             + String(data.bno055.linear_acceleration[1]) + "," 
             + String(data.bno055.linear_acceleration[2]) + "\n";
    
    output += "BNO055 Angular Velocity: "
                + String(data.bno055.angular_velocity[0]) + ","
                + String(data.bno055.angular_velocity[1]) + ","
                + String(data.bno055.angular_velocity[2]) + "\n";
    
    output += "calibration: " 
             + "system:" + String(data.bno055.calibration[0]) + "," 
             + "gyro:"   + String(data.bno055.calibration[1]) + "," 
             + "accel:"  + String(data.bno055.calibration[2]) + "," 
             + "mag:"    + String(data.bno055.calibration[3]) + "\n";

    output += "BNO055 Board Temperature: " + String(data.bno055.board_temperature) + "\n";

    // DPS310 Data
    output += "DPS310 Temperature: " + String(data.dps310.temperature) + "\n";
    output += "DPS310 Pressure: " + String(data.dps310.pressure) + "\n";

    // BMP280 Data
    output += "BMP280 Temperature: " + String(data.bmp280.temperature) + "\n";
    output += "BMP280 Pressure: " + String(data.bmp280.pressure) + "\n";
    output += "BMP280 Altitude: " + String(data.bmp280.altitude) + "\n";

    return output;
}


String serializeToCSV(const FlightData& data) {
    // Create a CSV string with all sensor data
    String csvLine = 
        // Timestamp
        String(data.timestamp) + "," +
        
        // BNO055 Orientation (Euler angles)
        String(data.bno055.orientation[0]) + "," +
        String(data.bno055.orientation[1]) + "," +
        String(data.bno055.orientation[2]) + "," +
        
        // Angular Velocity
        String(data.bno055.angular_velocity[0]) + "," +
        String(data.bno055.angular_velocity[1]) + "," +
        String(data.bno055.angular_velocity[2]) + "," +
        
        // Linear Acceleration
        String(data.bno055.linear_acceleration[0]) + "," +
        String(data.bno055.linear_acceleration[1]) + "," +
        String(data.bno055.linear_acceleration[2]) + "," +
        
        // Magnetometer
        String(data.bno055.magnetometer[0]) + "," +
        String(data.bno055.magnetometer[1]) + "," +
        String(data.bno055.magnetometer[2]) + "," +
        
        // Raw Accelerometer
        String(data.bno055.accelerometer[0]) + "," +
        String(data.bno055.accelerometer[1]) + "," +
        String(data.bno055.accelerometer[2]) + "," +
        
        // Gravity Vector
        String(data.bno055.gravity[0]) + "," +
        String(data.bno055.gravity[1]) + "," +
        String(data.bno055.gravity[2]) + "," +
        
        // Calibration Levels
        String(data.bno055.calibration[0]) + "," +
        String(data.bno055.calibration[1]) + "," +
        String(data.bno055.calibration[2]) + "," +
        String(data.bno055.calibration[3]) + "," +
        
        // Board Temperature
        String(data.bno055.board_temperature) + "," +
        
        // DPS310 Sensor Data
        String(data.dps310.temperature) + "," +
        String(data.dps310.pressure) + "," +
        
        // BMP280 Sensor Data
        String(data.bmp280.temperature) + "," +
        String(data.bmp280.pressure) + "," +
        String(data.bmp280.altitude);
    
    return csvLine;
}

// Optional: CSV Header Generation Function
String generateCSVHeader() {
    return 
        "Timestamp," +
        "BNO055_Orientation_X,BNO055_Orientation_Y,BNO055_Orientation_Z," +
        "BNO055_AngularVelocity_X,BNO055_AngularVelocity_Y,BNO055_AngularVelocity_Z," +
        "BNO055_LinearAccel_X,BNO055_LinearAccel_Y,BNO055_LinearAccel_Z," +
        "BNO055_Magnetometer_X,BNO055_Magnetometer_Y,BNO055_Magnetometer_Z," +
        "BNO055_Accelerometer_X,BNO055_Accelerometer_Y,BNO055_Accelerometer_Z," +
        "BNO055_Gravity_X,BNO055_Gravity_Y,BNO055_Gravity_Z," +
        "BNO055_Calib_System,BNO055_Calib_Gyro,BNO055_Calib_Accel,BNO055_Calib_Mag," +
        "BNO055_BoardTemp," +
        "DPS310_Temperature,DPS310_Pressure," +
        "BMP280_Temperature,BMP280_Pressure,BMP280_Altitude";
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