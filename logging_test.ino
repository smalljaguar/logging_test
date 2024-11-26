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

//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp;
Adafruit_Sensor *dps_pressure;

Adafruit_BMP280 bmp;  // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

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

float sqLen(float v[3]) {
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

float len(float v[3]) {
    return sqrt(sqLen(v));
}

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

struct FlightData {
    unsigned long timestamp;  // milliseconds since launch

    // BNO055 Sensor Data (9-axis IMU)
    struct {
        float orientation[3];          // Euler angles (x, y, z) can change to quaternion if needed
        float angular_velocity[3];     // Gyroscope data (x, y, z)
        float linear_acceleration[3];  // Linear acceleration (x, y, z)
        float magnetometer[3];         // Magnetometer readings (x, y, z)
        float accelerometer[3];        // Raw accelerometer data (x, y, z)
        float gravity[3];              // Gravity vector (x, y, z)
        uint8_t calibration[4];        // Calibration levels [system, gyro, accel, mag]
        int8_t board_temperature;      // BNO055 board temperature
    } bno055;
    // 3 bytes padding here to stay aligned to 4-byte boundary

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

void collectSensorData(struct FlightData &flight_data) {
    // Timestamp
    flight_data.timestamp = millis();

    // BNO055 Sensor Readings
    sensors_event_t orientationData, angVelocityData, linearAccelData,
        magnetometerData, accelerometerData, gravityData;

    // Orientation (Euler angles)
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    flight_data.bno055.orientation[0] = orientationData.orientation.x;
    flight_data.bno055.orientation[1] = orientationData.orientation.y;
    flight_data.bno055.orientation[2] = orientationData.orientation.z;

    // Angular Velocity (Gyroscope)
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    flight_data.bno055.angular_velocity[0] = angVelocityData.gyro.x;
    flight_data.bno055.angular_velocity[1] = angVelocityData.gyro.y;
    flight_data.bno055.angular_velocity[2] = angVelocityData.gyro.z;

    // Linear Acceleration
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    flight_data.bno055.linear_acceleration[0] = linearAccelData.acceleration.x;
    flight_data.bno055.linear_acceleration[1] = linearAccelData.acceleration.y;
    flight_data.bno055.linear_acceleration[2] = linearAccelData.acceleration.z;

    // Magnetometer
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    flight_data.bno055.magnetometer[0] = magnetometerData.magnetic.x;
    flight_data.bno055.magnetometer[1] = magnetometerData.magnetic.y;
    flight_data.bno055.magnetometer[2] = magnetometerData.magnetic.z;

    // Raw Accelerometer
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    flight_data.bno055.accelerometer[0] = accelerometerData.acceleration.x;
    flight_data.bno055.accelerometer[1] = accelerometerData.acceleration.y;
    flight_data.bno055.accelerometer[2] = accelerometerData.acceleration.z;

    // Gravity Vector
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    flight_data.bno055.gravity[0] = gravityData.acceleration.x;
    flight_data.bno055.gravity[1] = gravityData.acceleration.y;
    flight_data.bno055.gravity[2] = gravityData.acceleration.z;

    // Calibration Levels
    bno.getCalibration(&flight_data.bno055.calibration[0],
                       &flight_data.bno055.calibration[1],
                       &flight_data.bno055.calibration[2],
                       &flight_data.bno055.calibration[3]);

    // Board Temperature (optional)
    flight_data.bno055.board_temperature = bno.getTemp();

    // DPS310 Sensor Readings
    sensors_event_t dpsTempData, dpsPressureData;

    // I really don't like this, but it doesn't work without this
    // might have to make something better for higher frequency readings
    if (int(millis() / 1000) % 2 == 0) {
        if (dps.temperatureAvailable()) {
            dps_temp->getEvent(&dpsTempData);
            flight_data.dps310.temperature = dpsTempData.temperature;
        }
    } else {
        if (dps.pressureAvailable()) {
            dps_pressure->getEvent(&dpsPressureData);
            flight_data.dps310.pressure = dpsPressureData.pressure;
        }
    }

    // BMP280 Sensor Readings
    sensors_event_t bmpTempData, bmpPressureData;
    bmp_temp->getEvent(&bmpTempData);
    bmp_pressure->getEvent(&bmpPressureData);
    flight_data.bmp280.temperature = bmpTempData.temperature;
    flight_data.bmp280.pressure = bmpPressureData.pressure;
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

    if (!dps.begin_I2C(0x77, &Wire)) {
        Serial.println("Bayes DPS not detected");
        while (1);
    }
    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

    if (!bmp.begin(0x76, 88)) {
        Serial.println("Could not find a valid BMP280 sensor!");
        while (1) delay(10);
    }

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    Serial2.begin(9600);  // attempt to set this to 115200 (should work automagically?)
    int startTime = millis();
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
    unsigned int fileCnt = 0;
    unsigned int maxFileCnt = 256;
#ifdef NANDFLASH
    do {
        sprintf(filename, "%s%d.txt", baseFilename, fileCnt);
        fileCnt++;
    } while (myfs.exists(filename) && fileCnt < maxFileCnt);
    dataFile = myfs.open(filename, FILE_WRITE);
#else
    do {
        sprintf(filename, "%s%d.txt", baseFilename, fileCnt);
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
    Serial2.println(len(acceleration));
    Serial.println(len(acceleration));

    if (!hasLaunched && isArmed) {
        float *accel = linearAccelData.acceleration.v;
        if (len(accel) > 0 && len(accel) < 150) {  // units are SI (ms^-2), 150 is sanity check
            hasLaunched = true;
            Serial2.println("We have liftoff!");
            launchTime = millis();

            if (dataFile) {
                dataFile.println(generateCSVHeader());
            }
        }
    }

    if (hasLaunched && (millis() - launchTime < TIME_TO_CHUTE)) {
        FlightData flight_data;
        collectSensorData(flight_data);
        // MAKE SURE TO DISABLE FOR LAUNCH!!
        String dataBuf = serialiseFlightData(flight_data);
        Serial.println(dataBuf);
        Serial2.println(dataBuf);
        String csvData = serialiseToCSV(flight_data);
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

String serialiseFlightData(struct FlightData &flight_data) {
    String output = "";

    // Timestamp
    output += "Timestamp: " + String(flight_data.timestamp) + "\n";

    // BNO055 Data
    output += "BNO055 Orientation: " + String(flight_data.bno055.orientation[0]) + "," + String(flight_data.bno055.orientation[1]) + "," + String(flight_data.bno055.orientation[2]) + "\n";

    output += "BNO055 Linear Acceleration: " + String(flight_data.bno055.linear_acceleration[0]) + "," + String(flight_data.bno055.linear_acceleration[1]) + "," + String(flight_data.bno055.linear_acceleration[2]) + "\n";

    output += "BNO055 Angular Velocity: " + String(flight_data.bno055.angular_velocity[0]) + "," + String(flight_data.bno055.angular_velocity[1]) + "," + String(flight_data.bno055.angular_velocity[2]) + "\n";

    output +=
        "calibration:\n"
        "system:" +
        String(flight_data.bno055.calibration[0]) + "," + "gyro:" + String(flight_data.bno055.calibration[1]) + "," + "accel:" + String(flight_data.bno055.calibration[2]) + "," + "mag:" + String(flight_data.bno055.calibration[3]) + "\n";

    output += "BNO055 Board Temperature: " + String(flight_data.bno055.board_temperature) + "\n";

    // DPS310 Data
    output += "DPS310 Temperature: " + String(flight_data.dps310.temperature) + "\n";
    output += "DPS310 Pressure: " + String(flight_data.dps310.pressure) + "\n";

    // BMP280 Data
    output += "BMP280 Temperature: " + String(flight_data.bmp280.temperature) + "\n";
    output += "BMP280 Pressure: " + String(flight_data.bmp280.pressure) + "\n";
    output += "BMP280 Altitude: " + String(flight_data.bmp280.altitude) + "\n";

    return output;
}

String serialiseToCSV(struct FlightData &flight_data) {
    // Create a CSV string with all sensor data
    String csvLine =
        // Timestamp
        String(flight_data.timestamp) + "," +

        // BNO055 Orientation (Euler angles)
        String(flight_data.bno055.orientation[0]) + "," +
        String(flight_data.bno055.orientation[1]) + "," +
        String(flight_data.bno055.orientation[2]) + "," +

        // Angular Velocity
        String(flight_data.bno055.angular_velocity[0]) + "," +
        String(flight_data.bno055.angular_velocity[1]) + "," +
        String(flight_data.bno055.angular_velocity[2]) + "," +

        // Linear Acceleration
        String(flight_data.bno055.linear_acceleration[0]) + "," +
        String(flight_data.bno055.linear_acceleration[1]) + "," +
        String(flight_data.bno055.linear_acceleration[2]) + "," +

        // Magnetometer
        String(flight_data.bno055.magnetometer[0]) + "," +
        String(flight_data.bno055.magnetometer[1]) + "," +
        String(flight_data.bno055.magnetometer[2]) + "," +

        // Raw Accelerometer
        String(flight_data.bno055.accelerometer[0]) + "," +
        String(flight_data.bno055.accelerometer[1]) + "," +
        String(flight_data.bno055.accelerometer[2]) + "," +

        // Gravity Vector
        String(flight_data.bno055.gravity[0]) + "," +
        String(flight_data.bno055.gravity[1]) + "," +
        String(flight_data.bno055.gravity[2]) + "," +

        // Calibration Levels
        String(flight_data.bno055.calibration[0]) + "," +
        String(flight_data.bno055.calibration[1]) + "," +
        String(flight_data.bno055.calibration[2]) + "," +
        String(flight_data.bno055.calibration[3]) + "," +

        // Board Temperature
        String(flight_data.bno055.board_temperature) + "," +

        // DPS310 Sensor Data
        String(flight_data.dps310.temperature) + "," +
        String(flight_data.dps310.pressure) + "," +

        // BMP280 Sensor Data
        String(flight_data.bmp280.temperature) + "," +
        String(flight_data.bmp280.pressure) + "," +
        String(flight_data.bmp280.altitude);

    return csvLine;
}

String serialiseVecToJSON(float vec3[3]) {
    String json = "{";
    json += "\"x\": " + String(vec3[0]) + ",";
    json += "\"y\": " + String(vec3[1]) + ",";
    json += "\"z\": " + String(vec3[2]);
    json += "}";
    return json;
}

String serialiseToJSON(struct FlightData &flight_data) {
    String json = "{";
    json += "\"timestamp\": " + String(flight_data.timestamp) + ",";
    json += "\"bno055\": {";
    json += "\"orientation\": " + serialiseVecToJSON(flight_data.bno055.orientation) + ",";
    json += "\"angular_velocity\": " + serialiseVecToJSON(flight_data.bno055.angular_velocity) + ",";
    json += "\"linear_acceleration\": " + serialiseVecToJSON(flight_data.bno055.linear_acceleration) + ",";
    json += "\"magnetometer\": " + serialiseVecToJSON(flight_data.bno055.magnetometer) + ",";
    json += "\"accelerometer\": " + serialiseVecToJSON(flight_data.bno055.accelerometer) + ",";
    json += "\"gravity\": " + serialiseVecToJSON(flight_data.bno055.gravity) + ",";
    json += "\"calibration\": {";
    json += "\"system\": " + String(flight_data.bno055.calibration[0]) + ",";
    json += "\"gyro\": " + String(flight_data.bno055.calibration[1]) + ",";
    json += "\"accel\": " + String(flight_data.bno055.calibration[2]) + ",";
    json += "\"mag\": " + String(flight_data.bno055.calibration[3]);
    json += "},";
    json += "\"board_temperature\": " + String(flight_data.bno055.board_temperature);
    json += "},";
    json += "\"dps310\": {";
    json += "\"temperature\": " + String(flight_data.dps310.temperature) + ",";
    json += "\"pressure\": " + String(flight_data.dps310.pressure);
    json += "},";
    json += "\"bmp280\": {";
    json += "\"temperature\": " + String(flight_data.bmp280.temperature) + ",";
    json += "\"pressure\": " + String(flight_data.bmp280.pressure) + ",";
    json += "\"altitude\": " + String(flight_data.bmp280.altitude);
    json += "}";
    json += "}";
    return json;
}

// use write instead of print to avoid null terminator
unsigned char *serialiseToRaw(struct FlightData &flight_data) {
    unsigned char *rawData = (unsigned char *)malloc(sizeof(struct FlightData));
    memcpy(rawData, &flight_data, sizeof(struct FlightData));
    return rawData;
}

// Optional: CSV Header Generation Function
String generateCSVHeader() {
    return String("Timestamp,") +  // String call necessary for making sure type coercion works
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

// String serialiseToJSON(struct FlightData& flight_data) {}

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
