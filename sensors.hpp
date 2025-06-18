#pragma once
/* for sensors */
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_DPS310.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <utility/imumaths.h>
#include "utils.hpp"
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
uint16_t SAMPLERATE_DELAY_MS = 20;

//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

Adafruit_DPS310 dps;
Adafruit_Sensor *dps_temp;
Adafruit_Sensor *dps_pressure;
float seaLevelPressure = 0;

Adafruit_BMP280 bmp;  // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

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
    // if (int(millis() / SAMPLERATE_DELAY_MS) % 2 == 0) {
    //     if (dps.temperatureAvailable()) {
    //         dps_temp->getEvent(&dpsTempData);
    //         flight_data.dps310.temperature = dpsTempData.temperature;
    //     }
    //     else {
    //       Serial.println("DPS Temp not available!");
    //     }
    // } else {
    //     if (dps.pressureAvailable()) {
    //         dps_pressure->getEvent(&dpsPressureData);
    //         flight_data.dps310.pressure = dpsPressureData.pressure;
            
    //     }
    //     else {
    //       Serial.println("DPS pressure not available!");
    //     }
    // }

    dps.getEvents(&dpsTempData, &dpsPressureData);
    flight_data.dps310.pressure = dpsPressureData.pressure;
    flight_data.dps310.temperature = dpsTempData.temperature;
    

    // BMP280 Sensor Readings
    sensors_event_t bmpTempData, bmpPressureData;
    bmp_temp->getEvent(&bmpTempData);
    flight_data.bmp280.temperature = bmpTempData.temperature;
    bmp_pressure->getEvent(&bmpPressureData);
    flight_data.bmp280.pressure = bmpPressureData.pressure;
    flight_data.bmp280.altitude = bmp.readAltitude(seaLevelPressure);
}