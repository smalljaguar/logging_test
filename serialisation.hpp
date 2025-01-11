#include <stdio.h>
#include "sensors.hpp"

String serialiseFlightData(struct FlightData &flight_data) {
    String output = "";

    // Timestamp
    output += "Timestamp: " + String(flight_data.timestamp) + "\n";

    // BNO055 Data
    // output += "BNO055 Orientation: " + String(flight_data.bno055.orientation[0]) + "," + String(flight_data.bno055.orientation[1]) + "," + String(flight_data.bno055.orientation[2]) + "\n";

    // output += "BNO055 Linear Acceleration: " + String(flight_data.bno055.linear_acceleration[0]) + "," + String(flight_data.bno055.linear_acceleration[1]) + "," + String(flight_data.bno055.linear_acceleration[2]) + "\n";

    // output += "BNO055 Angular Velocity: " + String(flight_data.bno055.angular_velocity[0]) + "," + String(flight_data.bno055.angular_velocity[1]) + "," + String(flight_data.bno055.angular_velocity[2]) + "\n";

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
