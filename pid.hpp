#pragma once

/* 
initially start off with a proportional controller, then try adding derivative (if data isn't too noisy), 
probably don't bother with integral term
use https://github.com/tomstewart89/Geometry for quaternions
*/


#include <limits>
#include <algorithm>

class PIDController {
private:
    float kp, ki, kd;
    float setpoint;
    float output_min, output_max;
    float integral;
    float prev_error;

public:
    /* recommended to set ki to zero, for now also probably kd to zero */
    /* output_min and output_max are probs no more than 10-15 deg for canards?*/
    PIDController(float kp, float ki, float kd, float setpoint = 0.0f, 
                  float output_min = -std::numeric_limits<float>::infinity(),
                  float output_max = std::numeric_limits<float>::infinity())
        : kp(kp), ki(ki), kd(kd), setpoint(setpoint), 
          output_min(output_min), output_max(output_max),
          integral(0.0f), prev_error(0.0f) {}

    float update(float measurement, float dt) {
        float error = setpoint - measurement;
        integral += error * dt;
        float derivative = (dt > 0) ? (error - prev_error) / dt : 0.0f; /* seems sus, why would dt be 0? */
        float output = kp * error + ki * integral + kd * derivative;
        output = std::clamp(output, output_min, output_max);
        prev_error = error;
        return output;
    }
};

PIDController pitch_pid(4.0f, 0.0f, 1.0f, 0.0f, -1.0f, 1.0f);
PIDController yaw_pid(4.0f, 0.0f, 1.0f, 0.0f, -1.0f, 1.0f);
PIDController roll_pid(4.0f, 0.0f, 1.0f, 0.0f, -1.0f, 1.0f);


float curr_time = millis();
float old_time  = millis();
float dt;

float* findCanardAngles(struct FlightData &rocketState){
    old_time = curr_time;
    curr_time = millis();
    dt = curr_time - old_time;
    // use BNO055 Orientation (Euler angles)
    float pitch = rocketState.bno055.orientation[0];
    float yaw = rocketState.bno055.orientation[1];
    float roll = rocketState.bno055.orientation[2];
    pitch_pid.update(pitch,dt);
    yaw_pid.update(yaw,dt);
    roll_pid.update(roll,dt);

    static float angles[4];
    angles[0] = yaw     + roll;
    angles[1] = pitch   + roll;
    angles[2] = - yaw   + roll;
    angles[3] = - pitch + roll;
    
    return angles;
}