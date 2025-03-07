#pragma once

/* 
initially start off with a proportional controller, then try adding derivative (if data isn't too noisy), 
probably don't bother with integral term
use https://github.com/RCmags/vector_datatype/tree/main for quaternions 
*/

/*

*/

float PID(float target, float current, float old, float dt, float integral) {
    float prop_gain = 1;
    float int_gain = 0;
    float deriv_gain = 0;
    float error = target - current;
    integral += error * dt;
    float derivative = (current - old) / dt;
    return prop_gain * error + int_gain * integral + deriv_gain * derivative;
}
float error = 0;
