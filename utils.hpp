#pragma once
float sqMag(float v[3]) {
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

float magnitude(float v[3]) {
    return sqrt(sqMag(v));
}