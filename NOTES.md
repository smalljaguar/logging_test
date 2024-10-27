Example code this is hacked together from:
- BNO055 read_all_data
- Teensy SD Datalogger
- HC-05_ATMode


Calculated data output rate:
- Sensors probably can't output better than 5 sig figs (can test this)
- we are logging x,y,z for orientation, angular velocity, acceleration 
- sample rate upper bound of 100Hz
- so we have 3*3 numbers, which each take 8 bytes => 72 bytes
- add overhead of labels, say another 8 bytes per event => 72 + 24 = 96 bytes
- 96 bytes, 100 Hz => 10KB/s upper bound
- 5 minutes flight time (say) => no more than 3MB of logs expected

TODO:
- consider logging relative timestamps for easier reading of logfiles by hand
- attempt to check for liftoff by checking if linear acceleration is over threshold
- do we care about magnetometer data at all? (remember sensor already does data fusion for us)
- update burn time to be accurate

NB:
- to change logging rate, change `BNO055_SAMPLERATE_DELAY_MS`, default is 100ms
  (max sensor freq is 100Hz)
- check baud rate if logging doesn't work, can try setting it to a more common 9600 instead of 115200
- probably want to log in Euler angle (degrees), but do calculations with quaternions, 
  could just log both, idk if there's a better way to handle this
- check if pinouts are configured correctly


Simple optimisations:
- If running out of space in RAM, try wrapping constant strings in `F` macro which puts them in flash (read-only)
- log getCalibration only when it's not 3 and/or it changes, and/or at a slower rate
- open file in setup not loop, then flush instead of closing
- preallocate estimated string len
- truncate logged decimal places (second param of `String` is decimalPlaces)
Medium optimisations:
- switch from SD lib to SDFAT which uses exFAT, buffers things for you
- instead of creating new buffer string each time, reuse the same one, clearing it at the end of each loop
- compress logs somehow
Complex optimisations:
- switch to binary logging format with separate decoding


BNO055 Info
Output Data:
    Absolute Orientation (Euler Vector, 100Hz)
        Three axis orientation data based on a 360° sphere
    Absolute Orientation (Quaterion, 100Hz)
        Four point quaternion output for more accurate data manipulation
    Angular Velocity Vector (100Hz)
        Three axis of 'rotation speed' in rad/s
    Acceleration Vector (100Hz)
        Three axis of acceleration (gravity + linear motion) in m/s^2
    Magnetic Field Strength Vector (20Hz)
        Three axis of magnetic field sensing in micro Tesla (uT)
    Linear Acceleration Vector (100Hz)
        Three axis of linear acceleration data (acceleration minus gravity) in m/s^2
    Gravity Vector (100Hz)
        Three axis of gravitational acceleration (minus any movement) in m/s^2
    Temperature (1Hz)
        Ambient temperature in degrees celsius
