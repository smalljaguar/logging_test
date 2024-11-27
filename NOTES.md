CALIBRATION:

Gyroscope: The device must be standing still in any position
Magnetometer: In the past 'figure 8' motions were required in 3 dimensions, but with recent devices fast magnetic compensation takes place with sufficient normal movement of the device
Accelerometer: The BNO055 must be placed in 6 standing positions for +X, -X, +Y, -Y, +Z and -Z.  This is the most onerous sensor to calibrate, but the best solution to generate the calibration data is to find a block of wood or similar object, and place the sensor on each of the 6 'faces' of the block, which will help to maintain sensor alignment during the calibration process.  You should still be able to get reasonable quality data from the BNO055, however, even if the accelerometer isn't entirely or perfectly calibrated.




Example code this is hacked together from:
- BNO055 read_all_data
- Teensy SD Datalogger
- HC-05_ATMode
- bmp280_sensortest

Datasheets:
- [DPS310](https://www.infineon.com/dgdl/Infineon-DPS310-DataSheet-v01_02-EN.pdf?fileId=5546d462576f34750157750826c42242)
- [BNO055](https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf)
- [BMP280](https://cdn-shop.adafruit.com/datasheets/BST-BMP280-DS001-11.pdf)

Calculated data output rate:
- Sensors outputs 3 sig figs
- we are logging x,y,z for orientation, angular velocity, acceleration 
- sample rate upper bound of 100Hz
- so we have 3*3 numbers, which each take 4 bytes => 36 bytes
- add overhead of labels, say another 8 bytes per event => 36 + 24 = 60 bytes
- 60 bytes, 100 Hz => 6KB/s upper bound
- 5 minutes flight time (say) => no more than 2MB of logs expected

TODO:
- update burn time to be more accurate (Gui said shouldn't not be necessary, SD card is massive compared to logfile sizes)
- DPS now working but BMP temp stuck, need to fix
- store calibration with get/setSensorOffsets
- test csv/json/raw bytes decoding to see if reliable

NB:
- to change logging rate, change `BNO055_SAMPLERATE_DELAY_MS`, default is 100ms
  (max sensor freq is 100Hz)
- check baud rate if logging doesn't work, can try setting it to a more common 9600 instead of 115200
- probably want to log in Euler angle (degrees), but do calculations with quaternions, 
  could just log both, idk if there's a better way to handle this
- check if pinouts are configured correctly
- NAND Flash is enabled by defining macro NANDFLASH
- if adafruit BMP280 lib doesn't work, try DFRobot one
- consider using takeforcedmeasurement if bmp doesn't work
- units are all SI (ms^-1, ms^-2, Pa)
- DPS310 fails to return pressure if temperature has been read recently 
- If uploading fails try other usb-c port on laptop (I think one might be power-only)
- ON LAUNCH DISABLE SERIAL PRINTS!!
- 500 bytes per sample, at 10Hz and 1GB max log size that gives 48 hours which means totally fine

Simple optimisations:
- If running out of space in RAM, try wrapping constant strings in `F` macro which puts them in flash (read-only)
- log getCalibration only when it's not 3 and/or it changes, and/or at a slower rate
- preallocate estimated string len
Medium optimisations:
- switch from SD lib to SDFAT which uses exFAT, buffers things for you
- instead of creating new buffer string each time, reuse the same one, clearing it at the end of each loop
- compress logs somehow
- protobuf/actual libs for serialisation


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



--

https://www.pjrc.com/teensy/td_uart.html

CSV Format:
field_a,field_b,field_c
item_a,item_b,item_c
...

I2C device found at address 0x28 
I2C device found at address 0x76  
I2C device found at address 0x77  

