#include <Servo.h>
/* servo technical spec:
 https://cdn.shopify.com/s/files/1/0570/1766/3541/files/DS215MG_V8.0Technical_Specifcation..pdf?v=1700473154 
 */

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void initServos(){
    // TODO: Correct pin vals
    servo1.attach(15);
    servo2.attach(16);
    servo3.attach(17);
    servo4.attach(18);
}

void writeServos(float *servoAngles){
    servo1.write(servoAngles[0]);
    servo2.write(servoAngles[1]);
    servo3.write(servoAngles[2]);
    servo4.write(servoAngles[3]);
}