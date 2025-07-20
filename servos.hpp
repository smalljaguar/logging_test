#include <Servo.h>
/* servo technical spec:
 https://cdn.shopify.com/s/files/1/0570/1766/3541/files/DS215MG_V8.0Technical_Specifcation..pdf?v=1700473154 
 */

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void initServos(){
    // TODO: check if pin vals correct 
    servo1.attach(37);
    servo2.attach(14);
    servo3.attach(36);
    servo4.attach(33);
}

const int MIN_ANGLE = -60;
const int MAX_ANGLE = 60;
const int HOME_ANGLE = 0;
const int MOVE_DELAY = 1000; // Delay between movements (ms)
const int SERVO_DELAY = 2000; // Delay between servo tests (ms)

void testServo(Servo &servo, int servoNum) {
  Serial.printf("Testing Servo %d:\n", servoNum);
  
  Serial.printf("  Moving to %d degrees\n", MIN_ANGLE);
  servo.write(90 + MIN_ANGLE); // Convert -60 to servo range (30)
  delay(MOVE_DELAY);
  
  Serial.printf("  Moving to %d degrees\n", MAX_ANGLE);
  servo.write(90 + MAX_ANGLE); // Convert +60 to servo range (150)
  delay(MOVE_DELAY);
  
  // Move to 0 degrees
  Serial.printf("  Moving to %d degrees (home)\n", HOME_ANGLE);
  servo.write(90 + HOME_ANGLE); // Convert 0 to servo range (90)
  delay(MOVE_DELAY);
  
  Serial.printf("  Servo %d test complete\n\n", servoNum);
}

void writeServos(float *servoAngles){
    servo1.write(servoAngles[0]);
    servo2.write(servoAngles[1]);
    servo3.write(servoAngles[2]);
    servo4.write(servoAngles[3]);
}