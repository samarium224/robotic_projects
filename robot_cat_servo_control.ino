#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver ServoDriver = Adafruit_PWMServoDriver();

#define SERVOMIN  125 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  575 // This is the 'maximum' pulse length count (out of 4096)

int servoNumber = 0:

void setup(){
    Serial.begin(9600);
    Serial.println("Adafruit PWM Servo Driver Test!");
    
    ServoDriver.begin();
    ServoDriver.setPWMFreq(60); // Analog servos run at ~60 Hz updates
    
}

void loop(){
    // Sweep the servo from 0 to 180 degrees
    for (int angle = 0; angle <= 180; angle++) {
        for(int i=0; i<9 ; i++){
            ServoDriver.setPWM(i, 0, angleToPulse(angle));
            delay(15); // Wait for the servo to reach the position
        }
    }

    delay(100);
}

int angleToPulse(int ang){
    int pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
    Serial.print("Angle: ");
    Serial.print(ang);
    Serial.print(" Pulse: ");
    Serial.println(pulse);
    return pulse;
}