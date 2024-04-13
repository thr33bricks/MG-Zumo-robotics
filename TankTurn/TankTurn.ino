#include <Wire.h>
#include <ZumoShield.h>

#define LED_PIN 13

ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);

int speed = 250;

void setup(){
  Serial.begin(9600);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 1); // Turn the LED on

  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);
}

void loop(){
  // If button is not pressed we won't continue
  Serial.println("Press the button to continue!");
  if (!button.getSingleDebouncedRelease()){
    return;
  }
  
  // left motor goes backwards, right - forward
  motors.setSpeeds(-speed, speed);
  delay(500);

  // stop
  motors.setSpeeds(0, 0);
}
  
