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
  
  // run forward
  motors.setSpeeds(speed, speed);
  delay(500);

  //stop
  motors.setSpeeds(0, 0);
  delay(500);

  // run backwards
  motors.setSpeeds(-speed, -speed);
  delay(500);

  //stop
  motors.setSpeeds(0, 0);

  // blink
  for(int i=0; i < 10; i++){
    digitalWrite(LED_PIN, 0);
    delay(100);
    digitalWrite(LED_PIN, 1);
    delay(100);
  }
}
  
