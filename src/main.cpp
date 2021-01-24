#include <Arduino.h> // Include main arduino lib

// Include other libs
#include <servo.h>

Servo myservo;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}