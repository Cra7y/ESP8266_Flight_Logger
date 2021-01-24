#include <Arduino.h> // Include main arduino lib

// Include other libs
#include <servo.h>

Servo myservo;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position

unsigned long nextChange = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
  myservo.attach(16);  // attaches the servo on pin 2 (D4) to the servo object
  Serial.begin(115200);
  Serial.println("Started");
}

// the loop function runs over and over again forever
void loop() {
  
  if (nextChange <= millis()) {
    Serial.println("Switching led");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    if (pos == 0) {
      pos = 180;
    } else {
      pos = 0;
    }
    myservo.write(pos);
    nextChange = nextChange + 1000;
  }
  
}