#include <Arduino.h> // Include main arduino lib

// Include other libs
#include <servo.h> //Servo
#include <Wire.h> // I2C
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// Servo global variables
Servo myservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
void activateServo() {
  if (pos == 0) {
      pos = 180;
    } else {
      pos = 0;
    }
    myservo.write(pos);
}


// ADXL 345 global variables
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void checkWeightless() {
  
}

unsigned long nextChange = 0;

// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.
  myservo.attach(16);  // attaches the servo on pin 2 (D4) to the servo object
  Serial.begin(115200);
  Serial.println("Started");

  /* Initialise the sensor */
 if(!accel.begin())
 {
 /* There was a problem detecting the ADXL345 ... check your connections */
 Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
 while(1);
 }
 
 /* Set the range to whatever is appropriate for your project */
 accel.setRange(ADXL345_RANGE_16_G);
 // displaySetRange(ADXL345_RANGE_8_G);
 // displaySetRange(ADXL345_RANGE_4_G);
 // displaySetRange(ADXL345_RANGE_2_G);

 Serial.println("");
}

// the loop function runs over and over again forever
void loop() {
  
  if (nextChange <= millis()) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  /* Get a new sensor event */ 
 sensors_event_t event; 
 accel.getEvent(&event);
 
 /* Display the results (acceleration is measured in m/s^2) */
 Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print(" ");
 Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print(" ");
 Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print(" "); Serial.print("m/s^2 "); Serial.print(" ");
 Serial.println(sqrt(pow(event.acceleration.x, 2)+pow(event.acceleration.y, 2)+pow(event.acceleration.z, 2)));

    nextChange = nextChange + 1000;
  }
  
}