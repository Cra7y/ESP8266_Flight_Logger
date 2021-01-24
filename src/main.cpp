/* #region  Includes */
#include <Arduino.h> // Include main arduino lib

// Include other libs
#include <servo.h> // Servo
#include <Wire.h>  // I2C
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
/* #endregion */

/* #region  Defines */
#define SERVOPIN 16    // Pin that servo is connected to
#define SERVOHOME 180  // The position where the servo holds parachute in
#define SERVORELEASE 0 // The position where the servo releases parachute

#define WEIGHTLESSTRESHOLD 1.0 // The treshold for when we asume weightlessness in m/s^2
#define INFLIGHTTRESHOLD 30    // The treshold for when we asume device has left the ramp in m/s^2
/* #endregion */

/* #region  Global variables */
unsigned long nextChange = 0; //LED Control

// Flight variables
bool top = false;
bool inFlight = false;
unsigned int startTime;
unsigned int topTime;
int landedCounter = 0;
/* #endregion */

/* #region  Global functions */
// Check if devie is weightless
bool isWeightless(float sum)
{
  if (sum <= WEIGHTLESSTRESHOLD)
  {
    return true;
  }
  return false;
}
/* #endregion */

/* #region  Servo global variables and functions */
Servo myservo; // create servo object to control a servo
int pos = 0;   // variable to store the servo position
void activateServo()
{
  if (pos == 0)
  {
    pos = 180;
  }
  else
  {
    pos = 0;
  }
  myservo.write(pos);
}
/* #endregion */

/* #region  ADXL 345 global variables and functions */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
// Start accelerometer
void beginAccel()
{
  if (!accel.begin())
  { // Start accelerometer
    // There was a problem detecting the ADXL345 ... check your connections
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while (1)
      ;
  }
}
// Get event
sensors_event_t getEvent()
{
  sensors_event_t event;
  accel.getEvent(&event);
}
// Get sum of forces
float getSum(sensors_event_t event)
{
  return sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
}
// Log sensor event
void logEvent(sensors_event_t event)
{
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: ");
  Serial.print(event.acceleration.x);
  Serial.print(" ");
  Serial.print("Y: ");
  Serial.print(event.acceleration.y);
  Serial.print(" ");
  Serial.print("Z: ");
  Serial.print(event.acceleration.z);
  Serial.print(" ");
  Serial.print("SUM: ");
  Serial.print(getSum(event));
  Serial.print(" ");
  Serial.println("m/s^2 ");
}
/* #endregion */

// the setup function runs once when you press reset or power the board
void setup()
{
  // Setup libs
  myservo.attach(SERVOPIN); // attaches the servo on pin 2 (D4) to the servo object
  Serial.begin(115200);     // Start serial
  beginAccel();             // Start accelerometer

  // Setup pins
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.

  // Set the range of accelerometer to +-16G
  accel.setRange(ADXL345_RANGE_16_G);

  Serial.println("Started");
}

// the loop function runs over and over again forever
void loop()
{
  // Get a new ADXL345 sensor event
  sensors_event_t event = getEvent();
  float sum = getSum(event);

  // Check if device is weightless (at the top of arch)
  if (isWeightless(sum))
  {
    activateServo();
  }

  // Update LED and log current sensor data
  if (nextChange <= millis())
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    logEvent(event);
    nextChange = nextChange + 1000;
  }

  delay(100);
}