/* #region  Includes */
#include <Arduino.h> // Include main arduino lib
#include <SPI.h>

// Include other libs
#include <servo.h>              // Servo
#include <Wire.h>               // I2C
#include <Adafruit_Sensor.h>    // Unified sensor
#include <Adafruit_ADXL345_U.h> // ADXL345
#include <Adafruit_BMP085.h>    // BMP085
#include <FastLED.h>            // WS2812
/* #endregion */

/* #region  Defines */
#define SERVOPIN 16    // Pin that servo is connected to
#define SERVOHOME 180  // The position where the servo holds parachute in
#define SERVORELEASE 0 // The position where the servo releases parachute

#define NUM_LEDS 1 // Number of WS2812 Leds
#define DATA_PIN 0 // WS2812 pin

#define WEIGHTLESSTRESHOLD 1.0 // The treshold for when we asume weightlessness in m/s^2
#define INFLIGHTTRESHOLD 30    // The treshold for when we asume device has left the ramp in m/s^2
#define LANDEDTRESHOLD 15      // The treshold for when we asume device is landed (laying still) in m/s^2

// Startup
#define STARTUPDELAY 10000 // Time before closing servo on startup
// Led delay at diferent states (in ms)
#define LEDSTATE0 500
#define LEDSTATE1 250
#define LEDSTATE2 125

// States
#define STARTUP 0
#define WAITINGFORLAUNCH 1
#define INFLIGHT 2
#define LANDED 3
#define RUNDEBUG 4
/* #endregion */

/* #region  Global variables */
// Flight variables
#ifdef DEBUG
int state = RUNDEBUG;
#else
int state = STARTUP;
#endif
bool top = false; // True after max altitude is reached
unsigned int startTime;
unsigned int topTime;
int landedCounter = 0;

// Log
const int sizeI = 1200;
const int sizeJ = 5;
float flightLog[sizeI][sizeJ];
int logCursor = 0;
bool logOverflow = false;
/* #endregion */

/* #region  LED global variables and functions */
unsigned long nextChange = 0; //LED Control
int ledDelay = LEDSTATE0;     // Delay between led change
int ledState = 0;             // Current state of LED
// Change led delay
void changeLedDelay(int newDelay)
{
  nextChange = millis() + newDelay;
  ledDelay = newDelay;
}
void setLedState(int newState)
{
  if (newState != ledState)
  {
    ledState = newState;
    switch (ledState)
    {
    case 0:
      changeLedDelay(LEDSTATE0);
      break;
    case 1:
      changeLedDelay(LEDSTATE1);
      break;
    case 2:
      changeLedDelay(LEDSTATE2);
      break;
    }
  }
}
// Update LED
void updateLed()
{
  if (nextChange <= millis())
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    nextChange = nextChange + ledDelay;
  }
}
// Set LED to off
void ledOFF()
{
  digitalWrite(LED_BUILTIN, LOW);
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

/* #region  BMP085 global variables and functions */
Adafruit_BMP085 bmp;
double startPressure;
double altitude;
/* #endregion */

/* #region  WS2812 global variables and functions */
CRGB leds[NUM_LEDS];
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
// Map float values
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
// Gets battery voltage
float getBatteryVoltage() {
  return mapf(float(analogRead(A0)), 13.0, 1024.0, 0.0, 4.18);
}
// Create log
void createLog() { // TODO: Ghange to something that makes sense in the field
  Serial.println("Log");
  for (int i = 0; i < sizeI; i++) {
    if (flightLog[i][0] == 0.0) { //TODO: Reconsider conditions
      break;
    }
    Serial.print("Time: ");
    Serial.print(flightLog[i][0]);
    Serial.print("ms ");
    Serial.print("X: ");
    Serial.print(flightLog[i][1]);
    Serial.print(" ");
    Serial.print("Y: ");
    Serial.print(flightLog[i][2]);
    Serial.print(" ");
    Serial.print("Z: ");
    Serial.print(flightLog[i][3]);
    Serial.print(" ");
    Serial.print("SUM: ");
    Serial.print(flightLog[i][4]);
    Serial.print(" ");
    Serial.println("m/s^2 ");
  }
  Serial.print("Rocket reached a topheight of ");
  Serial.print(altitude);
  Serial.print("m at ");
  Serial.print(topTime);
  Serial.println("ms");
  Serial.println("End of log");
}

// Runs once when STARTUPDELAY is reached
void onStartupTimeout() {
  changeLedDelay(1000);
  myservo.write(SERVOHOME);
  startPressure = bmp.readPressure(); // Measure start pressure (Needed to calculate relative preasure later)
  state = WAITINGFORLAUNCH;
}
// Runs once on launch
void onLaunch() {
  state = INFLIGHT;     // Set device to inFligt mode
  startTime = millis(); // Save time of launch
  ledOFF();             // Turn led off
}
// Runs once when top is reached
void onTop(unsigned int functionStart) {
  topTime = functionStart - startTime;        // Caluculate time since launch
  altitude = bmp.readAltitude(startPressure); // Measure max height
  top = true;
  Serial.println("Releasing parachute");
  myservo.write(SERVORELEASE);
}
// Runs once when landed
void onLand() {
  state = LANDED;
  landedCounter = 0;
  top = false;

  createLog();
}

// Runs in flight
void runInFlight()
{
  //Runs while device in inflight
  unsigned int functionStart = millis(); // Keeps track of when function started

  // Get a new ADXL345 sensor event
  sensors_event_t event;
  accel.getEvent(&event);
  float sum = getSum(event);

  // Check if we reached maximum altitude
  if (sum < WEIGHTLESSTRESHOLD && !top)
  {
    onTop(functionStart);
  }

  // Log acceleration variables
  if (logCursor < sizeI)
  {
    flightLog[logCursor][0] = functionStart - startTime; // Time since launch
    flightLog[logCursor][1] = event.acceleration.x;      // Acceleration X
    flightLog[logCursor][2] = event.acceleration.y;      // Acceleration X
    flightLog[logCursor][3] = event.acceleration.z;      // Acceleration X
    flightLog[logCursor][4] = sum;                       // Acceleration resulting force
    logCursor++;
  }
  else
  {
    logOverflow = true;
  }

  // If acceleration is less than LANDEDTRESHOLD for 20 iterations we asume that we have landed
  if (landedCounter < 20 && sum < LANDEDTRESHOLD)
  {
    landedCounter++;
  }
  else if (landedCounter >= 20 && sum < LANDEDTRESHOLD)
  {
    onLand();
  }
  else if (sum > LANDEDTRESHOLD)
  {
    landedCounter = 0;
  }

  // Calculate how much delay is needed to keep constant update rate
  long int functionTime = millis() - functionStart; // Calculate execution time of function
  int remainingDelay = 50 - functionTime;
  // Only delay if function took less than 50ms
  if (remainingDelay > 0)
  {
    delay(remainingDelay);
  }
}
// Runs while waiting to be launched
void waitForLaunch()
{
  // Get a new ADXL345 sensor event
  sensors_event_t event;
  accel.getEvent(&event);
  float sum = getSum(event);

  // Wait until the device hits acceleration defined as INFLIGHTTRESHOLD
  if (sum > INFLIGHTTRESHOLD)
  {
    onLaunch();
  }
  else
  {
    delay(50); // Device was not launched, so we wait
  }
}
/* #endregion */

// the setup function runs once when you press reset or power the board
void setup()
{
  // Setup libs
  myservo.attach(SERVOPIN); // attaches the servo on pin 2 (D4) to the servo object
  Serial.begin(115200);     // Start serial
  beginAccel();             // Start accelerometer
  bmp.begin();              // Start barometer
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);  // FastLED - GRB ordering is assumed

  // Setup pins
  pinMode(LED_BUILTIN, OUTPUT); // initialize digital pin LED_BUILTIN as an output.

  // Set the range of accelerometer to +-16G
  accel.setRange(ADXL345_RANGE_16_G);

  // Set servo to relased position
  myservo.write(SERVORELEASE);

  Serial.println("Started");

  if (state == RUNDEBUG)
  {
    startPressure = bmp.readPressure();
  }
}

// the loop function runs over and over again forever
void loop()
{
  switch (state) {
    case STARTUP: {
      delay(50);
      updateLed();

      unsigned int curTime = millis();
      if (curTime > STARTUPDELAY)
      {
        onStartupTimeout();
      }
      else if (curTime > (STARTUPDELAY / 3) * 2)
      {
        setLedState(2);
      }
      else if (curTime > (STARTUPDELAY / 3))
      {
        setLedState(1);
      }
      else
      {
        setLedState(0);
      }
      break;
    }
    case WAITINGFORLAUNCH: {
      updateLed();
      waitForLaunch();
      break;
    }
    case INFLIGHT: {
      runInFlight();
      break;
    }
    case LANDED: {
      Serial.println("We are landed! TODO: Add some way to retrieve log");
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(500);
      break;
    }
    case RUNDEBUG: {
      // Check WS2812
      leds[0] = CRGB::Red;
      FastLED.show();
      delay(250);

      leds[0] = CRGB::Green;
      FastLED.show();
      delay(250);

      leds[0] = CRGB::Blue;
      FastLED.show();
      delay(250);

      leds[0] = CRGB::Black;
      FastLED.show();
      delay(250);

      // Check BMP185
      Serial.print("Temperature = ");
      Serial.print(bmp.readTemperature());
      Serial.println(" Celsius");

      Serial.print("Pressure = ");
      Serial.print(bmp.readPressure());
      Serial.println(" Pascal");

      Serial.print("Altitude = ");
      Serial.print(bmp.readAltitude(startPressure));
      Serial.println(" Meters");

      // Check ADXL345
      sensors_event_t event;
      accel.getEvent(&event);
      float sum = getSum(event);
      
      Serial.print("Sum of acceleration = ");
      Serial.print(sum);
      Serial.println(" m/s^2");

      Serial.print("Analog reading = ");
      Serial.print(analogRead(A0));
      Serial.println(" int");

      Serial.print("battery = ");
      Serial.print(getBatteryVoltage());
      Serial.println(" v");

      // Check Servo
      Serial.println("Moving servo home");
      myservo.write(SERVOHOME);
      delay(500);
      Serial.println("Moving servo to release");
      myservo.write(SERVORELEASE);
      delay(500);

      Serial.println();
      break;
    }
  }

  //Serial.print("State: "); Serial.print(state); Serial.print("  ");
  //Serial.print("LED State: ");  Serial.println(ledState);
}