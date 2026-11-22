//Libraries

#include <DS18B20.h> //For sensor
#include <math.h>


//Pin Config

const int TEMP_PIN = 2;

const int RPWM_PIN = 5;
const int LPWM_PIN = 6;
const int R_EN_PIN = 7;
const int L_EN_PIN = 8;


//Peltier Safety
// Keep disabled until hardware, current limiting,
// wiring, and thermal protections are verified.

#define PELTIER_HARDWARE_VERIFIED 0

//Temp Settings

float targetTemp = 37.0; //Target

const float MIN_TARGET_TEMP = 0.0; //Minimum Target
const float MAX_TARGET_TEMP = 95.0; //Maximum Target
const float MAX_SAFE_TEMP = 100.0; //Max Temp overall (Before emergency shutdown)

//PID Settings

float Kp = 8.0;
float Ki = 0.02;
float Kd = 0.0;

float integral = 0.0;
float previousError = 0.0;

// Derivative filter
const float DERIVATIVE_ALPHA = 0.2;
float filteredDerivative = 0.0;

const int MAX_PWM = 255;

//EMA and Temp Filter

const float FILTER_ALPHA = 0.2;

float rawTemp = NAN;
float filteredTemp = NAN;

bool filterInitialized = false;

//Timing

const unsigned long CONTROL_INTERVAL = 150;

unsigned long previousControlTime = 0;
unsigned long lastSensorReadTime = 0;

//Sensor initialization

DS18B20 ds(TEMP_PIN);

bool sensorsValid = false;
bool controlEnabled = false;

//Functions (Declarations)

bool readTemperature();
void updatePID();
void setPeltier(int pwm, bool heating);
void stopPeltier();
void processSerial();
void printStatus();
void safetyShutdown(const char* reason);

//Pre-Loop Setup

void setup() {

  Serial.begin(9600); //Serial Monitor at 9600

  //Configuration of H-Bridge Driver pins
  pinMode(RPWM_PIN, OUTPUT); 
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);

  stopPeltier(); //Peltier starts off

  Serial.println("PCR Temperature Controller");
  Serial.println("Initializing...");

  // Configure DS18B20 resolution
  ds.setResolution(10);

  // Initialize independent timers
  previousControlTime = millis();
  lastSensorReadTime = millis();

  Serial.println("Initialization complete.");
  Serial.println("Type HELP for commands.");

#if !PELTIER_HARDWARE_VERIFIED //If Peltier isn't Verified Before Setup (Set to 0)

  Serial.println("WARNING: Peltier hardware safety lockout is active.");
  Serial.println("Peltier outputs cannot be enabled.");

#endif

}

//main() loop

void loop() {

  unsigned long currentTime = millis();

  // Process Serial Monitor commands.
  processSerial();

  //sensor timers

  if (currentTime - lastSensorReadTime >= CONTROL_INTERVAL) { //if it is time to "re-sense"

    lastSensorReadTime = currentTime; //update last sensor time with current

    sensorsValid = readTemperature(); //read temps

    if (!sensorsValid) {
      safetyShutdown("Temperature sensor error"); //If sensor is invalid return
      return;
    }

    // Check raw temperature for overheating.
    if (rawTemp >= MAX_SAFE_TEMP) {
      safetyShutdown("Maximum safe temperature exceeded"); //If overheating return and stop
      return;
    }

    printStatus(); //Print status to Serial Monitor
  }

  //PID timer

  if (controlEnabled &&
      sensorsValid &&
      currentTime - previousControlTime >= CONTROL_INTERVAL) { //if temp control is enabled and sensor is valid and time to read temp again, update the PID

    updatePID();

  }

}

//Temp Reading function

bool readTemperature() {

  float reading;

  ds.resetSearch();

  // Find the first DS18B20 sensor otherwise return false
  if (!ds.selectNext()) {

    Serial.println("ERROR: No DS18B20 sensor detected.");

    return false;
  }

  reading = ds.getTempC(); //reading recieves Temperature in Celcius

  // Validate sensor reading
  if (isnan(reading)) {

    Serial.println("ERROR: Invalid temperature reading."); 

    return false;
  }

  // Check DS18B20 operating range
  if (reading < -55.0 || reading > 125.0) {

    Serial.println("ERROR: Temperature outside sensor range.");

    return false;
  }

  // Store raw temperature
  rawTemp = reading;

  
  //EMA Filter


  if (!filterInitialized) { //initialize filter

    filteredTemp = rawTemp;

    filterInitialized = true;

  } else { //calculate with current values

    filteredTemp =
        FILTER_ALPHA * rawTemp +
        (1.0 - FILTER_ALPHA) * filteredTemp;

  }

  return true;

}

//PID Controller (MAIN PID)

void updatePID() {

  unsigned long currentTime = millis();

  // Calculate actual elapsed time.
  float dt = (currentTime - previousControlTime) / 1000.0;

  // Update timestamp AFTER calculating dt.
  previousControlTime = currentTime;

  if (dt <= 0.0) {
    return;
  }

  // Calculate temperature error using filtered reading
  float error = targetTemp - filteredTemp;

  //INTEGRAL

  integral += error * dt;

  // Prevent excessive integral accum
  integral = constrain(integral, -100.0, 100.0);

  // DERIVATIVE

  float derivative = (error - previousError) / dt;

  // Filter derivative
  filteredDerivative =
      DERIVATIVE_ALPHA * derivative +
      (1.0 - DERIVATIVE_ALPHA) * filteredDerivative;

  // PID OUTPUT

  float output =
      Kp * error +
      Ki * integral +
      Kd * filteredDerivative;

  previousError = error;

  // Convert PID output to PWM.
  int pwm = (int)constrain(fabs(output), 0.0, (float)MAX_PWM);

  
  // HEATING / COOLING

  if (output > 0) { //Heat

    setPeltier(pwm, true);

  } else if (output < 0) { //Cool

    setPeltier(pwm, false);

  } else {

    stopPeltier(); //Off

  }

}

// Peltier Controller

void setPeltier(int pwm, bool heating) {

#if PELTIER_HARDWARE_VERIFIED

  digitalWrite(R_EN_PIN, HIGH); 
  digitalWrite(L_EN_PIN, HIGH);

  if (heating) {

    analogWrite(RPWM_PIN, pwm);
    analogWrite(LPWM_PIN, 0);

  } else {

    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, pwm);

  }

#else

  stopPeltier();

#endif

}

// Stop Peltier Function

void stopPeltier() {

  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);

  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);

}

//Serial Monitor Commands

void processSerial() {

  if (!Serial.available()) {
    return;
  }

  String command = Serial.readStringUntil('\n'); //command variable setup

  command.trim();


  // Set Target Temp
  if (command.startsWith("SET ")) {

    float newTarget = command.substring(4).toFloat();

    if (newTarget >= MIN_TARGET_TEMP &&
        newTarget <= MAX_TARGET_TEMP) {

      targetTemp = newTarget;

      integral = 0;
      previousError = 0;
      filteredDerivative = 0;

      Serial.print("Target temperature set to: ");
      Serial.println(targetTemp);

    } else {

      Serial.println("ERROR: Target temperature out of range.");

    }

  }


  //Enable Control
  else if (command == "CONTROL ON") {

#if PELTIER_HARDWARE_VERIFIED

    if (!sensorsValid) {

      Serial.println("ERROR: Sensor is not ready.");

    } else {

      controlEnabled = true;

      previousControlTime = millis();

      integral = 0;
      previousError = 0;
      filteredDerivative = 0;

      Serial.println("Temperature control enabled.");

    }

#else

    Serial.println("ERROR: Hardware safety lockout is active.");
    Serial.println("Do not bypass without verified current limiting.");

#endif

  }

  //Disable Control
  else if (command == "CONTROL OFF") {

    controlEnabled = false;

    stopPeltier();

    integral = 0;
    previousError = 0;
    filteredDerivative = 0;

    Serial.println("Temperature control disabled.");

  }

  //Check Status
  else if (command == "STATUS") {

    printStatus();

  }

  //CMD help/info
  else if (command == "HELP") {

    Serial.println("Available commands:");

    Serial.println("SET 37       - Set target temperature");
    Serial.println("CONTROL ON   - Enable temperature control");
    Serial.println("CONTROL OFF  - Disable temperature control");
    Serial.println("STATUS       - Show current temperature");
    Serial.println("HELP         - Show commands");

  }

  else {

    Serial.println("Unknown command. Type HELP.");

  }

}

//Print Status of Current Goal
void printStatus() {

  Serial.println("-------------------------");

  Serial.print("Raw Temperature: ");
  Serial.println(rawTemp, 2);

  Serial.print("Filtered Temperature: ");
  Serial.println(filteredTemp, 2);

  Serial.print("Target Temperature: ");
  Serial.println(targetTemp, 2);

  Serial.print("Control: ");
  Serial.println(controlEnabled ? "ON" : "OFF");

  Serial.println("-------------------------");

}

//Emergency Shutdown Sequence
void safetyShutdown(const char* reason) {

  controlEnabled = false;

  stopPeltier();

  integral = 0;
  previousError = 0;
  filteredDerivative = 0;

  sensorsValid = false;

  Serial.print("SAFETY SHUTDOWN: ");
  Serial.println(reason);

}
