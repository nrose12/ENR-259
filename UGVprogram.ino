
#include <QTRSensors.h>
#include <PWMServo.h>
#include <TB67H420FTG.h>

/* Edit these values as needed */
#define RED_THRESHOLD 8             // Lower this value to increase red detection sensitivity (default 8)
#define RED_COUNT_TRIGGER 20        // The number of red counts to see before releasing the UGV
#define MS_DELAY_BETWEEN_SENSOR_READINGS 200  // The ms delay time between color sensor readings
#define SERVO_ENGAGE_POSITION 180   // This is the position when mounted to the UAV
#define SERVO_DISENGAGE_POSITION 110  // This is the position to allow the UGV to dismount from the UAV

/*
 * DESCRIPTION:
 * This code is designed for the TYESA 2020 UAV Competition. This code is capable of controlling a ground vehicle that
 * detaches from a UAV and follows a black line on a white background.
 *
 * The settings below can be altered to change the configuration of the code. The RED_THRESHOLD determines how
 * sensitive the color sensor will be when detecting red, the lower the value, the more sensitive the sensor will be.
 *
 * The RED_COUNT_TRIGGER determines how many red detection readings need to be seen in a row before dropping the UGV
 * from the UAV. If you have trouble detecting 20 readings in a row before the count resets, try lowering the
 * RED_THRESHOLD value so red is detected more easily.
 *
 * The MS_DELAY_BETWEEN_SENSOR_READINGS determines how long to pause between color sensor readings. Effectively, the
 * time from the first red detection to the release of the robot is going to be
 * RED_COUNT_TRIGGER * MS_DELAY_BETWEEN_SENSOR_READINGS. As an example, the default values are 20 and 200 so it would
 * be 20 * 200 = 4000ms or four seconds. Adjust either value as necessary to increase or decrease this duration.
 *
 * Regarding Menu Options, Calibration will always commence once upon startup, however, afterward the menu options
 * control what the code will do.
 * 0. Runs the line sensor calibration (though this doesn't clear any existing calibration)
 * 1. Engages the servo, this is intended to be used to attach the UGV to the UAV
 * 2. Disengages the servo, this is intended to be used to remove the UGV from the UAV
 * 3. Starts the Detect sequence to look for red and once the variables are met, detach the robot and line follow
 * 4. Displays QTR sensor readings using calibrated values (good for testing your hookup and calibration)  
 * 5. Stops the current action, intended for stopping the Detect sequence which also resets the red count
 * (Inputs that are not recognized will also call option 4, the stop command)
 */

/*
 * SETUP:
 *
 * If using a Teensy, the Teensyduino program will have to be installed on your computer to allow the Teensy to work
 * with the Arduino IDE. Download from the included link https://www.pjrc.com/teensy/td_download.html
 *
 * The included libraries will need to be installed through the library manager, a guide is provided here
 * https://www.digikey.com/en/maker/blogs/2018/how-to-install-arduino-libraries.
 * The libraries can be found here:
 * QTR Sensor Array - https://github.com/gberl001/qtr-sensors-arduino.git
 * Motor Driver     - https://github.com/mcc-robotics/Dynamic_Motor_Driver
 */

/* ADDITIONAL NOTES:
 * // It is helpful to view the instructional videos at:
 * http://robotresearchlab.com/2019/03/13/build-a-custom-pid-line-following-robot-from-scratch/#Programming_the_motors
 * Credit to Professor Berl for creating these videos, they are extremely helpful and detailed about how to build and
 * program a line follow robot.
 *
 * Note: Students are encouraged to build off this code, or change it if they have any ideas to make it better.
 *       This code is also specifically written to work with the Teensy and other UGV equipment outlined in the rules.
 *       Other microcontrollers and equipment can be used, but the code may need to be modified to work with your
 *       specific equipment.
 *
 *       A laptop is also highly recommended for the competition to attach the UGV to the UAV before flight.
 */

#define RED 0
#define GREEN 1
#define BLUE 2
#define STATE_CALIBRATION '0'
#define STATE_ENGAGE_SERVO '1'
#define STATE_DISENGAGE_SERVO '2'
#define STATE_DETECT_RED '3'
#define STATE_QTR_DISPLAY '4'
#define STATE_PAUSE '5'
#define STATE_LINE_FOLLOW '6'

void calibrateLineSensor();
void detectRed();
void lineFollow();
void checkForSerialInputs();
void displayMenu();
void displayQTRReadings();

//             MOTOR DRIVER INITIALIZATION
TB67H420FTG driver(12, 11, 10, 7, 8, 9); // Set the INA1, INA2, PWMA, INB1, INB2, PWMB pins (in that order)

//                PID PROPERTIES
// These variables are used to control the course correction of the UGV when following the line
// To understand how to change these variables to tune your robot please watch the following video:
const double KP = 0.023;
const double KD = 0.0;
double lastError = 0;
const int LINE_POSITION_GOAL = 3500;
const unsigned char SET_SPEED = 40; // The goal speed of the motors
const unsigned char MAX_SPEED = 40; // Set max speed of the motors.

//            SERVO INITIALIZATION
PWMServo mountingServo;
const char SERVO_PIN = 23;

//         QTR IR SENSOR INITIALIZATION
#define NUM_SENSORS             8  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define EMITTER_PIN             QTR_NO_EMITTER_PIN  // Emitters are always on

QTRSensorsAnalog qtra((unsigned char[]) { 14, 15, 16, 17, 18, 19, 20, 21 }, NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];


//         LIGHT SENSOR INITIALIZATION
// These variables are used to assign names to the Arduino IO pins connected to the TCS230, these are not the same as the analog pins assigned earlier
const char COLOR_S0 = 0;
const char COLOR_S1 = 1;
const char COLOR_S2 = 2;
const char COLOR_S3 = 3;
const char COLOR_OUT = 22;
// This array will hold the Red, Green, and Blue components of the sensor
int rgbValues[3] = {};

//      INDICATOR FLAG INITIALIZATION
int redCount = 0;
char currentState = STATE_CALIBRATION;

//                     MAIN SETUP
// The microcontroller will run this section one time when plugged in to "SETUP" everything for the MAIN loop
void setup() {
  Serial.begin(115200);  // set up Serial library at 9600 bps
  pinMode(LED_BUILTIN, OUTPUT);
  delay(5000);  // five second delay to allow for serial monitor setup

  //                  MOTOR SETUP
  driver.init();
  driver.setMotorAPower(40);
  driver.setMotorBPower(40);
  //                  SERVO SETUP
  mountingServo.attach(SERVO_PIN);  // attaches the servo on pin to the servo object

  //               COLOR SENSOR SETUP
  // Set IO pin types for TCS230 color sensor, COLOR_S0 - COLOR_S3 and COLOR_OUT were assigned above
  pinMode(COLOR_S0, OUTPUT);
  pinMode(COLOR_S1, OUTPUT);
  pinMode(COLOR_S2, OUTPUT);
  pinMode(COLOR_S3, OUTPUT);
  pinMode(COLOR_OUT, INPUT);
  // Set TCS230 Sensor pins COLOR_S0 and COLOR_S1 to HIGH, activating the sensor
  digitalWrite(COLOR_S0, HIGH);
  digitalWrite(COLOR_S1, HIGH);

}


void loop() {

  checkForSerialInputs();

  // Handle current state
  switch (currentState) {
    case STATE_CALIBRATION:
      calibrateLineSensor();
      displayMenu();
      currentState = STATE_PAUSE;
      break;
    case STATE_ENGAGE_SERVO:
      // Engage the servo and update the state
      mountingServo.write(SERVO_ENGAGE_POSITION);
      displayMenu();
      currentState = STATE_PAUSE;
      break;
    case STATE_DISENGAGE_SERVO:
      // Disengage the servo and update the state
      mountingServo.write(SERVO_DISENGAGE_POSITION);
      displayMenu();
      currentState = STATE_PAUSE;
      break;
    case STATE_DETECT_RED:
      // Call detectRed which will handle state changes
      detectRed();
      break;
    case STATE_PAUSE:
      // Reset red count
      redCount = 0;
      driver.coastAll();
      // Do nothing else if paused
      break;
    case STATE_LINE_FOLLOW:
      lineFollow();
      break;
    case STATE_QTR_DISPLAY:
      displayQTRReadings();
      break;
        
    default:
      // Intentionally blank
      break;
  }

}


// *******************************************************
// **************  CUSTOM FUNCTIONS **********************
// *******************************************************

void checkForSerialInputs() {
  if (Serial.available()) {
    char userInput = Serial.read();

    if (userInput == STATE_CALIBRATION ||
        userInput == STATE_ENGAGE_SERVO ||
        userInput == STATE_DISENGAGE_SERVO ||
        userInput == STATE_DETECT_RED ||
        userInput == STATE_QTR_DISPLAY ||
        userInput == STATE_PAUSE ||
        userInput == STATE_LINE_FOLLOW) {
      Serial.print("User selected option ");
      Serial.println(userInput);
      currentState = userInput;
    } else {
      // Ignore newline characters but display error for other unknown inputs
      if (userInput != '\n') {
        Serial.println("Unknown user input");
        currentState = STATE_PAUSE;
      }
    }
    if (userInput == STATE_PAUSE) {
      // If the user manually hit pause, re-display the menu
      displayMenu();
    }
  }
}

void displayMenu() {
  Serial.println("\n------------------- Menu -----------------------");
  Serial.println("Choose an option:");
  Serial.println("   0. Calibrate Line Sensors");
  Serial.println("   1. Engage the Servo");
  Serial.println("   2. Disengage Servo");
  Serial.println("   3. Activate Red Detection and drop algorithm");
  Serial.println("   4. Display QTR Sensor Readings");
  Serial.println("   5. Stop current action");
  Serial.println("   6. State Line Follow");
  Serial.println("------------------------------------------------");
}

void displayQTRReadings() {
  unsigned int position = qtra.readLine(sensorValues);

  for (unsigned char sensorValue : sensorValues) {
    Serial.print(sensorValue);
    Serial.print('\t');
  }
  Serial.println(position); // comment this line out if you are using raw values
}

void calibrateLineSensor() {
  Serial.println("IR Sensor Calibration starting...");
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);    // turn on on board LED to indicate we are in calibration mode
  for (int i = 0; i < 3000; i++) { // make the calibration take about 10 seconds
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }

  digitalWrite(LED_BUILTIN, LOW);     // turn off Arduino's LED to indicate we are through with calibration
  Serial.println("IR Sensor Calibration complete.");
}

/*
 * Takes a reading from the color sensor
 * It reads each RGB component of light and stores in the global rgbValues array
 */
void readColorSensor() {

  // Set filter to RED and read frequency
  digitalWrite(COLOR_S2, LOW);
  digitalWrite(COLOR_S3, LOW);
  rgbValues[RED] = pulseIn(COLOR_OUT, HIGH);

  // Set filter to BLUE and read frequency
  digitalWrite(COLOR_S3, HIGH);
  rgbValues[BLUE] = pulseIn(COLOR_OUT, digitalRead(COLOR_OUT) == HIGH ? LOW : HIGH);

  // Set filter to GREEN and read frequency
  digitalWrite(COLOR_S2, HIGH);
  rgbValues[GREEN] = pulseIn(COLOR_OUT, digitalRead(COLOR_OUT) == HIGH ? LOW : HIGH);
}

void detectRed() {
  // TCS230 Loop Control
  readColorSensor(); // retrieves data from OUT on TCS230 sensor for red, green, and blue.
  // Print sensor values to serial monitor
  Serial.print("R | G | B Intensity: ");
  Serial.print(rgbValues[RED], DEC);
  Serial.print(" | ");
  Serial.print(rgbValues[GREEN], DEC);
  Serial.print(" | ");
  Serial.print(rgbValues[BLUE], DEC);


  // if the color is predominantly red with an intensity less than RED_THRESHOLD, keep a count
  if (rgbValues[RED] < rgbValues[BLUE] && rgbValues[RED] < rgbValues[GREEN] && rgbValues[RED] < RED_THRESHOLD) {
    Serial.println(" - (Red)");
    redCount++;                        //count up 1 every time a red color is detected
    Serial.print("Red Count: ");
    Serial.println(redCount);

    // If the color red is detected 20 times in a row since it will release the robot and move onto line follow
    if (redCount == RED_COUNT_TRIGGER) {
      mountingServo.write(SERVO_DISENGAGE_POSITION);
      currentState = STATE_LINE_FOLLOW;
    }
  } else if (rgbValues[BLUE] < rgbValues[RED] && rgbValues[BLUE] < rgbValues[GREEN]) {
    Serial.println(" - (Blue)");
    // Reset the red count
    redCount = 0;
  } else if (rgbValues[GREEN] < rgbValues[RED] && rgbValues[GREEN] < rgbValues[BLUE]) {
    Serial.println(" - (Green)");
    // Reset the red count
    redCount = 0;
  } else {
    // This will handle when red is detected but is greater than the threshold
    Serial.println(" - (Red) ");
    // Reset the red count
    redCount = 0;
  }

  // Pause for a short time
  delay(MS_DELAY_BETWEEN_SENSOR_READINGS);
}

void lineFollow() {
  // Get line position
  unsigned int position = qtra.readLine(sensorValues);

  // Compute error from line
  int error = LINE_POSITION_GOAL - position;

  // Compute motor adjustment
  int adjustment = KP * error + KD * (error - lastError);

  // Store error for next increment
  lastError = error;

  // Adjust motors
  driver.setMotorAPower(constrain(SET_SPEED - adjustment, 0, MAX_SPEED));
  driver.setMotorBPower(constrain(SET_SPEED + adjustment, 0, MAX_SPEED));
}
