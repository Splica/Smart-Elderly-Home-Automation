/*
  ET1025 IEP Mini Project

  Project Title:
  Smart Elderly Home Safety and Energy Monitoring System

  Members:
  SUAREZ MARCO NIEL JOHN RAET (2528179), 
  HAKIM RIFAI BIN MOHAMMED REDHA (2619929)

  Target User:
  Elderly people living alone in HDB flats

  ==================================================
  MAIN PROGRAM STRUCTURE
  ==================================================

  1.0 INPUT
      - Read DHT11 temperature and humidity
      - Read LDR light resistance
      - Read knob smoke/gas simulation
      - Read NTC temperature
      - Read K1 and K2 buttons
      - Read IR remote

  2.0 COMPUTATION
      - Process button and IR commands
      - Determine whether the room is dark
      - Determine whether a fan is needed
      - Determine warning conditions
      - Determine alarm conditions
      - Select the display mode

  3.0 OUTPUT
      - Control the LEDs
      - Control the buzzer
      - Update the TM1637 display
      - Print information to Serial Monitor

  ==================================================

  K1 = Emergency help button
  K2 = Change display mode / silence alarm

  Display Modes:
  0 = DHT11 Temperature
  1 = DHT11 Humidity
  2 = LDR light sensor value
  3 = Knob smoke/gas simulation
  4 = NTC temperature
*/

#include <Wire.h>

#include "RichShieldDHT.h"
#include "RichShieldTM1637.h"
#include "RichShieldKEY.h"
#include "RichShieldLED.h"
#include "RichShieldKnob.h"
#include "RichShieldLightSensor.h"
#include "RichShieldNTC.h"
#include "RichShieldPassiveBuzzer.h"
#include "RichShieldIRremote.h"

// ==================================================
// PIN DEFINITIONS
// ==================================================

#define BUZZER_PIN 3

#define LED_YELLOW_PIN 7
#define LED_BLUE_PIN 6
#define LED_GREEN_PIN 5
#define LED_RED_PIN 4

#define KEY1_PIN 8
#define KEY2_PIN 9

#define CLK 10
#define DIO 11

#define DHT_PIN 12
#define DHT_TYPE DHT11

#define KNOB_PIN A0
#define NTC_PIN A1
#define LIGHT_SENSOR_PIN A2

#define IR_PIN 2

// ==================================================
// RICH SHIELD OBJECTS
// ==================================================

TM1637 disp(CLK, DIO);
DHT dht(DHT_PIN, DHT_TYPE);
Key key(KEY1_PIN, KEY2_PIN);

LED led(
  LED_YELLOW_PIN,
  LED_BLUE_PIN,
  LED_GREEN_PIN,
  LED_RED_PIN
);

Knob knob(KNOB_PIN);
LightSensor lightSensor(LIGHT_SENSOR_PIN);
NTC ntc(NTC_PIN);
PassiveBuzzer buzzer(BUZZER_PIN);
IRrecv IR(IR_PIN);

// ==================================================
// LED NUMBERS
// ==================================================

// Based on the order used when creating the LED object.
#define LED_YELLOW 1
#define LED_BLUE 2
#define LED_GREEN 3
#define LED_RED 4

// ==================================================
// SYSTEM THRESHOLDS
// ==================================================

const int FAN_TEMP = 30;
const int DANGER_TEMP = 35;

const int HUMID_WARNING = 70;
const int HUMID_DANGER = 85;

// LightSensor.getRes() returns resistance in kΩ.
// A higher resistance means the environment is darker.
const float DARK_RESISTANCE = 40.0;

// Knob angle range: 0 to 280 degrees.
const int SMOKE_WARNING_ANGLE = 180;
const int SMOKE_DANGER_ANGLE = 220;

// ==================================================
// SENSOR INPUT VARIABLES
// ==================================================

float temperature = 0;
float humidity = 0;
float lightResistance = 0;
float ntcTemperature = 0;

int smokeAngle = 0;

bool dhtOK = false;

// ==================================================
// USER INPUT VARIABLES
// ==================================================

// These temporary variables record inputs for one loop cycle.
bool k1PressedEvent = false;
bool k2PressedEvent = false;

bool irSilenceEvent = false;
bool irKeyReceived = false;

int irRequestedDisplayMode = -1;
unsigned char receivedIRKeyCode = 0;

int lastKeyNumber = 0;

// ==================================================
// COMPUTED SYSTEM VARIABLES
// ==================================================

bool emergencyActive = false;
bool buzzerMuted = false;

bool roomDark = false;
bool fanNeeded = false;
bool warningActive = false;
bool alarmActive = false;

int displayMode = 0;
const int DISPLAY_MODE_COUNT = 5;

// ==================================================
// OUTPUT EVENT VARIABLES
// ==================================================

bool newSensorReadings = false;

bool emergencyActivatedMessage = false;
bool alarmSilencedMessage = false;
bool displayChangedMessage = false;
bool displayChangedByIR = false;

bool feedbackTonePending = false;
int feedbackToneFrequency = 0;
int feedbackToneDuration = 0;

// ==================================================
// TIMING VARIABLES
// ==================================================

unsigned long lastSensorRead = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastRedBlink = 0;
unsigned long lastBuzzerBeep = 0;

bool redBlinkState = false;

// ==================================================
// ARRAYS
// ==================================================

const int LED_COUNT = 4;

const int ledList[LED_COUNT] = {
  LED_YELLOW,
  LED_BLUE,
  LED_GREEN,
  LED_RED
};

const int IR_MODE_COUNT = 5;

const unsigned char irModeKeys[IR_MODE_COUNT] = {
  KEY_ONE,
  KEY_TWO,
  KEY_THREE,
  KEY_FOUR,
  KEY_FIVE
};

// ==================================================
// STARTUP MUSIC
// ==================================================

#define REST 0

int startupMelody[] = {
  262, 294, 330, 392, 330, 392, 440, 523,
  REST,
  523, 440, 392, 330, 294, 330, 392, 523,
  REST,
  392, 440, 523, 659, 523, 440, 392, 330,
  REST,
  330, 392, 440, 523, 440, 392, 330, 262
};

int startupDurations[] = {
  200, 200, 200, 300, 200, 200, 300, 500,
  200,
  200, 200, 200, 300, 200, 200, 300, 500,
  200,
  200, 200, 200, 300, 200, 200, 300, 500,
  200,
  200, 200, 200, 300, 200, 200, 300, 600
};

const int STARTUP_NOTE_COUNT =
  sizeof(startupMelody) / sizeof(startupMelody[0]);

// ==================================================
// SETUP
// ==================================================

void setup() {
  // Initialise Serial Monitor.
  Serial.begin(9600);

  // Initialise the TM1637 display.
  disp.init();
  disp.set(BRIGHT_TYPICAL);

  // Initialise the DHT11 sensor.
  dht.begin();

  // Initialise the IR receiver.
  IR.enableIRIn();

  // Test the output devices.
  turnOffAllLEDs();
  startupLEDTest();
  playStartupMusic();

  // Show zero after startup.
  disp.display(0);

  Serial.println("=====================================");
  Serial.println("Smart Elderly Home Safety System");
  Serial.println("K1 = Emergency Help Button");
  Serial.println("K2 = Change Display Mode / Silence Alarm");
  Serial.println("IR 1-5 = Select Display Mode");
  Serial.println("IR Power or C = Silence Alarm");
  Serial.println("=====================================");
}

// ==================================================
// MAIN LOOP
// ==================================================

void loop() {
  /*
    Reset temporary event variables before starting
    another Input → Computation → Output cycle.
  */
  resetCycleEvents();

  // ==================================================
  // 1.0 INPUT
  // ==================================================

  readSensorInputs();
  readButtonInputs();
  readIRInputs();

  // ==================================================
  // 2.0 COMPUTATION
  // ==================================================

  processUserInputs();
  computeSystemConditions();

  // ==================================================
  // 3.0 OUTPUT
  // ==================================================

  updateLEDOutputs();
  updateBuzzerOutput();
  updateDisplayOutput();
  updateSerialOutput();
}

// ==================================================
// RESET TEMPORARY LOOP EVENTS
// ==================================================

void resetCycleEvents() {
  // Reset button input events.
  k1PressedEvent = false;
  k2PressedEvent = false;

  // Reset IR input events.
  irSilenceEvent = false;
  irKeyReceived = false;
  irRequestedDisplayMode = -1;

  // Reset output message events.
  newSensorReadings = false;
  emergencyActivatedMessage = false;
  alarmSilencedMessage = false;
  displayChangedMessage = false;
  displayChangedByIR = false;

  // Reset feedback tone request.
  feedbackTonePending = false;
}

// ==================================================
// 1.0 INPUT
// ==================================================

// --------------------------------------------------
// 1.1 Read sensor inputs
// --------------------------------------------------

void readSensorInputs() {
  // Read all sensors once every 10 seconds.
  if (millis() - lastSensorRead >= 10000 ||
      lastSensorRead == 0) {

    lastSensorRead = millis();

    // Read DHT11 temperature and humidity.
    float newTemperature = dht.readTemperature();
    float newHumidity = dht.readHumidity();

    // Check whether the DHT11 reading is valid.
    if (isnan(newTemperature) || isnan(newHumidity)) {
      dhtOK = false;
    } else {
      dhtOK = true;

      temperature = newTemperature;
      humidity = newHumidity;
    }

    // Read the light sensor.
    lightResistance = lightSensor.getRes();

    // Read the knob for smoke/gas simulation.
    smokeAngle = knob.getAngle();

    // Read the NTC temperature sensor.
    ntcTemperature = ntc.getTemperature();

    // Tell the Output section that new readings are available.
    newSensorReadings = true;
  }
}

// --------------------------------------------------
// 1.2 Read button inputs
// --------------------------------------------------

void readButtonInputs() {
  int keyNumber = key.get();

  /*
    Trigger only once when a button is newly pressed.

    keyNumber = 1 means K1
    keyNumber = 2 means K2
    keyNumber = 0 means no button is pressed
  */
  if (keyNumber != 0 && lastKeyNumber == 0) {
    if (keyNumber == 1) {
      k1PressedEvent = true;
    } else if (keyNumber == 2) {
      k2PressedEvent = true;
    }
  }

  lastKeyNumber = keyNumber;
}

// --------------------------------------------------
// 1.3 Read IR remote input
// --------------------------------------------------

void readIRInputs() {
  if (IR.decode()) {
    if (IR.isReleased()) {
      receivedIRKeyCode = IR.keycode;
      irKeyReceived = true;

      // Check whether IR buttons 1 to 5 were pressed.
      for (int i = 0; i < IR_MODE_COUNT; i++) {
        if (receivedIRKeyCode == irModeKeys[i]) {
          irRequestedDisplayMode = i;
        }
      }

      // Check whether Power or C was pressed.
      if (receivedIRKeyCode == KEY_POWER ||
          receivedIRKeyCode == KEY_C) {

        irSilenceEvent = true;
      }
    }

    // Prepare the IR receiver for another command.
    IR.resume();
  }
}

// ==================================================
// 2.0 COMPUTATION
// ==================================================

// --------------------------------------------------
// 2.1 Process button and IR commands
// --------------------------------------------------

void processUserInputs() {
  // -----------------------------------------------
  // Process K1 emergency button
  // -----------------------------------------------

  if (k1PressedEvent) {
    emergencyActive = true;
    buzzerMuted = false;

    emergencyActivatedMessage = true;

    requestFeedbackTone(1500, 100);
  }

  // -----------------------------------------------
  // Process K2 button
  // -----------------------------------------------

  if (k2PressedEvent) {
    /*
      If an alarm is active:
      K2 silences the alarm.

      If no alarm is active:
      K2 changes the display mode.
    */

    if (calculateAlarmActive()) {
      silenceAlarmComputation();
      alarmSilencedMessage = true;
    } else {
      displayMode++;

      // Return to mode 0 after mode 4.
      if (displayMode >= DISPLAY_MODE_COUNT) {
        displayMode = 0;
      }

      displayChangedMessage = true;

      requestFeedbackTone(1000, 80);
    }
  }

  // -----------------------------------------------
  // Process IR number buttons
  // -----------------------------------------------

  if (irRequestedDisplayMode >= 0) {
    displayMode = irRequestedDisplayMode;

    displayChangedMessage = true;
    displayChangedByIR = true;

    requestFeedbackTone(1200, 80);
  }

  // -----------------------------------------------
  // Process IR alarm silence command
  // -----------------------------------------------

  if (irSilenceEvent) {
    silenceAlarmComputation();
    alarmSilencedMessage = true;
  }
}

// --------------------------------------------------
// 2.2 Calculate all system conditions
// --------------------------------------------------

void computeSystemConditions() {
  /*
    Computation:
    roomDark = lightResistance compared with threshold
  */
  roomDark = lightResistance >= DARK_RESISTANCE;

  /*
    Computation:
    fanNeeded = valid DHT reading AND temperature
                greater than or equal to FAN_TEMP
  */
  fanNeeded =
    dhtOK &&
    temperature >= FAN_TEMP;

  /*
    Computation:
    warningActive = humidity warning OR smoke warning
  */
  warningActive = calculateWarningActive();

  /*
    Computation:
    alarmActive = emergency OR temperature danger
                  OR humidity danger OR smoke danger
  */
  alarmActive = calculateAlarmActive();
}

// --------------------------------------------------
// 2.3 Calculate warning condition
// --------------------------------------------------

bool calculateWarningActive() {
  bool humidityWarning =
    dhtOK &&
    humidity >= HUMID_WARNING;

  bool smokeWarning =
    smokeAngle >= SMOKE_WARNING_ANGLE;

  return humidityWarning || smokeWarning;
}

// --------------------------------------------------
// 2.4 Calculate alarm condition
// --------------------------------------------------

bool calculateAlarmActive() {
  bool temperatureAlarm =
    dhtOK &&
    temperature >= DANGER_TEMP;

  bool humidityAlarm =
    dhtOK &&
    humidity >= HUMID_DANGER;

  bool smokeAlarm =
    smokeAngle >= SMOKE_DANGER_ANGLE;

  return emergencyActive ||
         temperatureAlarm ||
         humidityAlarm ||
         smokeAlarm;
}

// --------------------------------------------------
// 2.5 Silence alarm computation
// --------------------------------------------------

void silenceAlarmComputation() {
  /*
    Stop the manually activated emergency state.

    Environmental danger conditions may still remain
    active, but buzzerMuted prevents the buzzer from
    sounding until the danger condition clears.
  */
  emergencyActive = false;
  buzzerMuted = true;
}

// --------------------------------------------------
// 2.6 Request a feedback tone
// --------------------------------------------------

void requestFeedbackTone(int frequency, int duration) {
  feedbackTonePending = true;
  feedbackToneFrequency = frequency;
  feedbackToneDuration = duration;
}

// ==================================================
// 3.0 OUTPUT
// ==================================================

// --------------------------------------------------
// 3.1 Update LED outputs
// --------------------------------------------------

void updateLEDOutputs() {
  // Yellow LED = simulated room light.
  if (roomDark) {
    led.on(LED_YELLOW);
  } else {
    led.off(LED_YELLOW);
  }

  // Blue LED = simulated fan or air-conditioner.
  if (fanNeeded) {
    led.on(LED_BLUE);
  } else {
    led.off(LED_BLUE);
  }

  // Green LED = system is safe.
  if (!warningActive && !alarmActive) {
    led.on(LED_GREEN);
  } else {
    led.off(LED_GREEN);
  }

  // Red LED = warning or danger.
  if (warningActive || alarmActive) {
    blinkRedLED();
  } else {
    led.off(LED_RED);
    redBlinkState = false;
  }
}

// --------------------------------------------------
// 3.2 Blink red LED
// --------------------------------------------------

void blinkRedLED() {
  if (millis() - lastRedBlink >= 300) {
    lastRedBlink = millis();

    redBlinkState = !redBlinkState;

    if (redBlinkState) {
      led.on(LED_RED);
    } else {
      led.off(LED_RED);
    }
  }
}

// --------------------------------------------------
// 3.3 Update buzzer output
// --------------------------------------------------

void updateBuzzerOutput() {
  // Play a button or IR feedback tone first.
  if (feedbackTonePending) {
    buzzer.playTone(
      feedbackToneFrequency,
      feedbackToneDuration
    );

    return;
  }

  // Turn off the buzzer when no alarm is active.
  if (!alarmActive) {
    buzzerMuted = false;
    buzzer.off();

    return;
  }

  // Keep the buzzer off when the alarm is muted.
  if (buzzerMuted) {
    buzzer.off();

    return;
  }

  // Beep repeatedly while an alarm is active.
  if (millis() - lastBuzzerBeep >= 600) {
    lastBuzzerBeep = millis();

    buzzer.playTone(1200, 120);
  }
}

// --------------------------------------------------
// 3.4 Update display output
// --------------------------------------------------

void updateDisplayOutput() {
  // Update the display every 500 milliseconds.
  if (millis() - lastDisplayUpdate < 500) {
    return;
  }

  lastDisplayUpdate = millis();

  // Emergency message has the highest priority.
  if (emergencyActive) {
    showHELP();
    return;
  }

  // Show ERR if the selected DHT value is unavailable.
  if (!dhtOK &&
      (displayMode == 0 || displayMode == 1)) {

    showERR();
    return;
  }

  // Display mode 0: DHT11 temperature.
  if (displayMode == 0) {
    displayTemperature((int)temperature);
  }

  // Display mode 1: DHT11 humidity.
  else if (displayMode == 1) {
    displayHumidity((int)humidity);
  }

  // Display mode 2: LDR resistance.
  else if (displayMode == 2) {
    disp.display((int)lightResistance);
  }

  // Display mode 3: smoke simulation angle.
  else if (displayMode == 3) {
    disp.display(smokeAngle);
  }

  // Display mode 4: NTC temperature.
  else if (displayMode == 4) {
    displayTemperature((int)ntcTemperature);
  }
}

// --------------------------------------------------
// 3.5 Update Serial Monitor output
// --------------------------------------------------

void updateSerialOutput() {
  // Print sensor readings when they are newly read.
  if (newSensorReadings) {
    printSensorReadings();
  }

  // Print received IR key.
  if (irKeyReceived) {
    Serial.print("IR key code: ");
    Serial.println(receivedIRKeyCode, HEX);
  }

  // Print emergency event.
  if (emergencyActivatedMessage) {
    Serial.println("K1 pressed: Emergency activated");
  }

  // Print display change event.
  if (displayChangedMessage) {
    if (displayChangedByIR) {
      Serial.print("IR selected display mode ");
    } else {
      Serial.print("K2 pressed: Display mode changed to ");
    }

    Serial.println(displayMode);
  }

  // Print alarm silence event.
  if (alarmSilencedMessage) {
    if (irSilenceEvent) {
      Serial.println("IR pressed: Alarm silenced");
    } else {
      Serial.println("K2 pressed: Alarm silenced");
    }
  }
}

// ==================================================
// DISPLAY FORMATTING FUNCTIONS
// ==================================================

void displayTemperature(int temperatureValue) {
  int8_t data[4];

  if (temperatureValue < 0) {
    data[0] = INDEX_NEGATIVE_SIGN;
    temperatureValue = abs(temperatureValue);
  } else if (temperatureValue < 100) {
    data[0] = INDEX_BLANK;
  } else {
    data[0] = temperatureValue / 100;
  }

  temperatureValue = temperatureValue % 100;

  data[1] = temperatureValue / 10;
  data[2] = temperatureValue % 10;
  data[3] = 12;  // C

  disp.display(data);
}

void displayHumidity(int humidityValue) {
  int8_t data[4];

  if (humidityValue < 100) {
    data[0] = INDEX_BLANK;
  } else {
    data[0] = humidityValue / 100;
  }

  humidityValue = humidityValue % 100;

  data[1] = humidityValue / 10;
  data[2] = humidityValue % 10;
  data[3] = 18;  // H

  disp.display(data);
}

// ==================================================
// SPECIAL DISPLAY MESSAGES
// ==================================================

void showHELP() {
  const uint8_t rawData[4] = {
    0x76,  // H
    0x79,  // E
    0x38,  // L
    0x73   // P
  };

  displayRawSegments(rawData);
}

void showERR() {
  const uint8_t rawData[4] = {
    0x79,  // E
    0x50,  // r
    0x50,  // r
    0x00   // Blank
  };

  displayRawSegments(rawData);
}

void displayRawSegments(const uint8_t rawData[]) {
  disp.start();
  disp.writeByte(ADDR_AUTO);
  disp.stop();

  disp.start();
  disp.writeByte(STARTADDR);

  for (int i = 0; i < 4; i++) {
    disp.writeByte(rawData[i]);
  }

  disp.stop();

  disp.start();
  disp.writeByte(disp.Cmd_DispCtrl);
  disp.stop();
}

// ==================================================
// STARTUP OUTPUT FUNCTIONS
// ==================================================

void turnOffAllLEDs() {
  for (int i = 0; i < LED_COUNT; i++) {
    led.off(ledList[i]);
  }
}

void startupLEDTest() {
  for (int i = 0; i < LED_COUNT; i++) {
    led.on(ledList[i]);
    delay(150);

    led.off(ledList[i]);
  }
}

void playStartupMusic() {
  for (int i = 0; i < STARTUP_NOTE_COUNT; i++) {
    if (startupMelody[i] == REST) {
      buzzer.off();
      delay(startupDurations[i]);
    } else {
      buzzer.playTone(
        startupMelody[i],
        startupDurations[i]
      );

      delay(startupDurations[i] + 50);
    }
  }

  buzzer.off();
}

// ==================================================
// SERIAL MONITOR SENSOR OUTPUT
// ==================================================

void printSensorReadings() {
  Serial.println("========== SENSOR READINGS ==========");

  if (dhtOK) {
    Serial.print("DHT Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");

    Serial.print("DHT Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  } else {
    Serial.println("DHT11 Error");
  }

  Serial.print("Light resistance: ");
  Serial.print(lightResistance);
  Serial.println(" kOhm");

  Serial.print("Smoke simulation knob angle: ");
  Serial.print(smokeAngle);
  Serial.println(" degrees");

  Serial.print("NTC Temperature: ");
  Serial.print(ntcTemperature);
  Serial.println(" C");

  Serial.println("=====================================");
}