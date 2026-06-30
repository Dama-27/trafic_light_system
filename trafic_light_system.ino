// ======================================================
// ESP32 Demand Actuated Traffic Signal System
// 8 Ultrasonic Sensors + 4 Traffic Light Modules
// Corrected version:
// Crossing sensor updates count only.
// Signal does not suddenly change because of crossing sensor.
// ======================================================

// ---------------- Ultrasonic Sensor Configuration ----------------

#define TRIG_PIN 5
#define SENSOR_COUNT 8

#define DETECT_DISTANCE 8.0
#define RELEASE_DISTANCE 10.0

#define DETECT_STABLE_READS 2
#define RELEASE_STABLE_READS 3

#define EVENT_COOLDOWN 500UL

// Sensor order:
// 0 = North Entry
// 1 = North Crossing
// 2 = South Entry
// 3 = South Crossing
// 4 = East Entry
// 5 = East Crossing
// 6 = West Entry
// 7 = West Crossing

int echoPins[SENSOR_COUNT] = {
  34, 35,   // North Entry, North Crossing
  36, 39,   // South Entry, South Crossing
  18, 19,   // East Entry, East Crossing
  21, 22    // West Entry, West Crossing
};

// Change to false only if that sensor is not connected
bool sensorEnabled[SENSOR_COUNT] = {
  true, true,    // North Entry, North Crossing
  true, true,    // South Entry, South Crossing
  true, true,    // East Entry, East Crossing
  true, true     // West Entry, West Crossing
};

// Vehicle count for each road
int northCount = 0;
int southCount = 0;
int eastCount  = 0;
int westCount  = 0;

// Interrupt variables
volatile unsigned long riseTime[SENSOR_COUNT];
volatile unsigned long pulseWidth[SENSOR_COUNT];
volatile bool echoDone[SENSOR_COUNT];

float distanceCM[SENSOR_COUNT];

// Stable detection variables
bool detectedState[SENSOR_COUNT] = {
  false, false, false, false,
  false, false, false, false
};

bool previousDetectedState[SENSOR_COUNT] = {
  false, false, false, false,
  false, false, false, false
};

int detectStableCount[SENSOR_COUNT] = {
  0, 0, 0, 0,
  0, 0, 0, 0
};

int releaseStableCount[SENSOR_COUNT] = {
  0, 0, 0, 0,
  0, 0, 0, 0
};

unsigned long lastEventTime[SENSOR_COUNT] = {
  0, 0, 0, 0,
  0, 0, 0, 0
};

// ---------------- Pedestrian and Display Pin Configuration ----------------

#include <TM1637Display.h>

const int NS_CLK = 27;
const int NS_DIO = 26;
const int EW_CLK = 25;
const int EW_DIO = 4;
const int PEDESTRIAN_BUTTON = 16;

// Shared pedestrian phase request flag
volatile bool pedestrianRequest = false;

// Pedestrian phase duration (10 seconds)
#define PEDESTRIAN_TIME 10000UL

// Display objects (displays are connected in parallel for NS and EW)
TM1637Display nsDisplay(NS_CLK, NS_DIO);
TM1637Display ewDisplay(EW_CLK, EW_DIO);

// ---------------- Traffic Light Pin Configuration ----------------

// Most traffic light modules with GND pin are active HIGH.
// If your module works opposite, change this to false.
#define LED_ACTIVE_HIGH true

// North and South modules share the same control pins
#define NS_RED     13
#define NS_YELLOW  12
#define NS_GREEN   14

// East and West modules share the same control pins
#define EW_RED     33
#define EW_YELLOW  32
#define EW_GREEN   23

// ---------------- Traffic Timing Configuration ----------------

#define MIN_GREEN_TIME       5000UL
#define EXTRA_TIME_PER_CAR   2000UL
#define MAX_DEMAND_USED      5
#define MAX_GREEN_TIME       15000UL

// Hard maximum prevents one stuck crossing sensor from keeping green forever
#define HARD_MAX_GREEN_TIME  20000UL

#define YELLOW_TIME          2000UL
#define ALL_RED_TIME         1000UL

#define SENSOR_UPDATE_TIME   120UL
#define SERIAL_PRINT_TIME    1000UL

// ---------------- Traffic State Variables ----------------

enum Phase {
  PHASE_NONE = -1,
  PHASE_NS = 0,
  PHASE_EW = 1
};

enum SignalState {
  STATE_ALL_RED_IDLE,
  STATE_GREEN,
  STATE_YELLOW,
  STATE_ALL_RED_CLEAR,
  STATE_PEDESTRIAN_PHASE
};

volatile SignalState currentState = STATE_ALL_RED_IDLE;
Phase activePhase = PHASE_NONE;
Phase lastServedPhase = PHASE_NONE;

unsigned long stateStartTime = 0;
unsigned long greenDuration = 0;

unsigned long lastSensorUpdate = 0;
unsigned long lastSerialPrint = 0;

// ======================================================
// Interrupt Functions
// ======================================================

void IRAM_ATTR handleEcho(int index) {
  if (!sensorEnabled[index]) return;

  if (digitalRead(echoPins[index]) == HIGH) {
    riseTime[index] = micros();
  } else {
    pulseWidth[index] = micros() - riseTime[index];
    echoDone[index] = true;
  }
}

void IRAM_ATTR echoISR0() { handleEcho(0); }
void IRAM_ATTR echoISR1() { handleEcho(1); }
void IRAM_ATTR echoISR2() { handleEcho(2); }
void IRAM_ATTR echoISR3() { handleEcho(3); }
void IRAM_ATTR echoISR4() { handleEcho(4); }
void IRAM_ATTR echoISR5() { handleEcho(5); }
void IRAM_ATTR echoISR6() { handleEcho(6); }
void IRAM_ATTR echoISR7() { handleEcho(7); }

void IRAM_ATTR pedestrianButtonISR() {
  if (currentState != STATE_PEDESTRIAN_PHASE) {
    pedestrianRequest = true;
  }
}

// ======================================================
// Setup
// ======================================================

void setup() {
  Serial.begin(115200);

  // Initialize TM1637 displays
  nsDisplay.setBrightness(0x0f);
  ewDisplay.setBrightness(0x0f);
  nsDisplay.clear();
  ewDisplay.clear();

  // Initialize pedestrian button
  pinMode(PEDESTRIAN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PEDESTRIAN_BUTTON), pedestrianButtonISR, FALLING);

  setupSensors();
  setupTrafficLights();

  setAllRed();

  Serial.println("Corrected Demand Actuated Traffic Signal System Started");
}

// ======================================================
// Sensor Setup
// ======================================================

void setupSensors() {
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorEnabled[i]) {
      pinMode(echoPins[i], INPUT);
    }

    distanceCM[i] = -1;
    pulseWidth[i] = 0;
    echoDone[i] = false;
  }

  if (sensorEnabled[0]) attachInterrupt(digitalPinToInterrupt(echoPins[0]), echoISR0, CHANGE);
  if (sensorEnabled[1]) attachInterrupt(digitalPinToInterrupt(echoPins[1]), echoISR1, CHANGE);
  if (sensorEnabled[2]) attachInterrupt(digitalPinToInterrupt(echoPins[2]), echoISR2, CHANGE);
  if (sensorEnabled[3]) attachInterrupt(digitalPinToInterrupt(echoPins[3]), echoISR3, CHANGE);
  if (sensorEnabled[4]) attachInterrupt(digitalPinToInterrupt(echoPins[4]), echoISR4, CHANGE);
  if (sensorEnabled[5]) attachInterrupt(digitalPinToInterrupt(echoPins[5]), echoISR5, CHANGE);
  if (sensorEnabled[6]) attachInterrupt(digitalPinToInterrupt(echoPins[6]), echoISR6, CHANGE);
  if (sensorEnabled[7]) attachInterrupt(digitalPinToInterrupt(echoPins[7]), echoISR7, CHANGE);
}

// ======================================================
// Traffic Light Setup
// ======================================================

void setupTrafficLights() {
  pinMode(NS_RED, OUTPUT);
  pinMode(NS_YELLOW, OUTPUT);
  pinMode(NS_GREEN, OUTPUT);

  pinMode(EW_RED, OUTPUT);
  pinMode(EW_YELLOW, OUTPUT);
  pinMode(EW_GREEN, OUTPUT);
}

// ======================================================
// LED Helper Functions
// ======================================================

void writeLED(int pin, bool state) {
  if (LED_ACTIVE_HIGH) {
    digitalWrite(pin, state ? HIGH : LOW);
  } else {
    digitalWrite(pin, state ? LOW : HIGH);
  }
}

void setNorthSouth(bool red, bool yellow, bool green) {
  writeLED(NS_RED, red);
  writeLED(NS_YELLOW, yellow);
  writeLED(NS_GREEN, green);
}

void setEastWest(bool red, bool yellow, bool green) {
  writeLED(EW_RED, red);
  writeLED(EW_YELLOW, yellow);
  writeLED(EW_GREEN, green);
}

void setAllRed() {
  setNorthSouth(true, false, false);
  setEastWest(true, false, false);
}

void setPhaseGreen(Phase phase) {
  if (phase == PHASE_NS) {
    setNorthSouth(false, false, true);
    setEastWest(true, false, false);
  } 
  else if (phase == PHASE_EW) {
    setNorthSouth(true, false, false);
    setEastWest(false, false, true);
  }
}

void setPhaseYellow(Phase phase) {
  if (phase == PHASE_NS) {
    setNorthSouth(false, true, false);
    setEastWest(true, false, false);
  } 
  else if (phase == PHASE_EW) {
    setNorthSouth(true, false, false);
    setEastWest(false, true, false);
  }
}

// ======================================================
// Ultrasonic Reading Functions
// ======================================================

void triggerSensors() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    echoDone[i] = false;
    pulseWidth[i] = 0;
    distanceCM[i] = -1;
  }

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);

  digitalWrite(TRIG_PIN, LOW);
}

void calculateDistances() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (!sensorEnabled[i]) {
      distanceCM[i] = -1;
      continue;
    }

    if (echoDone[i] && pulseWidth[i] > 0) {
      distanceCM[i] = pulseWidth[i] * 0.0343 / 2.0;
    } else {
      distanceCM[i] = -1;
    }
  }
}

void updateStableDetection() {
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (!sensorEnabled[i]) {
      detectedState[i] = false;
      detectStableCount[i] = 0;
      releaseStableCount[i] = 0;
      continue;
    }

    float d = distanceCM[i];

    if (d > 0 && d <= DETECT_DISTANCE) {
      detectStableCount[i]++;
      releaseStableCount[i] = 0;
    } 
    else if (d < 0 || d >= RELEASE_DISTANCE) {
      releaseStableCount[i]++;
      detectStableCount[i] = 0;
    } 
    else {
      detectStableCount[i] = 0;
      releaseStableCount[i] = 0;
    }

    if (!detectedState[i] && detectStableCount[i] >= DETECT_STABLE_READS) {
      detectedState[i] = true;
      detectStableCount[i] = 0;
    }

    if (detectedState[i] && releaseStableCount[i] >= RELEASE_STABLE_READS) {
      detectedState[i] = false;
      releaseStableCount[i] = 0;
    }
  }
}

// ======================================================
// Vehicle Counting
// Entry sensor: +1 on detection
// Crossing sensor: -1 on detection
// Crossing sensor does not directly control traffic light
// ======================================================

void updateVehicleCounts() {
  unsigned long currentTime = millis();

  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (!sensorEnabled[i]) continue;

    bool risingEdge = detectedState[i] && !previousDetectedState[i];

    if (risingEdge) {
      if (currentTime - lastEventTime[i] >= EVENT_COOLDOWN) {

        if (i == 0) {
          northCount++;
          Serial.println("North Entry: Count +1");
        }
        else if (i == 1) {
          if (northCount > 0) {
            northCount--;
            Serial.println("North Crossing: Count -1");
          }
        }
        else if (i == 2) {
          southCount++;
          Serial.println("South Entry: Count +1");
        }
        else if (i == 3) {
          if (southCount > 0) {
            southCount--;
            Serial.println("South Crossing: Count -1");
          }
        }
        else if (i == 4) {
          eastCount++;
          Serial.println("East Entry: Count +1");
        }
        else if (i == 5) {
          if (eastCount > 0) {
            eastCount--;
            Serial.println("East Crossing: Count -1");
          }
        }
        else if (i == 6) {
          westCount++;
          Serial.println("West Entry: Count +1");
        }
        else if (i == 7) {
          if (westCount > 0) {
            westCount--;
            Serial.println("West Crossing: Count -1");
          }
        }

        lastEventTime[i] = currentTime;
      }
    }

    previousDetectedState[i] = detectedState[i];
  }
}

void updateSensors() {
  triggerSensors();

  delay(60);

  calculateDistances();
  updateStableDetection();
  updateVehicleCounts();
}

// ======================================================
// Demand Calculation
// ======================================================

int getNSDemand() {
  return northCount + southCount;
}

int getEWDemand() {
  return eastCount + westCount;
}

int getPhaseDemand(Phase phase) {
  if (phase == PHASE_NS) {
    return getNSDemand();
  } 
  else if (phase == PHASE_EW) {
    return getEWDemand();
  }

  return 0;
}

int getOppositeDemand(Phase phase) {
  if (phase == PHASE_NS) {
    return getEWDemand();
  } 
  else if (phase == PHASE_EW) {
    return getNSDemand();
  }

  return 0;
}

// ======================================================
// Crossing Area Clearance Check
// ======================================================

bool isCrossingAreaClear(Phase phase) {
  if (phase == PHASE_NS) {
    // North Crossing = sensor 1
    // South Crossing = sensor 3
    return !detectedState[1] && !detectedState[3];
  }

  if (phase == PHASE_EW) {
    // East Crossing = sensor 5
    // West Crossing = sensor 7
    return !detectedState[5] && !detectedState[7];
  }

  return true;
}

// ======================================================
// Phase Selection
// ======================================================

Phase chooseNextPhase() {
  int nsDemand = getNSDemand();
  int ewDemand = getEWDemand();

  if (nsDemand == 0 && ewDemand == 0) {
    return PHASE_NONE;
  }

  if (nsDemand > 0 && ewDemand == 0) {
    return PHASE_NS;
  }

  if (ewDemand > 0 && nsDemand == 0) {
    return PHASE_EW;
  }

  // Both phases have demand

  // Anti-starvation:
  // If the last served phase still has more demand, but the opposite phase also has demand,
  // give the opposite phase a chance.
  if (lastServedPhase == PHASE_NS && ewDemand > 0) {
    return PHASE_EW;
  }

  if (lastServedPhase == PHASE_EW && nsDemand > 0) {
    return PHASE_NS;
  }

  // If no starvation issue, choose higher demand
  if (nsDemand > ewDemand) {
    return PHASE_NS;
  }

  if (ewDemand > nsDemand) {
    return PHASE_EW;
  }

  // Equal demand: alternate
  if (lastServedPhase == PHASE_NS) {
    return PHASE_EW;
  }

  if (lastServedPhase == PHASE_EW) {
    return PHASE_NS;
  }

  return PHASE_NS;
}

// ======================================================
// Green Time Calculation
// ======================================================

unsigned long calculateGreenTime(int demand) {
  int usedDemand = demand;

  if (usedDemand > MAX_DEMAND_USED) {
    usedDemand = MAX_DEMAND_USED;
  }

  unsigned long timeValue = MIN_GREEN_TIME + (usedDemand * EXTRA_TIME_PER_CAR);

  if (timeValue > MAX_GREEN_TIME) {
    timeValue = MAX_GREEN_TIME;
  }

  return timeValue;
}

// ======================================================
// Traffic State Machine
// ======================================================

void startGreen(Phase phase) {
  activePhase = phase;

  int demand = getPhaseDemand(phase);
  greenDuration = calculateGreenTime(demand);

  setPhaseGreen(phase);

  currentState = STATE_GREEN;
  stateStartTime = millis();

  Serial.print("GREEN started: ");
  Serial.print(phase == PHASE_NS ? "North-South" : "East-West");
  Serial.print(" | Demand: ");
  Serial.print(demand);
  Serial.print(" | Planned Green Time: ");
  Serial.print(greenDuration / 1000);
  Serial.println(" s");
}

void startYellow() {
  setPhaseYellow(activePhase);

  currentState = STATE_YELLOW;
  stateStartTime = millis();

  Serial.print("YELLOW started: ");
  Serial.println(activePhase == PHASE_NS ? "North-South" : "East-West");
}

void startAllRedClearance() {
  setAllRed();

  currentState = STATE_ALL_RED_CLEAR;
  stateStartTime = millis();

  Serial.println("ALL RED clearance started");
}

void startPedestrianPhase() {
  // Turn all traffic lights RED
  setAllRed();

  currentState = STATE_PEDESTRIAN_PHASE;
  stateStartTime = millis();
  
  // Clear pedestrian request flag when pedestrian phase begins
  pedestrianRequest = false;

  Serial.println("PEDESTRIAN countdown phase started");
}

void updateTrafficController() {
  unsigned long currentTime = millis();

  if (currentState == STATE_ALL_RED_IDLE) {
    setAllRed();

    // Prioritize pedestrian request if active
    if (pedestrianRequest) {
      startPedestrianPhase();
    } else {
      Phase nextPhase = chooseNextPhase();

      if (nextPhase != PHASE_NONE) {
        startGreen(nextPhase);
      }
    }
  }

  else if (currentState == STATE_GREEN) {
    unsigned long elapsed = currentTime - stateStartTime;

    bool plannedTimeFinished = elapsed >= greenDuration;
    bool hardMaxReached = elapsed >= HARD_MAX_GREEN_TIME;
    bool crossingClear = isCrossingAreaClear(activePhase);

    // Corrected rule:
    // Do not change just because count became 0.
    // Change only if planned time is finished and crossing sensors are clear.
    if (plannedTimeFinished && crossingClear) {
      Serial.println("Planned green completed and crossing area clear");
      startYellow();
    }

    // Safety rule:
    // If crossing sensor is stuck, do not stay green forever.
    else if (hardMaxReached) {
      Serial.println("Hard maximum green time reached");
      startYellow();
    }
  }

  else if (currentState == STATE_YELLOW) {
    if (currentTime - stateStartTime >= YELLOW_TIME) {
      startAllRedClearance();
    }
  }

  else if (currentState == STATE_ALL_RED_CLEAR) {
    if (currentTime - stateStartTime >= ALL_RED_TIME) {
      lastServedPhase = activePhase;
      activePhase = PHASE_NONE;

      currentState = STATE_ALL_RED_IDLE;
      stateStartTime = millis();

      Serial.println("Returning to demand check");
    }
  }

  else if (currentState == STATE_PEDESTRIAN_PHASE) {
    if (currentTime - stateStartTime >= PEDESTRIAN_TIME) {
      Serial.println("Pedestrian phase completed");
      
      currentState = STATE_ALL_RED_IDLE;
      stateStartTime = millis();
    }
  }
}

// ======================================================
// Serial Printing
// ======================================================

String getStateName() {
  if (currentState == STATE_ALL_RED_IDLE) return "ALL RED / IDLE";
  if (currentState == STATE_GREEN) return "GREEN";
  if (currentState == STATE_YELLOW) return "YELLOW";
  if (currentState == STATE_ALL_RED_CLEAR) return "ALL RED CLEARANCE";
  if (currentState == STATE_PEDESTRIAN_PHASE) return "PEDESTRIAN PHASE";
  return "UNKNOWN";
}

String getPhaseName(Phase phase) {
  if (phase == PHASE_NS) return "North-South";
  if (phase == PHASE_EW) return "East-West";
  return "None";
}

unsigned long getRemainingTimeSeconds() {
  unsigned long currentTime = millis();
  unsigned long duration = 0;

  if (currentState == STATE_GREEN) {
    duration = greenDuration;
  } 
  else if (currentState == STATE_YELLOW) {
    duration = YELLOW_TIME;
  } 
  else if (currentState == STATE_ALL_RED_CLEAR) {
    duration = ALL_RED_TIME;
  } 
  else if (currentState == STATE_PEDESTRIAN_PHASE) {
    duration = PEDESTRIAN_TIME;
  } 
  else {
    return 0;
  }

  unsigned long elapsed = currentTime - stateStartTime;

  if (elapsed >= duration) {
    return 0;
  }

  return (duration - elapsed + 999) / 1000;
}

void printStatus() {
  Serial.println("--------------------------------");

  Serial.print("N Entry: ");
  Serial.print(distanceCM[0]);
  Serial.print(" cm\tN Cross: ");
  Serial.print(distanceCM[1]);
  Serial.print(" cm\tN Count: ");
  Serial.println(northCount);

  Serial.print("S Entry: ");
  Serial.print(distanceCM[2]);
  Serial.print(" cm\tS Cross: ");
  Serial.print(distanceCM[3]);
  Serial.print(" cm\tS Count: ");
  Serial.println(southCount);

  Serial.print("E Entry: ");
  Serial.print(distanceCM[4]);
  Serial.print(" cm\tE Cross: ");
  Serial.print(distanceCM[5]);
  Serial.print(" cm\tE Count: ");
  Serial.println(eastCount);

  Serial.print("W Entry: ");
  Serial.print(distanceCM[6]);
  Serial.print(" cm\tW Cross: ");
  Serial.print(distanceCM[7]);
  Serial.print(" cm\tW Count: ");
  Serial.println(westCount);

  Serial.print("North-South Demand: ");
  Serial.print(getNSDemand());

  Serial.print("\tEast-West Demand: ");
  Serial.println(getEWDemand());

  Serial.print("State: ");
  Serial.print(getStateName());

  Serial.print("\tActive Phase: ");
  Serial.print(getPhaseName(activePhase));

  Serial.print("\tRemaining Time: ");
  Serial.print(getRemainingTimeSeconds());
  Serial.print(" s");

  Serial.print("\tCrossing Clear: ");
  Serial.println(isCrossingAreaClear(activePhase) ? "YES" : "NO");
}

// ======================================================
// Timer Display Update
// ======================================================

void updateTimerDisplay() {
  static int lastSecondsNS = -1;
  static int lastSecondsEW = -1;
  
  int secondsLeft = getRemainingTimeSeconds();

  if (currentState == STATE_GREEN || currentState == STATE_YELLOW) {
    if (activePhase == PHASE_NS) {
      if (secondsLeft != lastSecondsNS) {
        nsDisplay.showNumberDec(secondsLeft);
        lastSecondsNS = secondsLeft;
      }
      if (lastSecondsEW != -2) {
        ewDisplay.clear();
        lastSecondsEW = -2;
      }
    } else if (activePhase == PHASE_EW) {
      if (secondsLeft != lastSecondsEW) {
        ewDisplay.showNumberDec(secondsLeft);
        lastSecondsEW = secondsLeft;
      }
      if (lastSecondsNS != -2) {
        nsDisplay.clear();
        lastSecondsNS = -2;
      }
    }
  } 
  else if (currentState == STATE_PEDESTRIAN_PHASE) {
    if (secondsLeft != lastSecondsNS) {
      nsDisplay.showNumberDec(secondsLeft);
      lastSecondsNS = secondsLeft;
    }
    if (secondsLeft != lastSecondsEW) {
      ewDisplay.showNumberDec(secondsLeft);
      lastSecondsEW = secondsLeft;
    }
  } 
  else {
    if (lastSecondsNS != -2) {
      nsDisplay.clear();
      lastSecondsNS = -2;
    }
    if (lastSecondsEW != -2) {
      ewDisplay.clear();
      lastSecondsEW = -2;
    }
  }
}

// ======================================================
// Main Loop
// ======================================================

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_TIME) {
    updateSensors();
    lastSensorUpdate = millis();
  }

  updateTrafficController();

  updateTimerDisplay();

  if (millis() - lastSerialPrint >= SERIAL_PRINT_TIME) {
    printStatus();
    lastSerialPrint = millis();
  }
}