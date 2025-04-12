#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>



#define ONE_WIRE_BUS 8  // Dallas temp sensor data pin
#define SERVO_PIN 9
#define MIN_LIMIT_SWITCH 7  // Active LOW (0 when pressed)
#define MAX_LIMIT_SWITCH 6

#define BTN_UP 3
#define BTN_DOWN 2
#define BTN_LEFT 4
#define BTN_RIGHT 5

Servo valveServo;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

double setpoint, Kp, Ki, Kd, inputTemp, outputServo, setpointTemp;
float currentTemp = 0.0;  // ✅ FIXED: Declared global variable
PID myPID(&inputTemp, &outputServo, &setpointTemp, Kp, Ki, Kd, DIRECT);
LiquidCrystal_I2C lcd(0x27, 20, 2);

int menuIndex = 0;  // 0=Set Temp, 1=Kp, 2=Ki, 3=Kd
const char* menuItems[] = { "Set Temp", "Kp Value", "Ki Value", "Kd Value" };

unsigned long lastButtonPress = 0;
bool menuActive = true;

float lastSetpoint = -100.0;
float lastCurrentTemp = -100.0;
unsigned long lastLcdUpdate = 0;
const unsigned long lcdUpdateInterval = 1000;  // Update LCD every 1 second

unsigned long lastButtonPressTime = 0;
const unsigned long buttonDebounceTime = 300;  // 300ms debounce delay
const unsigned long menuTimeout = 5000;        // 10s timeout
int lastServoPos = -1;                         // Store the last servo position

float lastKp = -100.0;
float lastKi = -100.0;
float lastKd = -100.0;

int minServoAngle = 0;
int maxServoAngle = 90;
unsigned long lastServoUpdate = 0;
const unsigned long servoUpdateInterval = 5000;  // 5 seconds
bool servoAttached = false;

void setup() {
  Serial.begin(9600);
  Serial.println("start");
  sensors.begin();

  pinMode(MIN_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(MAX_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);

  EEPROM.get(0, setpoint);
  EEPROM.get(4, Kp);
  EEPROM.get(8, Ki);
  EEPROM.get(12, Kd);

  if (isnan(setpoint)) setpoint = 40.0;
  if (isnan(Kp)) Kp = 3.0;
  if (isnan(Ki)) Ki = 0.5;
  if (isnan(Kd)) Kd = 1.0;



  valveServo.attach(SERVO_PIN);

  lcd.init();
  lcd.backlight();
  showMainScreen();

  //int lastServoPos = loadLastServoPosition();
  // Move toward max limit until the max switch is pressed
  for (int angle = 90; angle <= 180; angle++) {
    valveServo.write(angle);
    Serial.println(angle);
    delay(200);
    if (digitalRead(MAX_LIMIT_SWITCH) == HIGH) {
      maxServoAngle = angle;
      break;  // Max limit reached
    }
  }


  // Move back toward min limit until the min switch is pressed
  for (int angle = maxServoAngle; angle >= 0; angle--) {
    valveServo.write(angle);
    delay(200);
    Serial.println(angle);
    if (digitalRead(MIN_LIMIT_SWITCH) == HIGH) {
      minServoAngle = angle;
      break;  // Min limit reached
    }
  }
  Serial.print("Minimum servo angle = ");
  Serial.println(minServoAngle);
  Serial.print("Maximum servo angle = ");
  Serial.println(maxServoAngle);

  setpointTemp = setpoint;
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(minServoAngle, maxServoAngle);
}

void loop() {



  unsigned long now = millis();

  if (now - lastServoUpdate >= servoUpdateInterval) {
    lastServoUpdate = now;
Serial.println(millis());
    currentTemp = readTemperatureDallas();

    if (currentTemp > -55 && currentTemp < 125) {  // Valid DS18B20 range
      inputTemp = currentTemp;
      setpointTemp = setpoint;
      myPID.Compute();

      int newServoPos = constrain((int)outputServo, minServoAngle, maxServoAngle);

      if (isWithinLimits(newServoPos)) {
        if (abs(newServoPos - lastServoPos) > 1) {
          if (!servoAttached) {
            valveServo.attach(SERVO_PIN);
            servoAttached = true;
          }

          valveServo.write(newServoPos);
          lastServoPos = newServoPos;

          // Detach after short delay to let servo move
          delay(300);
          valveServo.detach();
          servoAttached = false;
        }
      }

      Serial.print("Temp: ");
      Serial.print(currentTemp);
      Serial.print(" °C | Servo: ");
      Serial.println(newServoPos);
    }
  }

  handleMenu();

  if (menuActive) {
    showMenu();

  } else {
    showMainScreen();
  }
}


void handleMenu() {
  unsigned long currentMillis = millis();

  // Check if menu was inactive and a button is pressed
  if (!menuActive) {
    if (digitalRead(BTN_LEFT) == LOW || digitalRead(BTN_RIGHT) == LOW || digitalRead(BTN_UP) == LOW || digitalRead(BTN_DOWN) == LOW) {
      resetTimeout();  // Reactivate menu
      showMenu();
    }
    return;  // Skip the rest of the function until menu is active
  }

  // Timeout: return to main screen after 10 sec
  if (currentMillis - lastButtonPressTime > menuTimeout) {
    menuActive = false;
    showMainScreen();
    return;
  }

  if (currentMillis - lastButtonPressTime < buttonDebounceTime) {
    return;  // Ignore fast button presses
  }

  if (digitalRead(BTN_LEFT) == LOW) {
    menuIndex = (menuIndex + 3) % 4;
    resetTimeout();
  }
  if (digitalRead(BTN_RIGHT) == LOW) {
    menuIndex = (menuIndex + 1) % 4;
    resetTimeout();
  }
  if (digitalRead(BTN_UP) == LOW) {
    adjustValue(menuIndex, 1);
    resetTimeout();
  }
  if (digitalRead(BTN_DOWN) == LOW) {
    adjustValue(menuIndex, -1);
    resetTimeout();
  }

  showMenu();
}

void showMenu() {
  static int lastMenuIndex = -1;      // Track menu index changes
  static bool lastMenuState = false;  // Track menu state changes

  // Only update LCD if menu index or state changed
  if (lastMenuIndex != menuIndex || lastMenuState != menuActive) {
    lcd.clear();  // Clear the screen to remove old data
    lcd.setCursor(0, 0);
    lcd.print(menuItems[menuIndex]);
    lastMenuIndex = menuIndex;   // Update the last menu index
    lastMenuState = menuActive;  // Update the last menu state
  }

  // Update the numerical value only when it changes, or show it on first entry
  lcd.setCursor(0, 1);  // Move to second row
  switch (menuIndex) {
    case 0:  // Setpoint menu
      if (setpoint != lastSetpoint || lastSetpoint == -100.0) {
        lcd.print("Value: ");
        lcd.print(setpoint, 1);   // Update setpoint if changed
        lastSetpoint = setpoint;  // Save the current value
      }
      break;
    case 1:  // Kp menu
      if (Kp != lastKp || lastKp == -100.0) {
        lcd.print("Value: ");
        lcd.print(Kp, 2);  // Update Kp if changed
        lastKp = Kp;       // Save the current value
      }
      break;
    case 2:  // Ki menu
      if (Ki != lastKi || lastKi == -100.0) {
        lcd.print("Value: ");
        lcd.print(Ki, 2);  // Update Ki if changed
        lastKi = Ki;       // Save the current value
      }
      break;
    case 3:  // Kd menu
      if (Kd != lastKd || lastKd == -100.0) {
        lcd.print("Value: ");
        lcd.print(Kd, 2);  // Update Kd if changed
        lastKd = Kd;       // Save the current value
      }
      break;
  }
}

void resetTimeout() {
  lastButtonPressTime = millis();
  menuActive = true;
}


void showMainScreen() {
  unsigned long currentMillis = millis();

  // Update LCD only if values have changed OR every lcdUpdateInterval
  if ((setpoint != lastSetpoint || currentTemp != lastCurrentTemp) || (currentMillis - lastLcdUpdate >= lcdUpdateInterval)) {

    lastLcdUpdate = currentMillis;
    lastSetpoint = setpoint;
    lastCurrentTemp = currentTemp;

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("S: ");
    lcd.print(setpoint, 1);
    lcd.print("C");

    lcd.setCursor(9, 1);
    lcd.print("C: ");
    lcd.print(currentTemp);
    lcd.print("C");

    lcd.setCursor(0, 0);
  unsigned long ms = millis();
  unsigned long totalSeconds = ms / 1000;

  unsigned int seconds = totalSeconds % 60;
  unsigned int minutes = (totalSeconds / 60) % 60;
  unsigned int hours = (totalSeconds / 3600) % 24;
  unsigned int days = totalSeconds / 86400;
    lcd.print("d");
    lcd.setCursor(1, 0);
    lcd.print(days);
    lcd.setCursor(4, 0);
    lcd.print("h:");
    lcd.setCursor(6, 0);
    lcd.print(hours);
    lcd.setCursor(8, 0);
    lcd.print("m:");
    lcd.setCursor(10, 0);
    lcd.print(minutes);
        lcd.setCursor(12, 0);
    lcd.print("s:");
    lcd.setCursor(14, 0);
    lcd.print(seconds);
  }
}


void adjustValue(int index, int step) {
  switch (index) {
    case 0:
      setpoint += step * 0.5;
      EEPROM.put(0, setpoint);
      break;
    case 1:
      Kp += step * 0.1;
      EEPROM.put(4, Kp);
      myPID.SetTunings(Kp, Ki, Kd);
      break;
    case 2:
      Ki += step * 0.1;
      EEPROM.put(8, Ki);
      myPID.SetTunings(Kp, Ki, Kd);
      break;
    case 3:
      Kd += step * 0.1;
      EEPROM.put(12, Kd);
      myPID.SetTunings(Kp, Ki, Kd);
      break;
  }
}

float readTemperatureDallas() {
  sensors.requestTemperatures();  // Send the command to get temperatures
  float tempC = sensors.getTempCByIndex(0);

  return tempC;
}

void saveServoPosition(int position) {
  EEPROM.put(16, position);  // Store at address 16 (just after Kd at 12)
}

int loadLastServoPosition() {
  int pos;
  EEPROM.get(16, pos);
  if (pos < 0 || pos > 180) {
    pos = 90;  // Default safe middle position
  }
  return pos;
}


// ✅ Prevents servo from exceeding 90° range using limit switches
bool isWithinLimits(int position) {
  if (digitalRead(MIN_LIMIT_SWITCH) == HIGH && position <= minServoAngle) return false;
  if (digitalRead(MAX_LIMIT_SWITCH) == HIGH && position >= maxServoAngle) return false;
  return true;
}
