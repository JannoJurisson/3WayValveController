#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include "printf.h"
#include "RF24.h"

#define TEMPERATURE_PRECISION 9

#define CE_PIN 9
#define CSN_PIN 10
// RF24 configuration
RF24 radio(CE_PIN, CSN_PIN);
uint8_t address[][6] = { "1Node", "2Node" };
bool radioNumber = 0;  // 0 uses address[0] to transmit, 1 uses address[1] to transmit
bool role = true;  // true = TX role, false = RX role
float payload = 0.0;

// Structure to send two temperatures
struct TempData {
  float temp1;
  float temp2;
  float temp3;
};
  TempData data;

#define ONE_WIRE_BUS 7  // Dallas temp sensor data pin
#define SERVO_PIN 3
#define MIN_LIMIT_SWITCH 8  // Active LOW (0 when pressed)
#define MAX_LIMIT_SWITCH 5
#define LED_STATUS 6


#define BTN_UP A0
#define BTN_DOWN A4
#define BTN_LEFT 4
#define BTN_RIGHT A1
#define BTN_OK A3

Servo valveServo;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
// arrays to hold device addresses
DeviceAddress soilThermometer, valveThermometer, airThermometer;

double setpoint, Kp, Ki, Kd, inputTemp, outputServo, setpointTemp;
float currentTemp = 0.0; 
PID myPID(&inputTemp, &outputServo, &setpointTemp, Kp, Ki, Kd, DIRECT);
LiquidCrystal_I2C lcd(0x27, 16, 2);

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


unsigned long currentMillis;
unsigned long prevMillis;
int LED_delay = 1000;

void setup() {
  Serial.begin(9600);
  Serial.println("start");
  sensors.begin();
  // locate devices on the bus
  Serial.print("Locating devices...");
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  if (!sensors.getAddress(soilThermometer, 0)) Serial.println("Unable to find address for Device 0");
  if (!sensors.getAddress(valveThermometer, 1)) Serial.println("Unable to find address for Device 1");
  if (!sensors.getAddress(airThermometer, 2)) Serial.println("Unable to find address for Device 2");

  Serial.print("Device 0 Address: ");
  printAddress(soilThermometer);
  Serial.println();

  Serial.print("Device 1 Address: ");
  printAddress(valveThermometer);
  Serial.println();

  Serial.print("Device 2 Address: ");
  printAddress(airThermometer);
  Serial.println();


  sensors.setResolution(soilThermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(valveThermometer, TEMPERATURE_PRECISION);
    sensors.setResolution(airThermometer, TEMPERATURE_PRECISION);


  pinMode(MIN_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(MAX_LIMIT_SWITCH, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_LEFT, INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);
  pinMode(LED_STATUS, OUTPUT);

  EEPROM.get(0, setpoint);
  EEPROM.get(4, Kp);
  EEPROM.get(8, Ki);
  EEPROM.get(12, Kd);

  if (isnan(setpoint)) setpoint = 40.0;
  if (isnan(Kp)) Kp = 3.0;
  if (isnan(Ki)) Ki = 0.5;
  if (isnan(Kd)) Kd = 1.0;

  if (!radio.begin()) {
    Serial.println(F("radio hardware is not responding!!"));
  }
 radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MAX is default.
  radio.setPayloadSize(sizeof(data));  // float datatype occupies 4 bytes
  radio.stopListening(address[radioNumber]);  // put radio in TX mode
  radio.openReadingPipe(1, address[!radioNumber]);  // using pipe 1
  if (!role) {
    radio.startListening();  // put radio in RX mode
  }


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

  valveServo.detach();
  servoAttached = false;

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

  if (millis() - prevMillis > LED_delay) {
    digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));

    prevMillis = millis();
  }


  unsigned long now = millis();

  if (now - lastServoUpdate >= servoUpdateInterval) {

    lastServoUpdate = now;
    currentTemp = readTemperatureDallas();

    //nrf24
  
    data.temp1 = sensors.getTempCByIndex(0);
    data.temp2 = sensors.getTempCByIndex(1);
    data.temp3 = sensors.getTempCByIndex(2);

    Serial.print("Sending data...");
  if (role) {
    // This device is a TX node
payload = data.temp2;
    unsigned long start_timer = micros();                // start the timer
    bool report = radio.write(&data, sizeof(data));  // transmit & save the report
    unsigned long end_timer = micros();                  // end the timer

    if (report) {
      Serial.print(F("Transmission successful! "));  // payload was delivered
      Serial.print(F("Time to transmit = "));
      Serial.print(end_timer - start_timer);  // print the timer result
      Serial.print(F(" us. Sent: "));
      Serial.println(payload);  // print payload sent
      payload += 0.01;          // increment float payload
      LED_delay = 1500;
    } else {
LED_delay = 300;
      Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
    }

    // to make this example readable in the serial monitor
    

  } else {
    // This device is a RX node

    uint8_t pipe;
    if (radio.available(&pipe)) {              // is there a payload? get the pipe number that received it
      uint8_t bytes = radio.getPayloadSize();  // get the size of the payload
      radio.read(&payload, bytes);             // fetch payload from FIFO
      Serial.print(F("Received "));
      Serial.print(bytes);  // print the size of the payload
      Serial.print(F(" bytes on pipe "));
      Serial.print(pipe);  // print the pipe number
      Serial.print(F(": "));
      Serial.println(payload);  // print the payload's value
    }
  }  // role


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
    lcd.print(data.temp1, 1);
    lcd.print("C");

    lcd.setCursor(9, 1);
    lcd.print("V: ");
    lcd.print(currentTemp);
    lcd.print("C");

    lcd.setCursor(0, 0);
    lcd.print("A: ");
    lcd.print(data.temp3, 1);
    lcd.print("C");

    unsigned long ms = millis();
    unsigned long totalSeconds = ms / 1000;

    unsigned int seconds = totalSeconds % 60;
    unsigned int minutes = (totalSeconds / 60) % 999999;
    unsigned int hours = (totalSeconds / 3600) % 24;
    unsigned int days = totalSeconds / 86400;
   // lcd.print("d");
   // lcd.setCursor(1, 0);
   // lcd.print(days);
    //lcd.setCursor(4, 0);
   // lcd.print("h:");
   // lcd.setCursor(6, 0);
   // lcd.print(hours);
    lcd.setCursor(9, 0);
    lcd.print("m:");
    lcd.setCursor(11, 0);
    lcd.print(minutes);
   // lcd.setCursor(12, 0);
    //lcd.print("s:");
    //lcd.setCursor(14, 0);
    //lcd.print(seconds);
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
  sensors.requestTemperatures();             // Send the command to get temperatures
  float tempC = sensors.getTempCByIndex(1);  // sensor with index should be valve sensor

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


//  Prevents servo from exceeding 90° range using limit switches
bool isWithinLimits(int position) {
  if (digitalRead(MIN_LIMIT_SWITCH) == HIGH && position <= minServoAngle) return false;
  if (digitalRead(MAX_LIMIT_SWITCH) == HIGH && position >= maxServoAngle) return false;
  return true;
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
