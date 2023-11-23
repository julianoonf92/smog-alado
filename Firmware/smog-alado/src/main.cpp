#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <logo.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <TelnetStream.h>
#include <TimeLib.h>
#include <sntp.h>
#include <TZ.h>


#define ledPin 2
#define SDA 4
#define SCL 5
#define buttonPin 12
#define heater 14

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C

#define TIME_ZONE TZ_America_Sao_Paulo

#define tempMax 190
#define ANALOG_RANGE 1024

#define EEPROM_WIFI_SSID_START 0
#define EEPROM_WIFI_PASS_START 64

char customWifiSSID[32];
char customWifiPass[32];

//Heater control
double resistor = 0;
double thermistor = 500;
double heaterTemperature = 0;
double tempGoal = 180;
double powerPercent = 0;
int power = 0;
int preset = 3;
bool debouncedButton = false;
bool state = false;

// PID calculation parameters
double error = 0;
double prevError = 0;
double integral = 0;
double proportional = 0;
double derivative = 0;

// Initial PID values, can be adjusted as needed
double kp = 6.54;
double ki = 0.83;
double kd = 156.13;

//ADC parameters
uint16_t adcRaw = 1000;
uint16_t adcFiltered = 1000;
double adcTimer = 0;
double heaterTimer = 0;
double loopTimer = 0;

// put function declarations here:
void updateDisplay();
void serialPrint();
void autoTunePID();
void TelnetPrint();
void drawXbm(const uint8_t *xbm_data, int16_t width, int16_t height, int16_t x, int16_t y);
void runHeater (int preset);
double resCalc ();
double steinhart (double termistor);
int buttonPress (int button);


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_ADS1115 ads;

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(heater, OUTPUT);
  digitalWrite(ledPin, LOW); //builtin LED set to ON on boot
  digitalWrite(heater, LOW);  //heater set to OFF on boot
  analogWriteRange(ANALOG_RANGE);

  // Set up WiFiManager
  WiFiManager wifiManager;
  wifiManager.setTimeout(30);
  wifiManager.setAPCallback(configModeCallback);

  // Try to load WiFi credentials from EEPROM
  EEPROM.begin(512); // Initialize EEPROM with 512 bytes
  EEPROM.get(EEPROM_WIFI_SSID_START, customWifiSSID);
  EEPROM.get(EEPROM_WIFI_PASS_START, customWifiPass);

  // Set the custom parameters for WiFiManager
  WiFiManagerParameter customSSID("SSID", "WiFi SSID", customWifiSSID, 32);
  WiFiManagerParameter customPass("password", "WiFi Password", customWifiPass, 32);

  wifiManager.addParameter(&customSSID);
  wifiManager.addParameter(&customPass);

  // Try to connect to WiFi, or start a configuration portal if connection fails
  if (!wifiManager.autoConnect("AutoConnectAP")) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    // Reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  // Save WiFi credentials to EEPROM
  strncpy(customWifiSSID, customSSID.getValue(), 32);
  strncpy(customWifiPass, customPass.getValue(), 32);
  EEPROM.put(EEPROM_WIFI_SSID_START, customWifiSSID);
  EEPROM.put(EEPROM_WIFI_PASS_START, customWifiPass);
  EEPROM.commit();

  // If you get here, you have connected to the WiFi
  Serial.println("Connected to WiFi");

  // Initialize OTA
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_FS
      type = "filesystem";

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    ESP.reset();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  if (!(ads.begin())){
    Serial.println("Failed to initialize ADS.");
  }
  ads.setGain(GAIN_ONE);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    for(;;);
  }

  configTime(TIME_ZONE, "pool.ntp.org");
  time_t now = time(nullptr);
  while (now < SECS_YR_2000) {
    delay(100);
    now = time(nullptr);
  }
  setTime(now);

  TelnetStream.begin();

  display.clearDisplay();
  drawXbm(logo_bits, _width, _height, 0, 0);
  delay(10000);
  display.clearDisplay();
}

void loop() {
  ArduinoOTA.handle();
  updateDisplay();
  serialPrint();
  
  if ((millis()-adcTimer) > 200){
    adcTimer = millis();
    thermistor = resCalc();
    heaterTemperature = steinhart(thermistor);
    runHeater(preset);
  }

  if (!digitalRead(buttonPin)){
    debouncedButton = buttonPress(buttonPin);
    if (debouncedButton){
      preset++;
    }
    //Serial.printf("Clict Clect \n");
    Serial.printf("Clict Clect \n");
    delay(100);
    if (preset >= 4)
      preset = 0;
  }

  if ((millis()-loopTimer) > 1000){
    loopTimer=millis();
    digitalWrite(ledPin, state);
    state = !(state);
    //double powerPercent = 100*power/ANALOG_RANGE;
  }
}

// put function definitions here:
double resCalc (){
  int pulldownResistor = 1000;
  double resistor = 0;
  adcRaw = ads.readADC_SingleEnded(0);
  adcFiltered = 0.9 * adcFiltered + 0.1 * adcRaw;
  double adcVoltage = ads.computeVolts(adcFiltered);
  resistor = (3.3 / (adcVoltage)) * pulldownResistor - pulldownResistor;
  if (resistor <50 )
    resistor = 50;
  if (resistor > 120000)
    resistor = 120000;
  return resistor;
}

double steinhart (double termistor){
  //NTC 3950 100k
  /* const static double  a = 0.0008002314855002526;
  const static double  b = 0.0001989545566222665;
  const static double  c = 1.7249962319615102e-7; */

  //NTC 3950 10k
  const static double  a = 0.0011260101763638105;
  const static double  b = 0.00023990205585764816;
  const static double  c = -3.1848655700239605e-8;

  // Utilizes the Steinhart-Hart Thermistor Equation:
  // Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3}
  double ln_r = log(termistor); // Saving the Log(resistance) so not to calculate it 4 times later.
  float temperature = (1 / (a + (b * ln_r) + (c * ln_r * ln_r * ln_r))) - 273.15; //Convert K to ºC
  return (temperature); 
}

void autoTunePID() {
  ArduinoOTA.handle();
  updateDisplay();
  serialPrint();
  TelnetPrint();
  const int tuningDuration = 1200000;  // 10 minutes in milliseconds
  unsigned long startTime = millis();
  double maxTemperature = 0;
  double minTemperature = tempMax;

  // Variables for PID control
  double setpoint = tempGoal;
  double output;

    while ((millis() - startTime) < tuningDuration) {
      thermistor = resCalc();
      heaterTemperature = steinhart(thermistor);

      if (heaterTemperature > maxTemperature) {
        maxTemperature = heaterTemperature;
      }

      if (heaterTemperature < minTemperature) {
        minTemperature = heaterTemperature;
      }
  
  	// Check for button press to change preset
      if (!digitalRead(buttonPin)) {
        debouncedButton = buttonPress(buttonPin);
        if (debouncedButton) {
          preset++;
          // End tuning if button is pressed
          Serial.println("Exiting PID tuning");
          return;
        }
      }

      double error = setpoint - heaterTemperature;

      // Calculate PID terms
      proportional = error;
      integral += (error + prevError) / 2.0;  // Trapezoidal rule for integration
      derivative = error - prevError;

      if (integral > 832) integral = 832;
      if (integral < -832) integral = -832;
      if (derivative > ANALOG_RANGE) derivative = ANALOG_RANGE;
      if (derivative < -ANALOG_RANGE) derivative = -ANALOG_RANGE;

      // Calculate PID output
      output = kp * proportional + ki * integral + kd * derivative;

      // Actuate the heater based on the PID output
      if (output > 0) {
        // Convert the PID output to a PWM value
        power = output;
  		  if (power > ANALOG_RANGE) power = ANALOG_RANGE;
        if (error > 25) power = ANALOG_RANGE;
        analogWrite(heater, power);
      } else {
        digitalWrite(heater, LOW);
      }

      // Save current values for the next iteration
      prevError = error;

      delay(1000);  // Adjust as needed based on your system's response time
    }

    // Calculate Ku and Pu for Ziegler-Nichols method
    double Ku = 4 * (2 * tempGoal) / (maxTemperature - minTemperature);
    double Pu = (millis() - startTime) / 1000.0 / (2 * 3.14);  // convert to seconds

    // Use Ziegler-Nichols ultimate gain and oscillation period to set PID parameters
    kp = (0.6 * Ku);
    ki = (2 * kp / Pu);
    kd = (kp * Pu / 8);

    Serial.println("Automatic PID tuning complete.");
    Serial.print("kp: ");
    Serial.println(kp);
    Serial.print("ki: ");
    Serial.println(ki);
    Serial.print("kd: ");
    Serial.println(kd);

    // Perform cooldown after tuning
    Serial.println("Cooldown in progress...");
    while (heaterTemperature > 35) {
      thermistor = resCalc();
      heaterTemperature = steinhart(thermistor);
      digitalWrite(heater, LOW);  // Turn off the heater during cooldown
      delay(1000);
    }

  // Now proceed to preset 3
  preset = 3;

  Serial.println("Cooldown complete. Proceeding to PID control.");
  updateDisplay();
}

void runHeater(int preset) {
  ArduinoOTA.handle();
  updateDisplay();
  serialPrint();
  TelnetPrint();
  power = 0;
  error = tempGoal - heaterTemperature;
  switch (preset){
    case 0:
      power = 0;
      break;
    case 1: //auto-tune PID using Ziegler-Nichols method
      Serial.println("Auto tuning PID");
      autoTunePID();
      break;
    case 2: //open loop preheating at 40% power
          if (heaterTemperature < 120)
        power = (40*ANALOG_RANGE)/ANALOG_RANGE;
      else
        power = 0;
      break;
    case 3: 
      if (heaterTemperature < tempMax){
        if (error > 25)
          power = ANALOG_RANGE;
        else {
          // Calculate proportional, integral, and derivative terms
          proportional = error;
          integral += (error + prevError) / 2.0;  // Trapezoidal rule for integration
          derivative = error - prevError;
          
          // Limit integral to prevent windup
          if (integral > 832) integral = 832;
          if (integral < -832) integral = -832;

          // Calculate power using PID terms
          power = kp * proportional + ki * integral + kd * derivative;

          // Limit power to the allowable range
          if (power > ANALOG_RANGE) power = ANALOG_RANGE;
          if (power < 0) power = 0;

          prevError = error;
        }
      } 
      break;
    default:
      Serial.printf("No valid power option \n");
  }
  powerPercent = 100 * power/ANALOG_RANGE;
  if (power > 0)
    analogWrite (heater, power);
  else
    digitalWrite (heater, LOW);
}

void serialPrint() {
  Serial.print(">Filtered ADC: ");
  Serial.println(adcFiltered);
  Serial.printf(">Temperature Goal: ");
  Serial.println(tempGoal);
  Serial.print(">Power output: ");
  Serial.println(powerPercent);
  Serial.print(">Thermistor resistance: ");
  Serial.println(thermistor);
  Serial.print(">Temperature reading: ");
  Serial.println(heaterTemperature);
  Serial.print(">Proportional part: ");
  Serial.println(proportional * kp);
  Serial.print(">Integral part: ");
  Serial.println(integral * ki);
  Serial.print(">Derivative part: ");
  Serial.println(derivative * kd);
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1); // Set text size to 2 (you can adjust the size as needed)
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("Temperature: ");
  display.print(heaterTemperature);
  display.print((char)247);
  display.print("C");

  display.setCursor(0, 16); // Adjust vertical position
  display.print("Temp. Goal:  ");
  display.print(tempGoal);
  display.print((char)247);
  display.print("C");

  display.setCursor(0, 32); // Adjust vertical position
  display.print("PWM: ");
  display.print(powerPercent);
  display.print("%");

  // Display the IP address
  display.setCursor(0, 48); // Adjust vertical position
  display.print("IP: ");
  display.print(WiFi.localIP());

  display.display();
}

void TelnetPrint() {
  static int i = 0;

  char timeStr[20];
  sprintf(timeStr, "%02d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());

  TelnetStream.print(i++);
  TelnetStream.print(" ");
  TelnetStream.print(timeStr);
  TelnetStream.print(">Filtered ADC: ");
  TelnetStream.println(adcFiltered);
  TelnetStream.printf(">Temperature Goal: ");
  TelnetStream.println(tempGoal);
  TelnetStream.print(">Power output: ");
  TelnetStream.println(powerPercent);
  TelnetStream.print(">Thermistor resistance: ");
  TelnetStream.println(thermistor);
  TelnetStream.print(">Temperature reading: ");
  TelnetStream.println(heaterTemperature);
  TelnetStream.print(">Proportional part: ");
  TelnetStream.println(proportional * kp);
  TelnetStream.print(">Integral part: ");
  TelnetStream.println(integral * ki);
  TelnetStream.print(">Derivative part: ");
  TelnetStream.println(derivative * kd);
}

int buttonPress(int button) {
  int count = 0;
  while(!digitalRead(button)){
    count++;
    delay(10);
  if (count>10)
    return 1;
  }
  return 0; 
}

void drawXbm(const uint8_t *xbm_data, int16_t width, int16_t height, int16_t x, int16_t y) {
  display.drawXBitmap(x, y, xbm_data, width, height, SSD1306_WHITE);
  display.display();
}