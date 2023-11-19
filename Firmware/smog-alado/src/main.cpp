#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define ledPin 2
#define SDA 4
#define SCL 5
#define buttonPin 12
#define heater 14

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3D

#define tempMax 190
#define ANALOG_RANGE 1024

// put function declarations here:
double resCalc ();
double steinhart (double termistor);
void runHeater (int preset);
int buttonPress (int button);

double thermistor = 500;
double heaterTemperature = 0;
double tempGoal = 180;
double powerPercent = 0;

double error = 0;
double prevError = 0;
double integral = 0;
double proportional = 0;
double derivative = 0;

int power = 0;
int preset = 3;
bool debouncedButton = false;
bool state = false;
uint16_t adcRaw = 1000;
uint16_t adcFiltered = 1000;

double adcTimer = 0;
double heaterTimer = 0;
double loopTimer = 0;

double kp = 6.54;  // Initial values, can be adjusted as needed
double ki = 0.83;
double kd = 156.13;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADS1115 ads;

void updateDisplay();

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(heater, OUTPUT);
  digitalWrite(ledPin, LOW); //builtin LED set to ON on boot
  digitalWrite(heater, LOW);  //heater set to OFF on boot

  if (!(ads.begin())){
    Serial.println("Failed to initialize ADS.");
  }
  ads.setGain(GAIN_ONE);
  Serial.begin(115200);
  WiFi.mode(WIFI_OFF);
  analogWriteRange(ANALOG_RANGE);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
    for(;;);
  }

  display.display();
  delay(2000);
  display.clearDisplay();
  display.drawPixel(10, 10, SSD1306_WHITE);
  display.display();
  delay(2000);
}

void loop() {
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
    double powerPercent = 100*power/ANALOG_RANGE;
    Serial.print(">Thermistor resistence: ");
    Serial.println(thermistor);
    Serial.print(">Temperature reading: ");
    Serial.println(heaterTemperature);
    Serial.print(">Power output: ");
    Serial.println(powerPercent);
    Serial.print(">Filtered ADC: ");
    Serial.println(adcFiltered);
    Serial.printf(">Temperature Goal: ");
    Serial.println(tempGoal);
    updateDisplay();
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
      double proportional = error;
      integral += (error + prevError) / 2.0;  // Trapezoidal rule for integration
      double derivative = error - prevError;

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

      // Print for monitoring progress
      Serial.print(">Proportional part: ");
      Serial.println(kp * proportional);
      Serial.print(">Integral part: ");
      Serial.println(ki * integral);
      Serial.print(">Derivative part: ");
      Serial.println(kd * derivative);

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
      Serial.print(">Thermistor resistance: ");
      Serial.println(thermistor);
      Serial.print(">Temperature reading: ");
      Serial.println(heaterTemperature);
      digitalWrite(heater, LOW);  // Turn off the heater during cooldown
      delay(1000);
    }

  // Now proceed to preset 3
  preset = 3;

  Serial.println("Cooldown complete. Proceeding to PID control.");
  updateDisplay();
}

void runHeater(int preset) {
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
  Serial.print(">Proportional part: ");
  Serial.println(proportional * kp);
  Serial.print(">Integral part: ");
  Serial.println(integral * ki);
  Serial.print(">Derivative part: ");
  Serial.println(derivative * kd);
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("Temp: ");
  display.print(heaterTemperature);
  display.print("Goal: ");
  display.print(tempGoal);
  display.print(" ºC");

  display.setCursor(0, 20);
  display.print("PWM: ");
  display.print(powerPercent);
  display.print("%");

  display.display();
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