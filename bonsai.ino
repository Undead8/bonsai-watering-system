#include <Wire.h>
#include <SerLCD.h> // http://librarymanager/All#SparkFun_SerLCD
#include <I2CSoilMoistureSensor.h> // https://github.com/Apollon77/I2CSoilMoistureSensor
#include <NewPing.h> // https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home

// Customizable pin constants
const unsigned int SONAR_TRIGGER_PIN = 6;
const unsigned int SONAR_ECHO_PIN = 7;
const unsigned int MANUAL_MODE_BTN = 4;
const unsigned int PUMP_PIN = 5;

// Customizable constants
const unsigned int FULL_WATER_LVL = 415; // Sonar ping in uS (57 uS = 1 cm) for full water
const unsigned int LOW_WATER_LVL = 700; // Sonar ping in uS (57 uS = 1 cm) for low water warning
const unsigned int NO_WATER_LVL = 770; // Sonar ping in uS (57 uS = 1 cm) for no water
const unsigned int WATERING_TIME = 15000; // Duration in ms that the pump will be active when watering
const unsigned int WATERING_DELAY = 120000; // Duration in ms before watering another time
const unsigned long MEASUREMENT_DELAY = 600000; // Delay in ms between temp/humidity measures
const double MIN_HUMIDITY = 325.0; // Minimum humidity in Capacitance that can be reached before watering

// Global variables
unsigned long last_measurement;
double humidity, temperature;
bool manual_mode;
String status_message;
unsigned long rgb_backlight = 0xffffff; // White

// Library objects
SerLCD lcd; // Default I2C address is 0x72
I2CSoilMoistureSensor sensor; // Default I2C address is 0x20
NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, 100);


// Check temperature and humidity, return true if low humidity
bool checkTempHum() {
    
  // Use I2CSoilMoisture to measure temperature and humidity
  sensor.getCapacitance();
  while (sensor.isBusy()) { delay(50); }
  sensor.getTemperature();
  while (sensor.isBusy()) { delay(50); }
  humidity = sensor.getCapacitance(); // In Capacitance
  temperature = sensor.getTemperature() / 10.0; // In Celsius   
  last_measurement = millis();

  // Return true if humidity is low
  return humidity < MIN_HUMIDITY;
}


// Check if there is enough water
bool enoughWater() {

  // Ping the distance to the water
  int sonar_ping = sonar.ping();

  // Try again up to 4 times if error
  for (int i = 0; sonar_ping == 0 && i < 4; i++) {
    delay(250);
    sonar_ping = sonar.ping();
  }

  // Status message for LCD
  int water_percent = map(min(sonar_ping, NO_WATER_LVL), NO_WATER_LVL, FULL_WATER_LVL, 0, 100);
  status_message = String("    EAU " + String(water_percent) + "%");

  // If the distance to the water is large, there is no more water
  if (sonar_ping > NO_WATER_LVL) {

    // LCD variables
    rgb_backlight = 0xff0000; // Red
    
    return false;
    
  // If the distance to the water is medium, water is low
  } else if (sonar_ping > LOW_WATER_LVL) {

    // LCD variables
    rgb_backlight = 0xffff00; // Yellow
    
    return true;

  // If the sonar returns 0, there is a reading error
  } else if (sonar_ping == 0) {

    // LCD variables
    rgb_backlight = 0xff00ff; // Pink
    status_message = "     ERREUR";
    
    return false;

  // If the distance to the water is low, there is enough water
  } else {

    // LCD variables
    rgb_backlight = 0xffffff; // White
    return true;
  }
}


// Start the pump for WATERING_TIME duration
void pumpWater() {
  digitalWrite(PUMP_PIN, HIGH);
  delay(WATERING_TIME);
  digitalWrite(PUMP_PIN, LOW);
}


// Display information on LCD
void displayLCD() {

  lcd.clear();
  lcd.setFastBacklight(rgb_backlight);
  
  // Display temperature and humidity
  lcd.setCursor(1, 0);
  lcd.print(static_cast<int>(temperature));
  lcd.print(char(223));
  lcd.print("C  ");
  lcd.print(static_cast<int>(humidity));
  lcd.print(" HUMI");

  // Display status message
  lcd.setCursor(0, 1);
  lcd.print(status_message);
}


void setup() {
  
  Wire.begin();

  // Set pin modes
  pinMode(MANUAL_MODE_BTN, INPUT_PULLUP);
  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  // Set auto or manual mode
  manual_mode = digitalRead(MANUAL_MODE_BTN) == LOW;

  // Delay before begin
  delay(1000);

  // LCD - Init and splash screen
  lcd.begin(Wire);
  lcd.clear();
  lcd.print("  ROBO-BONSAI");
  lcd.setCursor(0, 1);
  if (manual_mode) {
    lcd.print("  MODE MANUEL");
    rgb_backlight = 0x99ccff; // Blue
    lcd.setFastBacklight(rgb_backlight);
  } else {
    lcd.print("   MODE AUTO");
    rgb_backlight = 0xccffcc; // Green
    lcd.setFastBacklight(rgb_backlight);
  }
  
  // I2CSoilMoisture - Init
  sensor.begin();

  // Delay for splash screen
  delay(2000);
}


void loop() {

  // Manual mode loop
  while (manual_mode && enoughWater()) { pumpWater(); }
 
  // Auto mode loop
  while (checkTempHum() & enoughWater()) { 
    displayLCD();
    pumpWater();
    delay(WATERING_DELAY);
  }

  // Display temp, humidity and water lvl on LCD
  displayLCD();

  // Put on stand-by
  while ((millis() - last_measurement) < MEASUREMENT_DELAY) { delay(1); }
}
