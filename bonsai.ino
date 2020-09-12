#include <Wire.h>
#include <SerLCD.h> // http://librarymanager/All#SparkFun_SerLCD
#include <I2CSoilMoistureSensor.h> // https://github.com/Apollon77/I2CSoilMoistureSensor
#include <NewPing.h> // https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home

// Customizable pin constants
const unsigned int SONAR_TRIGGER_PIN = 6;
const unsigned int SONAR_ECHO_PIN = 7;
const unsigned int MANUAL_MODE_BTN = 4;
const unsigned int PUMP_PIN = 5;
const unsigned int POTENTIOMETER_PIN = A0;

// Customizable constants
const unsigned int FULL_WATER_LVL = 415; // Sonar ping in uS (57 uS = 1 cm) for full water
const unsigned int LOW_WATER_LVL = 700; // Sonar ping in uS (57 uS = 1 cm) for low water warning
const unsigned int NO_WATER_LVL = 770; // Sonar ping in uS (57 uS = 1 cm) for no water
const unsigned int WATERING_TIME = 15000; // Duration in ms that the pump will be active when watering
const unsigned long MEASUREMENT_DELAY = 600000; // Delay in ms between temp/humidity measures

// Global variables
unsigned long next_measurement, rgb_backlight, last_poten_adjust;
bool manual_mode;
String status_message;
int temperature, humidity, min_humidity;

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
  temperature = (sensor.getTemperature() + 5) / 10; // In Celsius with proper rounding
  next_measurement = millis() + MEASUREMENT_DELAY;

  // Return true if humidity is low
  return humidity < min_humidity;
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
    rgb_backlight = 0xccffcc; // Green
    return true;
  }
}


// Start the pump for WATERING_TIME duration
void pumpWater() {
  digitalWrite(PUMP_PIN, HIGH);
  delay(WATERING_TIME);
  digitalWrite(PUMP_PIN, LOW);
}


// Get the potentiometer value and convert it to humidity capacitance, then set min_humidity. 
void getMinHumidity() {
  int poten = map(analogRead(POTENTIOMETER_PIN), 0, 1023, 270, 370);
  
  if (poten != min_humidity) {
    min_humidity = poten;
    status_message = String("HUMIDITE MIN " + String(min_humidity));
    displayStatus();
    last_poten_adjust = millis();
  }  
}


// Display temp/hum information on LCD
void displayTempHum() {
  lcd.setCursor(1, 0);
  lcd.print(temperature);
  lcd.print(char(223));
  lcd.print("C  ");
  lcd.print(humidity);
  lcd.print(" HUMI");
}


// Display status information on LCD
void displayStatus() {
  lcd.setFastBacklight(rgb_backlight);
  lcd.setCursor(0, 1);
  lcd.print(status_message);
}


void setup() {
  
  Wire.begin();

  // Set pin modes
  pinMode(POTENTIOMETER_PIN, INPUT);
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
  lcd.setCursor(2, 0);
  if (manual_mode) {
    lcd.print("MODE MANUEL");
    rgb_backlight = 0x99ccff; // Blue
  } else {
    lcd.print(" MODE AUTO");
    rgb_backlight = 0xccffcc; // Green
  }
    
  // I2CSoilMoisture - Init
  sensor.begin();
}


void loop() {

  // Manual mode - Pump continuously
  while (manual_mode && enoughWater()) { pumpWater(); }
  
  // Adjust min_humidity using the potentiometer
  // Keep checking as long as it has not been adjusted for 2 sec
  do { getMinHumidity(); }
  while ((millis() - last_poten_adjust) < 2000);
 
  // Auto mode - Pump when humidity is below min_humidity
  if (millis() > next_measurement && checkTempHum() & enoughWater()) { 
    displayTempHum();
    displayStatus();
    pumpWater();
  }

  // Display temp, humidity and water lvl on LCD
  displayTempHum();
  displayStatus();
}
