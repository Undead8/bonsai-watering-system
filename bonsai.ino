#include <Wire.h>
#include <SerLCD.h> // http://librarymanager/All#SparkFun_SerLCD
#include <I2CSoilMoistureSensor.h> // https://github.com/Apollon77/I2CSoilMoistureSensor
#include <NewPing.h> // https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home

// Pin constants
const unsigned int SONAR_TRIGGER_PIN = 6;
const unsigned int SONAR_ECHO_PIN = 7;
const unsigned int MANUAL_MODE_BTN = 4;
const unsigned int PUMP_HEATER_PIN = 5;

// Other customizable constants
const unsigned int FULL_WATER_LVL = 285; // Sonar ping in uS (57 uS = 1 cm) for full water
const unsigned int LOW_WATER_LVL = 627; // Sonar ping in uS (57 uS = 1 cm) for low water warning
const unsigned int NO_WATER_LVL = 741; // Sonar ping in uS (57 uS = 1 cm) for no water
const unsigned int WATERING_TIME = 5000; // Duration in ms that the pump will be active when watering
const unsigned long MEASUREMENT_DELAY = 300000; // Delay in ms between temp/humidity measures
const double MIN_HUMIDITY = 213.0; // Minimum humidity in Capacitance that can be reached before watering

// Other global variables
unsigned long last_measurement;
double humidity, temperature;
bool manual_mode;
String status_message;
unsigned long rgb_backlight = 0xffffff; // White

// Library objects
SerLCD lcd; // Default I2C address 0x72
I2CSoilMoistureSensor sensor; // Default I2C address is 0x20
NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, 100);


// Check temperature and humidity
void checkTempHum() {
    
  // Use I2CSoilMoisture to measure temperature and humidity
  sensor.getCapacitance();
  while (sensor.isBusy()) delay(50);
  sensor.getTemperature();
  while (sensor.isBusy()) delay(50);
  humidity = sensor.getCapacitance(); // In Capacitance
  temperature = sensor.getTemperature() / 10.0; // In Celsius   
  last_measurement = millis();

  // Serial debug
  Serial.print("Temp: ");
  Serial.println(temperature);
  Serial.print("Humi: ");
  Serial.println(humidity);
}


// Check if there is enough water
bool enoughWater() {

  // Ping the distance to the water
  int sonar_ping = sonar.ping();

  // Serial debug
  Serial.print("Sonar distance: ");
  Serial.print(sonar_ping);
  Serial.println(" uS");

  // Status message for water lvl
  unsigned int water_percent = map(sonar_ping, NO_WATER_LVL, FULL_WATER_LVL, 0, 100);
  status_message = String("    EAU " + String(water_percent) + "%");

  // If the distance to the water is large, there is no more water
  if (sonar_ping > NO_WATER_LVL) {

    // Serial debug
    Serial.println("No water alert");

    // LCD variables
    rgb_backlight = 0xff0000; // Red
    
    return false;
    
  // If the distance to the water is medium, water is low
  } else if (sonar_ping > LOW_WATER_LVL) {

    // Serial debug
    Serial.println("Low water alert");

    // LCD variables
    rgb_backlight = 0xffff00; // Yellow
    
    return true;

  // If the sonar returns 0, there is a reading error
  } else if (sonar_ping == 0) {

    // Serial debug
    Serial.println("Sonar error alert");

    // LCD variables
    rgb_backlight = 0xff00ff; // Pink
    status_message = "     ERREUR";
    
    return false;

  // If the distance to the water is low, there is enough water
  } else {
    
    // Serial debug
    Serial.println("Enough water");

    return true;
  }
}


// Start the pump for WATERING_TIME duration
void pumpWater() {

  // Pump water for WATERING_TIME ms
  digitalWrite(PUMP_HEATER_PIN, HIGH);
  Serial.println("Pump on");
  delay(WATERING_TIME);
  digitalWrite(PUMP_HEATER_PIN, LOW);
  Serial.println("Pump off");

  // Update temp, humidity and water lvl.
  checkTempHum();
  enoughWater();
  displayLCD();
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
  
  Serial.begin(9600);
  Wire.begin();

  // Set pin modes
  pinMode(MANUAL_MODE_BTN, INPUT_PULLUP);
  pinMode(PUMP_HEATER_PIN, OUTPUT);
  digitalWrite(PUMP_HEATER_PIN, LOW);

  // Set auto or manual mode
  manual_mode = digitalRead(MANUAL_MODE_BTN) == LOW;
  Serial.print("Manual mode: ");
  Serial.println(manual_mode);

  // LCD - Init
  Serial.println("Initializing LCD");
  lcd.begin(Wire);
  delay(1000);
  lcd.clear();
  lcd.print("  ROBO-BONSAI");
  lcd.setCursor(0, 1);
  lcd.print("   MODE ");
  if (manual_mode) {
    lcd.print("MANUEL");
    rgb_backlight = 0x99ccff; // Blue
    lcd.setFastBacklight(rgb_backlight);
  } else {
    lcd.print("AUTO");
    rgb_backlight = 0xccffcc; // Green
    lcd.setFastBacklight(rgb_backlight);
  }
  Serial.println("LCD initialized");
  
  // I2CSoilMoisture - Init
  Serial.println("Initializing I2CSoilMoisture");
  sensor.begin();
  delay(1000);
  Serial.println("I2CSoilMoisture initialized");  

  // Delay for splash screen
  delay(2000);

  // Check temp, humidity and water and display on LCD
  checkTempHum();
  enoughWater();
  displayLCD();

  Serial.println("Setup completed");
}


void loop() {

  if (manual_mode) {

    // Pump water non-stop until there is no more water
    while (enoughWater()) { pumpWater(); }

    // When there is no water, update temp and humidity every MEASUREMENT_DELAY
    while ((millis() - last_measurement) < MEASUREMENT_DELAY) { delay(1); }
    checkTempHum();
    displayLCD();
       
  } else {
    
    // Update temp and humidity every MEASUREMENT_DELAY
    while ((millis() - last_measurement) < MEASUREMENT_DELAY) { delay(1); }
    checkTempHum();
    displayLCD();

    // Pump water once if there is water and humidity is low
    if (enoughWater() && humidity < MIN_HUMIDITY) { pumpWater(); } 
  }
}
