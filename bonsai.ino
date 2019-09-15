#include <Wire.h>
#include <SerLCD.h> // http://librarymanager/All#SparkFun_SerLCD
#include <DFRobot_SHT20.h> // https://github.com/DFRobot/DFRobot_SHT20
#include <NewPing.h> // https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library

// Pin constants
const unsigned int SONAR_TRIGGER_PIN = 2;
const unsigned int SONAR_ECHO_PIN = 3;
const unsigned int WINTER_MODE_BTN = 4;
const unsigned int PUMP_HEATER_PIN = 5;

// Other customizable constants
const unsigned int LOW_WATER_LVL = 5;
const unsigned int NO_WATER_LVL = 6;
const unsigned int WATERING_TIME = 5000;
const unsigned int WATERING_PAUSE = 120000;
const unsigned long MEASUREMENT_DELAY = 15000; // Delay between temp/humidity measures
const double MIN_HUMIDITY = 50.0;

// PID variables and constants
const double KP = 1.0;
const double KI = 1.0;
const double KD = 1.0;
const double TEMP_TARGET = 0;
double input, output, setpoint;

// Other global variables
bool no_water = false;
unsigned long last_measurement = 0;
double humidity, temperature;
bool winter_mode;
String status_message;
unsigned long rgb_backlight;

// Library objects
SerLCD lcd; // Default I2C address 0x72
DFRobot_SHT20 sht20; // Default I2C address is 0x40
NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, 100);
PID myPID(&input, &output, &setpoint, KP, KI, KD, DIRECT);


void setup() {
  
  Serial.begin(9600);
  Wire.begin();

  // SHT20 - Init
  Serial.println("Initializing SHT20");
  sht20.initSHT20();
  Serial.println("SHT20 initialized");

  // Set pin modes
  pinMode(WINTER_MODE_BTN, INPUT_PULLUP);
  pinMode(PUMP_HEATER_PIN, OUTPUT);

  // Set summer or winter mode
  winter_mode = digitalRead(WINTER_MODE_BTN) == LOW;
  Serial.print("Winter mode: ");
  Serial.println(winter_mode);

  // LCD - Init
  Serial.println("Initializing LCD");
  lcd.begin(Wire); //Set up the LCD for I2C communication
  lcd.setFastBacklight(rgb_backlight);
  lcd.clear(); //Clear the display - this moves the cursor to home position as well
  lcd.print("  Robo-Bonsai!");
  lcd.setCursor(0, 1);
  lcd.print("   MODE ");
  if (winter_mode) {
    lcd.print("HIVER");
    lcd.setFastBacklight(0x99ccff); // Blue
  } else {
    lcd.print("ETE");
    lcd.setFastBacklight(0xccffcc); // Green
  }
  Serial.println("LCD initialized");

  // Activate PID if in winter mode
  if (winter_mode) {
    input = temperature;
    setpoint = TEMP_TARGET;
    myPID.SetMode(AUTOMATIC);
  }

  Serial.println("Setup completed");
}


void loop() {

  // Do nothing until enough time has passed since the last measurement
  while ((millis() - last_measurement) < MEASUREMENT_DELAY) { delay(1); }

  if (winter_mode) {
    heatSoil();
  } else {
    waterPlants();
  }

  displayLCD();
}


// Check temperature and humidity, then display on screen
void checkTempHum() {

  // Use sht20 to read humidity and temperature, log the measurement time
  temperature = sht20.readTemperature();
  humidity = sht20.readHumidity();
  last_measurement = millis();

  // Serial debug
  Serial.print("Temp: ");
  Serial.println(temperature);
  Serial.print("Humi: ");
  Serial.println(humidity);
}


// Check if there is enough water and print negative result on screen
bool enoughWater() {

  // If no_water was set to true on a previous cycle, return false
  if (no_water) { return false; }

  // Ping the distance to the water
  int sonar_ping = sonar.ping_cm();

  // Serial debug
  Serial.print("Sonar distance: ");
  Serial.print(sonar_ping);
  Serial.println(" cm");

  // If the distance to the water is large, there is no more water
  if (sonar_ping > NO_WATER_LVL) {

    // Serial debug
    Serial.println("No water alert");

    // LCD variables
    rgb_backlight = 0xff0000; // Red
    status_message = "  EAU EPUISEE";

    // Set no_water to true for future cycles
    no_water = true;
    
    return false;
    
  // If the distance to the water is medium, water is low
  } else if (sonar_ping > LOW_WATER_LVL) {

    // Serial debug
    Serial.println("Low water alert");

    // LCD variables
    rgb_backlight = 0xffff00; // Yellow
    status_message = "   EAU BASSE";
    
    return true;

  // If the sonar returns 0, there is a reading error
  } else if (sonar_ping == 0) {

    // Serial debug
    Serial.println("Sonar error alert");

    // LCD variables
    rgb_backlight = 0xff00ff; // Purple
    status_message = "     ERREUR";
    
    return false;

  // If the distance to the water is low, there is enough water
  } else {
    
    // Serial debug
    Serial.println("Enough water");

    // LCD variables
    rgb_backlight = 0x000000; // Black
    status_message = "     EAU OK";
    
    return true;
  }
}


// Start the pump for WATERING_TIME duration
void pumpWater() {
  
  digitalWrite(PUMP_HEATER_PIN, HIGH);
  Serial.println("Pump on");
  delay(WATERING_TIME);
  digitalWrite(PUMP_HEATER_PIN, LOW);
  Serial.println("Pump off");
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
  lcd.print("% Hum.");

  // Display status message
  lcd.setCursor(0, 1);
  lcd.print(status_message);
}


// Check humidity and water plants if needed and if there is enough water
void waterPlants() {
  checkTempHum();

  // The pump will be activated many times until MIN_HUMIDITY is reached
  while (enoughWater() && humidity < MIN_HUMIDITY) {
    Serial.println("Watering loop started");
    pumpWater();
    delay(WATERING_PAUSE);
    checkTempHum();
  }
}


// Check the temperature, compute PID output and PWM the PID output to the heating pad
void heatSoil() {

  // The temperature is the PID input, the PID output is the PWM value (0-255)
  checkTempHum();
  input = temperature;
  myPID.Compute();
  analogWrite(PUMP_HEATER_PIN, output);
  
  // Variables for LCD
  unsigned int heat_percent = map(output, 0, 255, 0, 100);
  String heat_percent_string = String(heat_percent);
  status_message = String(" CHAUFFAGE " + heat_percent_string + "%");

  // Serial debug
  Serial.print("Heat PID output: ");
  Serial.println(output);
  Serial.print("Heat %: ");
  Serial.println(heat_percent);
}
