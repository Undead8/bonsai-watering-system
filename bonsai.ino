#include <Wire.h>
#include <SerLCD.h> // http://librarymanager/All#SparkFun_SerLCD
#include <DFRobot_SHT20.h> // https://github.com/DFRobot/DFRobot_SHT20
#include <I2CSoilMoistureSensor.h> // https://github.com/Apollon77/I2CSoilMoistureSensor
#include <NewPing.h> // https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library

// Pin constants
const unsigned int SONAR_TRIGGER_PIN = 6;
const unsigned int SONAR_ECHO_PIN = 7;
const unsigned int WINTER_MODE_BTN = 4;
const unsigned int PUMP_HEATER_PIN = 5;

// Other customizable constants
const unsigned int FULL_WATER_LVL = 285; // Sonar ping in uS (57 uS = 1 cm) for full water
const unsigned int LOW_WATER_LVL = 627; // Sonar ping in uS (57 uS = 1 cm) for low water warning
const unsigned int NO_WATER_LVL = 741; // Sonar ping in uS (57 uS = 1 cm) for no water
const unsigned int WATERING_TIME = 5000; // Duration in ms that the pump will be active when watering
const unsigned int WATERING_PAUSE = 120000; // Minimum wait time between watering in ms during watering cycle
const unsigned long MEASUREMENT_DELAY = 300000; // Delay in ms between temp/humidity measures
const double MIN_HUMIDITY = 10.0; // Minimum humidity in % that can be reached before watering

// PID variables and constants
const double KP = 1.0;
const double KI = 1.0;
const double KD = 1.0;
const double TEMP_TARGET = 0;
double input, output, setpoint;

// Other global variables
unsigned long last_measurement;
double humidity, temperature;
bool winter_mode;
String status_message;
unsigned long rgb_backlight = 0xffffff; // White

// Library objects
SerLCD lcd; // Default I2C address 0x72
DFRobot_SHT20 sht20; // Default I2C address is 0x40
I2CSoilMoistureSensor sensor; // Default I2C address is 0x20
NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, 100);
PID myPID(&input, &output, &setpoint, KP, KI, KD, DIRECT);


void setup() {
  
  Serial.begin(9600);
  Wire.begin();

  // Set pin modes
  pinMode(WINTER_MODE_BTN, INPUT_PULLUP);
  pinMode(PUMP_HEATER_PIN, OUTPUT);
  digitalWrite(PUMP_HEATER_PIN, LOW);

  // Set summer or winter mode
  winter_mode = digitalRead(WINTER_MODE_BTN) == LOW;
  Serial.print("Winter mode: ");
  Serial.println(winter_mode);

  // LCD - Init
  Serial.println("Initializing LCD");
  lcd.begin(Wire); //Set up the LCD for I2C communication
  delay(1000);
  lcd.setFastBacklight(rgb_backlight);
  lcd.clear(); //Clear the display - this moves the cursor to home position as well
  lcd.print("  ROBO-BONSAI");
  lcd.setCursor(0, 1);
  lcd.print("   MODE ");
  if (winter_mode) {
    lcd.print("HIVER");
    rgb_backlight = 0x99ccff; // Blue
    lcd.setFastBacklight(rgb_backlight);
  } else {
    lcd.print("ETE");
    rgb_backlight = 0xccffcc; // Green
    lcd.setFastBacklight(rgb_backlight);
  }
  Serial.println("LCD initialized");

  // Sensors/PID - Init for winter and summer mode
  if (winter_mode) {

    // Activate PID
    input = temperature;
    setpoint = TEMP_TARGET;
    myPID.SetMode(AUTOMATIC);

    // SHT20 - Init
    Serial.println("Initializing SHT20");
    sht20.initSHT20();
    delay(1000);
    Serial.println("SHT20 initialized");
    
  } else {
    
    // I2CSoilMoisture - Init
    Serial.println("Initializing I2CSoilMoisture");
    sensor.begin();
    delay(1000);
    Serial.println("I2CSoilMoisture initialized");
  }

  // Perform first cycle
  delay(3000);
  if (winter_mode) {
    heatSoil();
  } else {
    waterPlants();
  }
  displayLCD();

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


// Check temperature and humidity
void checkTempHum() {

  if (winter_mode) {
    
    // Use sht20 to measure temperature and humidity
    temperature = sht20.readTemperature(); // In Celsius
    humidity = sht20.readHumidity(); // In percent
    
  } else {
    
    // Use I2CSoilMoisture to measure temperature and humidity
    sensor.getCapacitance();
    while (sensor.isBusy()) delay(50);
    sensor.getTemperature();
    while (sensor.isBusy()) delay(50);
    humidity = sensor.getCapacitance(); // In Capacitance
    temperature = sensor.getTemperature() / 10.0; // In Celsius 
  }
  
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

    // LCD variables
    rgb_backlight = 0xffffff; // White
    
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
  lcd.print(" HUMI");

  // Display status message
  lcd.setCursor(0, 1);
  lcd.print(status_message);
}

// RGB to hex converter
unsigned long createRGB(int r, int g, int b)
{   
    return ((r & 0xff) << 16) + ((g & 0xff) << 8) + (b & 0xff);
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
  unsigned long rgb_output = createRGB(255, 255 - output, 255 - output);
  rgb_backlight = (rgb_output); // From white to red depending of output
  unsigned int heat_percent = map(output, 0, 255, 0, 100);
  status_message = String(" CHAUFFAGE " + String(heat_percent) + "%");

  // Serial debug
  Serial.print("Heat PID output: ");
  Serial.println(output);
  Serial.print("Heat %: ");
  Serial.println(heat_percent);
}
