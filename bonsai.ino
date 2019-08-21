#include <Wire.h>
#include <SerLCD.h> // http://librarymanager/All#SparkFun_SerLCD
#include <DFRobot_SHT20.h> // https://github.com/DFRobot/DFRobot_SHT20
#include <NewPing.h> // https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library

// Pin constants
const unsigned int PUMP_HEATER_PIN = 1;
const unsigned int SONAR_TRIGGER_PIN = 2;
const unsigned int SONAR_ECHO_PIN = 3;
const unsigned int WINTER_MODE_BTN = 4;

// Other constants
const unsigned int LOW_WATER_LVL = 5;
const unsigned int NO_WATER_LVL = 6;
const unsigned int WATERING_TIME = 5000;
const unsigned int WATERING_PAUSE = 120000;
const unsigned int MEASUREMENT_DELAY = 600000; // Delay between temp/humidity measures
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
double humidity;
double temperature;
bool winter_mode;


SerLCD lcd; // Default I2C address 0x72
DFRobot_SHT20 sht20; // Default I2C address is 0x40
NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, 100);
PID myPID(&input, &output, &setpoint, KP, KI, KD, DIRECT);


void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // LCD - Init
  lcd.begin(Wire); //Set up the LCD for I2C communication
  lcd.setBacklight(0, 0, 0); //Set backlight to black
  lcd.setContrast(5); //Set contrast. Lower to 0 for higher contrast.
  lcd.clear(); //Clear the display - this moves the cursor to home position as well
  
  // SHT20 - Init 
  sht20.initSHT20();

  // Set summer or winter mode
  pinMode(WINTER_MODE_BTN, INPUT_PULLUP);
  winter_mode = digitalRead(WINTER_MODE_BTN) == LOW;

  // Check and display temperature and humidity
  checkTempHum();

  // Activate PID if in winter mode
  if (winter_mode){
    input = temperature;
    setpoint = TEMP_TARGET;
    myPID.SetMode(AUTOMATIC);
  }
}


void loop() {

  // Do nothing until enough time has passed since the last measurement
  while ((millis() - last_measurement) < MEASUREMENT_DELAY) { delay(1); }

  if (winter_mode) { 
    heatSoil();
  } else {
    waterPlants();
  }
}


// Check temperature and humidity, then display on screen
void checkTempHum() {
  temperature = sht20.readTemperature();
  humidity = sht20.readHumidity();
  last_measurement = millis();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(static_cast<int>(temperature));
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Hum: ");
  lcd.print(static_cast<int>(humidity));
  lcd.print("%");
}


// Check if there is enough water and print negative result on screen
bool enoughWater() {
  if(no_water) { return false; }
  
  int sonar_ping = sonar.ping_cm();
  
  if (sonar_ping >= NO_WATER_LVL) {
    
    // Print on screen no water and change backlight to red
    lcd.setBacklight(255, 51, 0); //Set backlight to red
    lcd.setCursor(11, 0);
    lcd.print("NO");
    lcd.setCursor(10, 1);
    lcd.print("WATER");
    no_water = true;
    return false;
    
  } else if (sonar_ping >= LOW_WATER_LVL) {
    
    // Print on screen low water and change backlight to yellow
    lcd.setBacklight(255, 255, 0); //Set backlight to yellow
    lcd.setCursor(11, 0);
    lcd.print("LOW");
    lcd.setCursor(10, 1);
    lcd.print("WATER");
    return true;
    
  } else {
    return true;
  }
}


// Start the pump for WATERING_TIME duration
void pumpWater() {
  digitalWrite(PUMP_HEATER_PIN, HIGH);
  delay(WATERING_TIME);
  digitalWrite(PUMP_HEATER_PIN, LOW);
}


// Check humidity and water plants if needed and if there is enough water
void waterPlants() {
  checkTempHum();
  
  while (enoughWater() && humidity < MIN_HUMIDITY) {
    pumpWater();
    delay(WATERING_PAUSE);
    checkTempHum();
  }
}


// Check the temperature, compute PID output and PWM the PID output
void heatSoil() {
  checkTempHum();
  input = temperature;
  myPID.Compute();
  analogWrite(PUMP_HEATER_PIN, output);
}
