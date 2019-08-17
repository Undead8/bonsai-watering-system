#include <Wire.h>
#include <DFRobot_SHT20.h> // https://github.com/DFRobot/DFRobot_SHT20
#include <NewPing.h> // https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home

// Pin constants
const unsigned int PUMP_HEATER_PIN = 1;
const unsigned int SONAR_TRIGGER_PIN = 2;
const unsigned int SONAR_ECHO_PIN = 3;

// Other constants and variables
const unsigned int LOW_WATER_LVL = 5;
const unsigned int NO_WATER_LVL = 6;
bool no_water = false;
const unsigned int WATERING_TIME = 5000;
const unsigned int WATERING_PAUSE = 120000;
const unsigned int MEASUREMENT_DELAY = 600000; // Delay between temp/humidity measures
unsigned long last_measurement;
const float MIN_HUMIDITY = 50.0;
float humidity;

DFRobot_SHT20 sht20;
NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, 100);

void setup() {
  Serial.begin(9600);
  
  // SHT20 - Init then serial.print status
  sht20.initSHT20();
  delay(100);
  sht20.checkSHT20();
  checkHumidity();
}

void loop() {
  
  unsigned long time_since = millis() - last_measurement;
  if (time_since > MEASUREMENT_DELAY) {
    waterPlants();
  } else {
    // DISPLAY time_since / 60000 on the screen
  }

}

// Check humidity and display on screen
float checkHumidity()
{
  humidity = sht20.readHumidity();
  last_measurement = millis();
  // DISPLAY humidity ON SCREEN
  return humidity;
}

// Check if there is enough water and print negative result on screen
bool enoughWater()
{
  if(no_water) { return false; }
  
  int sonar_ping = sonar.ping_cm();
  if (sonar_ping >= NO_WATER_LVL)
  {
    // PRINT ON SCREEN NO WATER
    no_water = true;
    return false;
  } else if (sonar_ping >= LOW_WATER_LVL) {
    // PRINT ON SCREEN LOW WATER
    return true;
  } else {
    return true;
  }
}

// Start the pump for ms
void pumpWater()
{
  digitalWrite(PUMP_HEATER_PIN, HIGH);
  delay(WATERING_TIME);
  digitalWrite(PUMP_HEATER_PIN, LOW);
}

// Check humidity and water plants
void waterPlants()
{
  checkHumidity();
  
  while (enoughWater() && humidity < MIN_HUMIDITY) {
    pumpWater();
    delay(WATERING_PAUSE);
    checkHumidity();
  }
}
