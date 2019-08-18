#include <Wire.h>
#include <SerLCD.h> // http://librarymanager/All#SparkFun_SerLCD
#include <DFRobot_SHT20.h> // https://github.com/DFRobot/DFRobot_SHT20 Default I2C address is 0x40
#include <NewPing.h> // https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home

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
const float MIN_HUMIDITY = 50.0;

// Global variables
bool no_water = false;
unsigned long last_measurement;
int humidity;
int temperature;
bool winter_mode;

SerLCD lcd; // Initialize the library with default I2C address 0x72
DFRobot_SHT20 sht20;
NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, 100);


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

  // Delay to make sure everything is init, then check temp and humidity.
  delay(100);
  checkTempHum();
}


void loop() {

  unsigned long time_since = millis() - last_measurement;

  if (winter_mode) {
    // WINTER MODE HERE
  } else if (time_since > MEASUREMENT_DELAY) {
    waterPlants();
  }
}


// Check temperature and humidity, then display on screen
void checkTempHum() {
  temperature = static_cast<int>(sht20.readTemperature());
  humidity = static_cast<int>(sht20.readHumidity());
  last_measurement = millis();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("Hum: ");
  lcd.print(humidity);
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


// Start the pump for ms
void pumpWater() {
  digitalWrite(PUMP_HEATER_PIN, HIGH);
  delay(WATERING_TIME);
  digitalWrite(PUMP_HEATER_PIN, LOW);
}


// Check humidity and water plants, unless there is not enough water
void waterPlants() {
  checkTempHum();
  
  while (enoughWater() && humidity < MIN_HUMIDITY) {
    pumpWater();
    delay(WATERING_PAUSE);
    checkTempHum();
  }
}
