# Bonsai/Plant watering system
Code for an arduino-based plant watering system.
## Hardware
- [Sparkfun Pro Micro 3.3V](https://www.sparkfun.com/products/12587)
- [Water pump](https://www.robotshop.com/ca/en/immersible-water-pump-water-tube.html)
- [HC-SR04 Ultrasonic range finder](https://www.robotshop.com/ca/en/hc-sr04-ultrasonic-range-finder-osepp.html)
- [SparkFun 16x2 SerLCD - Black on RGB 3.3V](https://www.sparkfun.com/products/14072)
- [I2C Soil moisture sensor](https://www.tindie.com/products/miceuz/i2c-soil-moisture-sensor/)
## How it works
There is an auto mode and a manual mode for this system, selected by a sustaining button or switch.

**Manual Mode:** The pump is activated continuously as long as there is water available.

**Auto Mode:** The system will periodically check the humidity of the soil and the water level. If the humidity falls below a certain level and if there is still water available, it will active the pump for a few seconds, then wait a few minutes before checking the humidity and water level again. It does this continuously.

## Notes
- The water level is indicated in percent with both modes and the LCD screens turns yellow and red when there is low and no water respectively.
- The ultrasonic range finder is located a few cm above the water, pointing the top of the water. It pings the distance with the top of the water. If the distance is high, it means that the water level is low.
- The code can be customized to your system by simply changing the *customizable constants* at the top.
