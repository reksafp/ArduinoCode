![Screenshot 2024-10-22 180311](https://github.com/user-attachments/assets/127114bb-f88b-4e97-b20a-d2c7416aa124)

Basically, weather station will measure environmental condition with various parameters using sensors. Here is the combined code that contain various type of sensors. 
Communication protocol that used is Serial I2C. After data gathered, it will be stored to postgresql databases and shown in NodeRed Dashboard UI.

Arduino Code for Weather Station which contain:
- Adafruit BME280 (Temperature, Humidity)
  library: https://docs.arduino.cc/libraries/adafruit-bme280-library/
- Adafruit TSL2561 (Lux)
  library: https://www.arduinolibraries.info/libraries/adafruit-tsl2561
- Adafruit VEML6075 (UV Index)
  library: https://github.com/adafruit/Adafruit_VEML6075
- Adafruit VL6180x (TOF Distance)
  library: https://github.com/pololu/vl6180x-arduino/blob/master/VL6180X.h
- Rain Gauge custom (Precipitation)
