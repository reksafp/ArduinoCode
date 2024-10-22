// Libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_VEML6075.h>
#include <ArduinoJson.h>
#include <hp_BH1750.h>
#include <VL6180X.h>

// Precipitation pinout
#define dropletPower 7
#define dropletPin 8

// Dependencies
Adafruit_BME280 bme;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 0);
Adafruit_VEML6075 vml = Adafruit_VEML6075();
hp_BH1750 bhl;
VL6180X North;
VL6180X East;

// Variables
const float msl = 1013.25;
const float hpa = 100.0;
bool bmeStatus, vmlStatus, tslStatus;
bool ledState = true;
uint16_t luxTSL;
const byte North_address = 0x30;
const byte East_address = 0x31;

// Wind Direction and Wind Speed Pin
const int North_enable = 16;
const int East_enable = 17;

// VL6180X sensor constant
const float Radius = 36;           // inside pipe radius (mm)
const float North_offset = -13.0;  // distance (mm) from centre-line
const float East_offset = -13.0;   // distance (mm) from centre-line
const float Scale = 4.0;           // scale factor
float North_reading = 0.0;
float East_reading = 0.0;
float Windspeed = 0.0;
float Direction = 0.0;

// ----- Filter0 (North Sensor)
float Array0[40] = { 0 };  // array
int Index0 = 0;            // position of first item
float Sum0 = 0.0;          // array sum
float Average0 = 0.0;      // moving average

// ----- Filter1 (East Sensor)
float Array1[40] = { 0 };  // array
int Index1 = 0;            // position of first item
float Sum1 = 0.0;          // array sum
float Average1 = 0.0;      // moving average

// ----- Filter2 (Direction)
float Array2[40] = { 0 };  // array
int Index2 = 0;            // position of first item
float Sum2 = 0.0;          // array sum
float Average2 = 0.0;      // moving average

// ----- Communications
const long Baudrate = 115200;  // serial comminication speed
char Char;

// Rain Gauge Tipping Bucket Constant
const int pin_interrupt = 14;
long int jumlah_tip = 0;
long int temp_jumlah_tip = 0;
float curah_hujan = 0.00;
float milimeter_per_tip = 0.70;

volatile boolean flag = false;

void ICACHE_RAM_ATTR hitung_curah_hujan() {
  flag = true;
}

// UV Sensor, Temperature, and Lux Constant
double temp, hum;
uint8_t uv;
uint16_t luxBH;

// void blinky() {
//   digitalWrite(LED_BUILTIN, ledState);
//   ledState = !ledState;
// }

float readTemp() {
  float volt = ((analogRead(4) * 3.3F) / 65536.0F);
  float temp = 27 - ((volt - 0.172) / 0.02721);
  return temp;
}

double round2(double value) {
  return (int)(value * 100 + 0.5) / 100.0;
}

double genFloat(double min, double max) {
  long temp = random(min * 100, max * 100);
  double val = (double)temp / 100.0F;
  return round2(val);
}

long calcLux(long luxFromTSL, long luxFromBH) {
  long calcLux;
  long diff = (long)luxFromTSL - luxFromBH;
  diff = abs(diff);

  //sanity check whether both values differ by more than 3000
  //TODO: implement percentage difference (e.g. more than 30% difference) to tackle low value compare
  if (diff > 3000)  //something is off, check which is abnormal
  {
    //TODO: have more sophisticated checks
    if (luxFromTSL == 0)
      return luxFromBH;
    if (luxFromBH == 0)
      return luxFromTSL;
  }

  calcLux = (long)(luxFromTSL + luxFromBH) / 2;
  return calcLux;
}

// // Capacitive Rain Gauge Function
// int capacitiveRain() {
//   digitalWrite(dropletPower, HIGH);
//   delay(10);
//   int droplet = digitalWrite(dropletPin);
//   digitalWrite(dropletPower, LOW);
//   return droplet;
// }

// Wind Speed Calculation
float calculate_speed_direction(float north, float east) {

  // ----- locals
  float N = north;   // north reading (adjusted)
  float E = east;    // east reading (adjusted)
  float C;           // chord formed by laser readings
  float R = Radius;  // pipe radius
  float A1;          // angle between radius and chord
  float A2;          // angle between north and chord
  float A3;          // angle between east and chord;
  float A4;          // diference angle between north and radius;
  float A5;          // difference angle between east and radius;
  float A6;          // arc-tangent dY/dX
  float dX;          // horizontal offset
  float dY;          // vertical offset
  float D;           // wind direction

  // ----- calculations
  C = sqrt(sq(N) + sq(E));
  A1 = acos(C / (2 * R)) * RAD_TO_DEG;
  A2 = acos(N / C) * RAD_TO_DEG;
  A3 = 90 - A2;
  A4 = A1 - A2;
  A5 = A1 - A3;
  dY = R * fabs(cos(A4 * DEG_TO_RAD)) - N;
  dX = R * fabs(cos(A5 * DEG_TO_RAD)) - E;
  if (fabs(dX) == 0) dX = 0.0001;
  A6 = fabs(atan(dY / dX) * RAD_TO_DEG);

  // ----- Apply signs to dY and dX
  if (fabs(N - R * cos(A4 * DEG_TO_RAD) - dY) > 0.001) dY = -dY;
  if (fabs(E - R * cos(A5 * DEG_TO_RAD) - dX) > 0.001) dX = -dX;

  // ----- Calculate wind direction
  if ((dX >= 0) && (dY >= 0)) Direction = 270 - A6;  // quadrant 0
  if ((dX < 0) && (dY >= 0)) Direction = 90 + A6;    // quadrant 1
  if ((dX < 0) && (dY < 0)) Direction = 90 - A6;     // quadrant 2
  if ((dX >= 0) && (dY < 0)) Direction = 270 + A6;   // quadrant 3

  // ----- calculate the windspeed
  Windspeed = sqrt(sq(dX) + sq(dY)) * Scale;
}

float moving_average(float *array_name, int array_length, float *array_sum, int *array_index, float new_value) {
  *array_sum = *array_sum - array_name[*array_index] + new_value;  // subtract earliest value; add latest value
  array_name[*array_index] = new_value;                            // store latest value in vacant position
  (*array_index)++;                                                // point to next earliest value
  if (*array_index >= array_length) {
    *array_index = 0;  // wrap around
  }
  return *array_sum / array_length;  // moving average
}

void softwareReset(uint16_t ms) {
  Serial.print("Rebooting pico in");
  Serial.print(ms);
  Serial.println(" ms");
  delay(ms);
  ESP.restart();
}

void setup() {
  Serial.begin(115200);

  // Rain Gauge Tipping Bucket Configuration Pins
  pinMode(pin_interrupt, INPUT);

  // Wind Configuration pins
  pinMode(North_enable, OUTPUT);
  pinMode(East_enable, OUTPUT);

  Wire.begin();

  bmeStatus = bme.begin(0x76, &Wire);
  tslStatus = tsl.begin(&Wire);
  vmlStatus = vml.begin(VEML6075_400MS, false, false, &Wire);
  tsl.enableAutoRange(true);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);

  bhl.begin(BH1750_TO_GROUND);  // set address and initialize the sensor
  bhl.calibrateTiming();        // calibrate the timings, about 855ms with a bad chip
  bhl.start();                  // start measurement first time in setup

  // Shutdown Rain Gauge
  digitalWrite(dropletPower, LOW);

  // Interrupt Pin for Tipping Bucket if logic change from HIGH to LOW
  attachInterrupt(digitalPinToInterrupt(pin_interrupt), hitung_curah_hujan, FALLING);

  // Shutdown Wind Sensor
  digitalWrite(North_enable, LOW);
  digitalWrite(East_enable, LOW);

  // ----- Configure North_facing sensor
  digitalWrite(North_enable, HIGH);  // activate sensor
  delay(50);
  North.init();  // configure sensor
  North.configureDefault();
  North.setAddress(North_address);  // overide the default address
  North.setTimeout(500);

  //North.writeReg(VL6180X::SYSRANGE__PART_TO_PART_RANGE_OFFSET,0);   // use this offset when calibrating
  North.writeReg(VL6180X::SYSRANGE__PART_TO_PART_RANGE_OFFSET, 9);
  North.stopContinuous();
  delay(300);
  North.startRangeContinuous(50);

  // ------ Configure East_facing sensor
  digitalWrite(East_enable, HIGH);  // activate sensor
  delay(50);
  East.init();  // configure sensor
  East.configureDefault();
  East.setAddress(East_address);  // change default address
  East.setTimeout(500);

  //East.writeReg(VL6180X::SYSRANGE__PART_TO_PART_RANGE_OFFSET,0);
  East.writeReg(VL6180X::SYSRANGE__PART_TO_PART_RANGE_OFFSET, 22);
  East.stopContinuous();
  delay(300);
  East.startRangeContinuous(50);
}

void loop() {
  StaticJsonDocument<600> doc;

  //TSL-2651 readout

  sensors_event_t reading;
  tsl.getEvent(&reading);
  luxTSL = reading.light;

  //TSL-2561 saturation function

  if (luxTSL == 0)  // wait some time until data is ready
  {
    delay(100);
    tsl.getEvent(&reading);
    luxTSL = reading.light;
  }

  if (luxTSL > 40000)  // saturation function, TSL2561 can measure up to 40k Lux
    luxTSL = 40000;

  //BH-1750 readout

  //TODO: Implement small waiting cycle up to 50ms if sensor has no value

  if (bhl.hasValue()) {
    luxBH = (uint16_t)bhl.getLux();
    bhl.start();
  }

  // blinky();

  temp = bme.readTemperature();  //read BME280 temp
  hum = bme.readHumidity();      //read BME280 humidity
  uv = vml.readUVI();            //read VEML UV Index

  if (Serial.available() > 0) {
    if (!North.timeoutOccurred()) {
      North_reading = (float)North.readRangeContinuousMillimeters() - North_offset;  // Membaca jarak kontinu dalam mm, mengalikan dgn skala apabila kontinu mode aktif
      Average0 = moving_average(Array0, sizeof(Array0) / sizeof(Array0[0]), &Sum0, &Index0, North_reading);
    }

    if (!East.timeoutOccurred()) {
      East_reading = (float)East.readRangeContinuousMillimeters() - East_offset;
      Average1 = moving_average(Array1, sizeof(Array1) / sizeof(Array1[0]), &Sum1, &Index1, East_reading);
    }

    calculate_speed_direction(Average0, Average1);

    Average2 = moving_average(Array2, sizeof(Array2) / sizeof(Array2[0]), &Sum2, &Index2, Direction);


    // ----- send results to the display
    // Serial.print(Windspeed);
    // Serial.print(',');
    // Serial.print(Average2);               // Direction
    // Serial.print(',');

    /*
        To use these values:
         - Set the North_offset and East_offset in the Arduino header to zero
         - Replace the above offset values with these readings
         - The windspeed indicator should resemble a pin-prick
      */
    // Serial.print(Average0 - Radius);      // Required North_offset in still air
    // Serial.print(',');
    // Serial.println(Average1 - Radius);    // Required East_offset in still air
  }

  // capacitiveRain();

    if (flag == true)  // don't really need the == true but makes intent clear for new users
    {
      curah_hujan += milimeter_per_tip;  // Akan bertambah nilainya saat tip penuh
      jumlah_tip++;
      delay(500);
      flag = false;  // reset flag
    }
    curah_hujan = jumlah_tip * milimeter_per_tip;
    if ((jumlah_tip != temp_jumlah_tip))  // Print serial setiap 1 menit atau ketika jumlah_tip berubah
    {
      // printSerial();
    }
    temp_jumlah_tip = jumlah_tip;
  
  doc["cpu_temperature"] = 0;                   //lattepanda CPU temp
  doc["rtc_temperature"] = round2(readTemp());  //actually, pico CPU temp
  doc["external_temperature"] = round2(temp);
  doc["humidity"] = round2(hum);
  doc["air_pressure"] = round2(bme.readPressure() / hpa);
  doc["altitude"] = trunc(bme.readAltitude(msl));  //standard MSL pressure
  doc["visible_light"] = calcLux(luxTSL, luxBH);   //vis_light calculated
  doc["uv"] = trunc(abs(uv));                      //integer UV index, for float use format(abs(vml.readUVI()))
  doc["wind_direction"]       = trunc(genFloat(90, 134));       //still untested
  doc["wind_velocity"]        = genFloat(0, 2);                 //still untested
  doc["wind_direction"] = Average2;                  //still untested
  doc["wind_velocity"] = Windspeed;                  //still untested
  doc["latitude"] = -6.59;                           //static lat, no GPS
  doc["longitude"] = 105.81;                         //static lon, no GPS
  // doc["precipitation"] = calcPrecip(temp, hum, uv);  //still untested
  doc["precipitation"] = curah_hujan;  //still untested
  // doc["vbus"] = genFloat(4.89, 5.16);                //no frontend to read VBUS
  // doc["vbattery"] = genFloat(12.2, 13.1);            //no frontend to read battery
  // doc["vsolar"] = genFloat(8.1, 16.66);              //solar panel not used
  // doc["rssi"] = random(-35, -6);                     //lorawan not implemented

  serializeJson(doc, Serial);
  Serial.println(" ");

  delay(1700);
  //delay(25000);
}