/*
 * HARP UNO AVIONICS
 * SENSOR SUITE
 * 
 * Programmed by:
 * Jonathan Carpio, Jenil Thakker
 * 
 * High Altitude Rocketry Program
 * San Jose State University
 * Aerospace Engineering 
 * Computer Science
 */

// Include required libraries
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>

// Setup file for datalogging
File hdata; // HARP data object
int pinCS = 10; // CS pin, pin 10 on arduino


// Declare sensors
Adafruit_BMP085 bmp;
MS5803 sensor(ADDRESS_HIGH); // ADDRESS_HIGH = 0x76 (for ADDRESS_LOW = 0x77)
Adafruit_BNO055 bno = Adafruit_BNO055(55);


// ADXL377 sensor init
int scale = 200; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377
boolean micro_is_5V = true;


// ______________For MS5803-14BA: create variables to store results_________________________
float temperature_c, temperature_f;
double pressure_abs, pressure_relative, altitude_delta, pressure_baseline;

// Create Variable to store altitude in (m) for calculations;
double base_altitude = 1655.0; // Altitude of SparkFun's HQ in Boulder, CO. in (m)

// *****************This may not be necesary 
double sealevel(double P, double A)
// Given a pressure P (mbar) taken at a specific altitude (meters),
// return the equivalent pressure (mbar) at sea level.
// This produces pressure readings that can be used for weather measurements.
{
  return(P/pow(1-(A/44330.0),5.255));
}


double altitude(double P, double P0)
// Given a pressure measurement P (mbar) and the pressure at a baseline P0 (mbar),
// return altitude (meters) above baseline.
{
  return(44330.0*(1-pow(P/P0,1/5.255)));
}

// _______________________end MS5803-14BA_____________________________



void setup() {
  
  Serial.begin(9600);
  
  /************* SD Card initialization ************/
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  }
  else
  {
    Serial.println("SD card initialization failed.");
    return;
  }
  
  /************ End SD card initialization ************/
  
  // MS5803-14BA
  sensor.reset();
  sensor.begin();
  pressure_baseline = sensor.getPressure(ADC_4096);

  // BMP180
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}}
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
    

}

void loop() {

  // Create/open file
  hdata = SD.open("harpdata.txt", FILE_WRITE);
  
  // Check if file opened correctly
  if (hdata) {
    Serial.println("harpdata.txt created..");
    hdata.println("__________ HARP DATA [today's date & time] _________");
  }
  else {
    Serial.println("failed to create harpdata.txt");
    while(1);
  }

  // Read temperature from the sensor in deg C. This operation takes about 
  temperature_c = sensor.getTemperature(CELSIUS, ADC_512);
  
  // Read temperature from the sensor in deg F. Converting
  // to Fahrenheit is not internal to the sensor.
  // Additional math is done to convert a Celsius reading.
  temperature_f = sensor.getTemperature(FAHRENHEIT, ADC_512);
  
  // Read pressure from the sensor in mbar.
  pressure_abs = sensor.getPressure(ADC_4096);
  
  // Let's do something interesting with our data.
  
  // Convert abs pressure with the help of altitude into relative pressure
  // This is used in Weather stations.
  pressure_relative = sealevel(pressure_abs, base_altitude);
  
  // Taking our baseline pressure at the beginning we can find an approximate
  // change in altitude based on the differences in pressure.   
  altitude_delta = altitude(pressure_abs , pressure_baseline);
  
  // MS5803-14BA --------------------------------------------
  hdata.println("MS5803-14BA -------------------------------");
  hdata.print("Temperature C = ");
  hdata.println(temperature_c);
  
  hdata.print("Temperature F = ");
  hdata.println(temperature_f);
  
  hdata.print("Pressure abs (mbar)= ");
  hdata.println(pressure_abs);
   
  hdata.print("Pressure relative (mbar)= ");
  hdata.println(pressure_relative); 
  
  //Serial.print("Altitude change (m) = ");
  //Serial.println(altitude_delta); 

  hdata.println();

  // BMP180 ---------------------------------------------------
  hdata.println("BMP180 -------------");
  hdata.print("Temperature = ");
  hdata.print(bmp.readTemperature());
  hdata.println(" *C");
    
  hdata.print("Pressure = ");
  hdata.print(bmp.readPressure());
  hdata.println(" Pa");
    
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  hdata.print("Altitude = ");
  hdata.print(bmp.readAltitude());
  hdata.println(" meters");

  hdata.print("Pressure at sealevel (calculated) = ");
  hdata.print(bmp.readSealevelPressure());
  hdata.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
  hdata.print("Real altitude = ");
  hdata.print(bmp.readAltitude(101500));
  hdata.println(" meters");
    
  hdata.println();
  
  // BNO055 ------------------------------------------------
  hdata.println("BNO055 -------------------------");
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  hdata.print("X: ");
  hdata.print(event.orientation.x, 4);
  hdata.print("\tY: ");
  hdata.print(event.orientation.y, 4);
  hdata.print("\tZ: ");
  hdata.print(event.orientation.z, 4);
  hdata.println("");
  
  //------------------------ADXL377-------------------------------------
  hdata.println("ADXL377 -----------------------------");
  
  // Get raw accelerometer data for each axis
  int rawX = analogRead(A0);
  int rawY = analogRead(A1);
  int rawZ = analogRead(A2);
  
  // Scale accelerometer ADC readings into common units
  // Scale map depends on if using a 5V or 3.3V microcontroller
  float scaledX, scaledY, scaledZ; // Scaled values for each axis
  if (micro_is_5V) // Microcontroller runs off 5V
  {
    scaledX = mapf(rawX, 0, 675, -scale, scale); // 3.3/5 * 1023 =~ 675
    scaledY = mapf(rawY, 0, 675, -scale, scale);
    scaledZ = mapf(rawZ, 0, 675, -scale, scale);
  }
  else // Microcontroller runs off 3.3V
  {
    scaledX = mapf(rawX, 0, 1023, -scale, scale);
    scaledY = mapf(rawY, 0, 1023, -scale, scale);
    scaledZ = mapf(rawZ, 0, 1023, -scale, scale);
  }
  
  // Print out raw X,Y,Z accelerometer readings
  hdata.print("X: "); hdata.println(rawX);
  hdata.print("Y: "); hdata.println(rawY);
  hdata.print("Z: "); hdata.println(rawZ);
  hdata.println();
  
  // Print out scaled X,Y,Z accelerometer readings
  hdata.print("X: "); hdata.print(scaledX); hdata.println(" g");
  hdata.print("Y: "); hdata.print(scaledY); hdata.println(" g");
  hdata.print("Z: "); hdata.print(scaledZ); hdata.println(" g");
  hdata.println();
  
  delay(200); // Minimum delay of 2 milliseconds between sensor reads (500 Hz)

  hdata.close();
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
