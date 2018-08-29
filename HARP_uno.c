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

// Declare sensors
Adafruit_BMP085 bmp;
MS5803 sensor(ADDRESS_HIGH); // ADDRESS_HIGH = 0x76 (for ADDRESS_LOW = 0x77)
Adafruit_BNO055 bno = Adafruit_BNO055(55);

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
  // MS5803-14BA
  sensor.reset();
  sensor.begin();
  pressure_baseline = sensor.getPressure(ADC_4096);

  // BMP180
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}}
  
  // BNO055
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
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
  Serial.println("MS5803-14BA -------------------------------");
  Serial.print("Temperature C = ");
  Serial.println(temperature_c);
  
  Serial.print("Temperature F = ");
  Serial.println(temperature_f);
  
  Serial.print("Pressure abs (mbar)= ");
  Serial.println(pressure_abs);
   
  Serial.print("Pressure relative (mbar)= ");
  Serial.println(pressure_relative); 
  
  //Serial.print("Altitude change (m) = ");
  //Serial.println(altitude_delta); 

  Serial.println();

  // BMP180 ---------------------------------------------------
  Serial.println("BMP180 -------------");
  Serial.print("Temperature = ");
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");
    
  Serial.print("Pressure = ");
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");
    
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print("Altitude = ");
  Serial.print(bmp.readAltitude());
  Serial.println(" meters");

  Serial.print("Pressure at sealevel (calculated) = ");
  Serial.print(bmp.readSealevelPressure());
  Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
  Serial.print("Real altitude = ");
  Serial.print(bmp.readAltitude(101500));
  Serial.println(" meters");
    
  Serial.println();
  
  // BNO055 ------------------------------------------------
  Serial.println("BNO055 -------------------------);
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");

  delay(1000);
}
