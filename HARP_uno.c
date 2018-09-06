/*
 * 
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
 * 
 */
 
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <SparkFun_MS5803_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// Setup file for datalogging
File myFile;
int pinCS = 10; // Pin 10 on Arduino Uno



// _____________________MS5803-14BA_______________________________
MS5803 sensor(ADDRESS_HIGH); 

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

// ____________________end MS5803-14BA__________________________


// _______________________BNO055_______________________________

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// ______________________end BNO055_______________________________


// _____________________ADXL377____________________________

int scale = 200; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377
boolean micro_is_5V = true;

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// _____________________end ADXL377____________________________

void setup() {
    
  Serial.begin(9600);
  pinMode(pinCS, OUTPUT);
  
  // SD Card Initialization
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }
  
  // Create/Open file 
  myFile = SD.open("harpdata.txt", FILE_WRITE);
  
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.println("Writing to file...");
  }
  // if the file didn't open, print an error:
  else {
    Serial.println("error opening file");
    while(1);
  }
  // Reading the file
  myFile = SD.open("harpdata.txt");
  if (myFile) {
    Serial.println("Read:");
    // Reading the whole file
    while (myFile.available()) {
      Serial.write(myFile.read());
   }
    myFile.close();
  }
  else {
    Serial.println("error opening test.txt");
  }
  
  // MS5803-14BA
  sensor.reset();
  sensor.begin();
  pressure_baseline = sensor.getPressure(ADC_4096);

  // BNO055
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
  // empty
  myFile = SD.open("harpdata.txt", FILE_WRITE);

  /*______________ MS5803-14BA ________________*/

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
  
  // print data
  myFile.println("MS5803-14BA -------------------------------");
  myFile.print("Temperature C = ");
  myFile.println(temperature_c);
  //Serial.println(temperature_f);
  
  myFile.print("Temperature F = ");
  myFile.println(temperature_f);
  
  myFile.print("Pressure abs (mbar)= ");
  myFile.println(pressure_abs);
   
  myFile.print("Pressure relative (mbar)= ");
  myFile.println(pressure_relative); 
  

  myFile.println();


  /* ____________ end MS5803-14BA _____________*/
 
  myFile.println();

  /* _____________________ BNO055 ____________________*/
  myFile.println("BNO055 -------------------------");
  sensors_event_t event; 
  bno.getEvent(&event);
  
  myFile.print("X: ");
  myFile.print(event.orientation.x, 4);
  myFile.print("\tY: ");
  myFile.print(event.orientation.y, 4);
  myFile.print("\tZ: ");
  myFile.print(event.orientation.z, 4);
  myFile.println("");
  /* _____________________end BNO055 ____________________*/


  /* 
   *  
   *  commented this section out, still need to change all 
   *  "hdata" to "myFile" and condense where possible
   *  
   *  as of right now with BNO055 and MS5803-14BA, still experiencing
   *  some issues. 75% of dynamic memory is occupied. 62% of program
   *  storage space is occupied.
   *  
   *  
   *  
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
  hdata.println("RAW");
  hdata.print("X: "); hdata.println(rawX);
  hdata.print("Y: "); hdata.println(rawY);
  hdata.print("Z: "); hdata.println(rawZ);
  hdata.println();
  
  // Print out scaled X,Y,Z accelerometer readings
  hdata.println("Scaled measurements");
  hdata.print("X: "); hdata.print(scaledX); hdata.println(" g");
  hdata.print("Y: "); hdata.print(scaledY); hdata.println(" g");
  hdata.print("Z: "); hdata.print(scaledZ); hdata.println(" g");
  hdata.println();

  */
  
  delay(3500); // this should be delay(200), but it is 3500 for sake of testing
}
