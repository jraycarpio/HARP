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


// Setup file for datalogging
File myFile;
int pinCS = 10; // Pin 10 on Arduino Uno


double t; // time t



// _____________________MS5803-14BA_______________________________
MS5803 sensor(ADDRESS_HIGH); 

float temperature_c;
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





// _____________________ADXL377____________________________

int scale = 200; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377
boolean micro_is_5V = true;

// _____________________end ADXL377____________________________

int n = 1; // test file var



// _________________deployment logic___________________
double cpress; // current pressure
float amag; // acceleration magnitude 
float amag0; // initial acceleration magnitude
float amag_lift; // acceleration that signifies that rocket has launched
int inc_press; // counter to confirm that pressure is increasing
boolean initacc; // initial acceleration, did rocket turn on
boolean liftoff; // rocket has launched

void setup() {


  t = 0; // set time at 0 
  cpress = sensor.getPressure(ADC_4096);;
  inc_press = 0;
  initacc = false;
  liftoff = false;
  pinMode(pinCS, OUTPUT);

  SD.begin();
  myFile = SD.open("harpdata.txt", FILE_WRITE);
  myFile.println("----------------- HARP_uno_v8 -----------------");
  myFile.println();
  myFile.println("Time [sec]\tTemperature [C]\tPressure [mbar]\tX [g]\tY [g]\tZ [g]\tAccl. Magnitude [g]\tApo (Accl)\tApo (Press)");
  myFile.println();
  myFile.close();
 
  
  // MS5803-14BA
  sensor.reset();
  sensor.begin();
  pressure_baseline = sensor.getPressure(ADC_4096);

  
  delay(1000);
    
}
void loop() {
  
  // initialize SD card
  myFile = SD.open("harpdata.txt", FILE_WRITE);

  // Print time
  myFile.print(t);
  myFile.print("\t\t");
  
  /*______________ MS5803-14BA ________________*/

  temperature_c = sensor.getTemperature(CELSIUS, ADC_512);
  pressure_abs = sensor.getPressure(ADC_4096);

  // print data on file
  myFile.print(temperature_c);

  myFile.print("\t\t");
  
  myFile.print(pressure_abs);
  


  /* ____________ end MS5803-14BA _____________*/
 
  myFile.print("\t\t");




  //------------------------ADXL377-------------------------------------
  //Serial.println("ADXL377 -----------------------------");
  
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

  // Print to file
  myFile.print(scaledX); 
  myFile.print("\t"); 
  myFile.print(scaledY); 
  myFile.print("\t"); 
  myFile.print(scaledZ); 
  myFile.print("\t");
  
  t = t + 0.5;


  
  amag = sqrt((scaledX*scaledX) + (scaledY*scaledY) + (scaledZ*scaledZ));
  myFile.print(amag);
  myFile.print("\t");
  if (initacc == false) {
    initacc = true;
    amag0 = amag;
    amag_lift = amag0 + 3; // determine if rocket has launched
    myFile.print("0\t0"); // print 0 0 for apo accl and apo press
  } 
  // Rocket has been initiated
  else {
    if (amag >= amag_lift && liftoff == false) {
      liftoff = true;
    }

    // If rocket has launched, detect apogee
    if (liftoff == true) {
      
      // Detect apogee based on acceleration
      if (amag <= amag0) {
        myFile.print("1\t"); // Print 1 for apogee detection by accl.
      } else {
        myFile.print("0\t"); // Print 0 for no apogee detection by accl
      }

      // Detect apogee based on pressure
      if (pressure_abs < cpress) {
        cpress = pressure_abs;
        myFile.print("0\t"); // Print 0 for no apogee detection by press
        // pressure is decreasing, altitude increasing
      } else {
        // pressure is now increasing, once it increases 6 increments, apogee has been 
        inc_press = inc_press + 1;
        
        if (inc_press == 5) {
          myFile.print("1\t"); // Print 1 for apogee detection by press
          }
        }
    } else {
      myFile.print("0\t0\t"); // No apogee detection yet, rocket is on but has not liftoff yet
    }
  }

  

  myFile.println(); // new data line
  
  myFile.close();
  delay(500);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
