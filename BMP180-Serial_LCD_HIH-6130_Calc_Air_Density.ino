/* SFE_BMP180 library example sketch

This sketch shows how to use the SFE_BMP180 library to read the
Bosch BMP180 barometric pressure sensor.
SparkFun Barometric Pressure Sensor Breakout - BMP180
https://www.sparkfun.com/products/11824
BMP180 Barometric Pressure Sensor Hookup, per
https://learn.sparkfun.com/tutorials/bmp180-barometric-pressure-sensor-hookup-?_ga=1.258478538.1752775786.1425858930

Like most pressure sensors, the BMP180 measures absolute pressure.
This is the actual ambient pressure seen by the device, which will
vary with both altitude and weather.

Before taking a pressure reading you must take a temparture reading.
This is done with startTemperature() and getTemperature().
The result is in degrees C.

Once you have a temperature reading, you can take a pressure reading.
This is done with startPressure() and getPressure().
The result is in millibar (mb) aka hectopascals (hPa).

If you'll be monitoring weather patterns, you will probably want to
remove the effects of altitude. This will produce readings that can
be compared to the published pressure readings from other locations.
To do this, use the sealevel() function. You will need to provide
the known altitude at which the pressure was measured.

If you want to measure altitude, you will need to know the pressure
at a baseline altitude. This can be average sealevel pressure, or
a previous pressure reading at your altitude, in which case
subsequent altitude readings will be + or - the initial baseline.
This is done with the altitude() function.

BMP180 Hardware connections: SDA  SCL   -     +
Uno, Redboard, Pro:          A4   A5   GND  3.3V
Mega2560, Due:               20   21   GND  3.3V
Leonardo:                     2    3   GND  3.3V

(WARNING: do not connect + to 5V or the sensor will be damaged!)

You will also need to connect the I2C pins (SCL and SDA) to your
Arduino. The pins are different on different Arduinos:

Leave the IO (VDDIO) pin unconnected. This pin is for connecting
the BMP180 to systems with lower logic levels such as 1.8V

The SFE_BMP180 library uses floating-point equations developed by the
Weather Station Data Logger project: http://wmrx00.sourceforge.net/

V10 Mike Grusin, SparkFun Electronics 10/24/2013

combined/modified Al Hegemann 03/08/2015:

Starting with an Arduino Uno - R3 (~$25)
https://www.sparkfun.com/products/11021

Honeywell HumidIconTM Digital Humidity/Temperature Sensors (HIH6130/6131 and HIH6120/6121 Series)
http://playground.arduino.cc//Main/HoneywellHumidIconTMDigitalHumidity-TemperatureSensors
SparkFun Humidity and Temperature Sensor Breakout - HIH6130 (~$30)
http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Weather/HIH6130_Breakout_v10.pdf
https://www.sparkfun.com/products/11295

SparkFun Serial Enabled LCD Kit - LCD-10097 (~$25)
https://www.sparkfun.com/products/10097
Serial LCD Kit Quickstart Guide 
https://www.sparkfun.com/tutorials/289
Serial LCD Hardware connections: 5V (Red)  GND (Black)   RX
Arduino                          5V        GND           D3
*/

// You must install this "library".
#include <SFE_BMP180.h>
// Wire is a standard library included with Arduino 1.x and on
#include <Wire.h>
/*
https://code.google.com/p/arduino-hih61xx/downloads/detail?name=HIH-0.2.tar.gz&can=2&q=
with the following note: (in need of being edited from the repository!)
*/
#include <HIH61XX.h>
//  Create an HIH61XX with I2C address 0x27, powered by pin 8
//HIH61XX hih(0x27, 8);
HIH61XX hih(0x27);

// You will need to create an SFE_BMP180 object, here called "pressure":
SFE_BMP180 pressure;

#define ALTITUDE 1793.14  // Altitude of STEM High School, Highlands Ranch, CO. in meters

#include <SoftwareSerial.h>
SoftwareSerial lcd(2, 3);  // This is required, to start an instance of an LCD
void setup()
{
  double T;
  double P;
  lcd.begin(9600);  // Start the LCD at 9600 baud
  clearDisplay();  // Clear the display
  setLCDCursor(5);  // Set cursor to the 5th spot, 1st line
  lcd.print("Hello");
  setLCDCursor(16);  // Set cursor to the 1st spot, 2nd line
  lcd.print("STEM HIGH School");
  
  //
  Serial.begin(9600);
  Wire.begin();
  //
  
  // Flash the backlight:
  for (int i=0; i<3; i++)
  {
    setBacklight(0);
    delay(250);
    setBacklight(255);
    delay(250);
  }
  //
  // Initialize the sensor (it is important to get calibration values stored on the device).

  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.
  }
}

void loop()
{
  //
    while(Serial.available())  // If serial data is available do this
    lcd.print((char) Serial.read());
  //
  char status;
  double T,P,p0,a;

  // Loop here getting pressure readings every 10 seconds.

  // If you want sea-level-compensated pressure, as used in weather reports,
  // you will need to know the altitude at which your measurements are taken.
  // We're using a constant called ALTITUDE in this sketch:
  
  Serial.println();
  Serial.print("Provided local altitude: ");
  Serial.print(ALTITUDE,0);
  Serial.print(" meters, ");
  Serial.print(ALTITUDE*3.28084,0);
  Serial.println(" feet");
  
  //  start the sensor
  hih.start();

  //  request an update of the humidity
  hih.update();

  Serial.print("Humidity: ");
  Serial.print((hih.humidity())*100, 1);
  Serial.println(" % RH");

  clearDisplay();  // Clear the display
  setLCDCursor(6);  // set cursor to 6th spot, 1st row
  lcd.print (hih.humidity()*100,1);
  setLCDCursor(16);  // set the cursor to the 1st spot, 2nd row
  lcd.print("Humidity - % RH");
  delay(2000);  // Pause for 2 seconds.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("Temperature: ");
      Serial.print(T,1);
      Serial.print(" deg C, ");
      Serial.print((9.0/5.0)*T+32.0,1);
      Serial.println(" deg F");
  
      clearDisplay();  // Clear the display
      setLCDCursor(6);  // set cursor to 6nd spot, 1st row
      lcd.print(T,1);
          
      setLCDCursor(18);  // set the cursor to the 5th spot, 2nd row
      lcd.print("Temp - deg C");
      delay(2000);  // Pause for 2 seconds.
      //
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("Station (absolute) pressure: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          clearDisplay();  // Clear the display
          setLCDCursor(5);  // set cursor to 2nd spot, 1st row
          lcd.print(P,2);
          
          setLCDCursor(16);  // set the cursor to the 1st spot, 2nd row
          lcd.print("Station Pres-mb");
          delay(2000);  // Pause for 2 seconds.
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1793.14 meters (Highlands Ranch, CO) - change/update as needed!
          Serial.print("Altimeter Setting / Relative (sea-level) pressure: ");
          Serial.print(p0,2);
          Serial.print(" millibars, ");
          //
          clearDisplay();  // Clear the display
          setLCDCursor(5);  // set cursor to 2nd spot, 1st row
          lcd.print(p0,2);
          setLCDCursor(16);  // set the cursor to the 1st spot, 2nd row
          lcd.print("Alt Setting - mb");
          delay(2000);  // Pause for 2 seconds.
          //
          Serial.print(p0*0.0295333727,2);
          Serial.println(" inHg");
//double Air_Density;
          Serial.print (Air_Density(T,P));
          Serial.println (" kg/m3");

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          //a = pressure.altitude(P,p0);
          //Serial.print("Computed altitude: ");
          //Serial.print(a,0);
          //Serial.print(" meters, ");
          //Serial.print(a*3.28084,0);
          //Serial.println(" feet");
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

  delay(1000);  // Pause for 1 seconds.
}
void setBacklight(byte brightness)
{
  lcd.write(0x80);  // send the backlight command
  lcd.write(brightness);  // send the brightness value
}

void clearDisplay()
{
  lcd.write(0xFE);  // send the special command
  lcd.write(0x01);  // send the clear screen command
}

void setLCDCursor(byte cursor_position)
{
  lcd.write(0xFE);  // send the special command
  lcd.write(0x80);  // send the set cursor command
  lcd.write(cursor_position);  // send the cursor position
}

double Air_Density (double (T), double(P)){
  // Temperature (T) Degrees in Celsius, Pressure (P) in millibars 
  double result;
  double D; 
  double Tk = T + 273.15; // deg K (Tk)= deg C + 273.15
  double eso=6.1078;
  double p; 
  double c0 =  0.99999683;
  double c1 = -0.90826951*10-2;
  double c2 =  0.78736169*10-4;
  double c3 = -0.61117958*10-6;
  double c4 =  0.43884187*10-8;
  double c5 = -0.29883885*10-10;
  double c6 =  0.21874425*10-12;
  double c7 = -0.17892321*10-14;
  double c8 =  0.11112018*10-16;
  double c9 = -0.30994571*10-19;
  double Pv;
  double RH;
  double Rd;
  double Es;
  
  //Air Density Calculations:
  //http://wahiduddin.net/calc/density_altitude.htm 
  
  /*To begin to understand the calculation of air density,  consider the ideal gas law:
  
  (1)      P*V = n*Rg*T
    where:  P = pressure
               V = volume
               n = number of moles
               Rg = universal gas constant
               T = temperature
    
  Density is simply the mass of the molecules of the ideal gas in a certain volume, which may be mathematically expressed as:
  
  (2)      D = m / V
    where:  D = density
               m =  mass
               V = volume
  
  Note that:
              m = n * M
              where: m = mass 
                        n = number of moles
                        M = molar mass
                  
   And define a specific gas constant for the gas under consideration:
  
              R = Rg / M
             where R = specific gas constant
                       Rg = universal gas constant
                       M = molar mass            
  
  Then, by combining the previous equations, the expression for the density becomes:
  
  (3)      D =P/(R*T)
    where:   D = density, kg/m3 
                P = pressure, Pascals ( multiply mb by 100 to get Pascals)
                R = specific gas constant , J/(kg*degK) = 287.05 for dry air
                T = temperature, deg K = deg C + 273.15
  
   As an example, using the ISA standard sea level conditions of P = 101325 Pa and T = 15 deg C,
   the air density at sea level, may be calculated as:
  
              D = (101325) / (287.05 * (15 + 273.15)) = 1.2250 kg/m3 
  
  This example has been derived for the dry air of the standard conditions. However, for real-world situations,
  it is necessary to understand how the density is affected by the moisture in the air.
  
  Neglecting the small errors due to non-ideal gas compressibility and vapor pressure measurements not made over liquid water, 
  the density of a mixture of dry air molecules and water vapor molecules may be simply written as:
  
  (4b)         D=(P/Rd*Tk)*(1-(0.378*Pv/P)
    where:  D = density, kg/m3 
              Pd = pressure of dry air (partial pressure), Pascals
              Pv= pressure of water vapor (partial pressure), Pascals
              P = Pd + Pv = total air pressure, Pascals (multiply mb by 100 to get Pascals)
              Rd = gas constant for dry air, J/(kg*degK) = 287.05 = R/Md
              Rv = gas constant for water vapor, J/(kg*degK) = 461.495 = R/Mv
              R = universal gas constant = 8314.32 (in 1976 Standard Atmosphere)
              Md = molecular weight of dry air = 28.964 gm/mol
              Mv = molecular weight of water vapor = 18.016 gm/mol
              Tk = temperature, deg K = deg C + 273.15
  
  To use equation 4b to determine the density of the air, one must know the actual air pressure 
  (which is also called absolute pressure, total air pressure, or station pressure), the water vapor pressure, and the temperature.
  
  Vapor Pressure:
  In order to calculate water vapor pressure, we need to first calculate the saturation vapor pressure. 
  
  A very accurate, albeit quite odd looking, formula for determining the saturation vapor pressure is a 
  polynomial developed by Herman Wobus :
  
  (Saturation vapor Pressure - Es)
  (5)      Es = eso/p^8
  
           where:  Es = saturation pressure of water vapor, mb
                      eso=6.1078
                      p = (c0+T*(c1+T*(c2+T*(c3+T*(c4+T*(c5+T*(c6+T*(c7+T*(c8+T*(c9)))))))))) 
                      T = temperature, deg C
                      c0 = 0.99999683
                      c1 = -0.90826951*10-2
                      c2 = 0.78736169*10-4
                      c3 = -0.61117958*10-6
                      c4 = 0.43884187*10-8
                      c5 = -0.29883885*10-10
                      c6 = 0.21874425*10-12
                      c7 = -0.17892321*10-14
                      c8 = 0.11112018*10-16
                      c9 = -0.30994571*10-19
    
  Actual Vapor Pressure (Pv) from Relative Humidity:
  
  Relative humidity is defined as the ratio (expressed as a percentage) of the actual vapor pressure to the 
  saturation vapor pressure at a given temperature. 
  
  To find the actual vapor pressure, simply multiply the saturation vapor pressure by the percentage and the 
  result is the actual vapor pressure. For example, if the relative humidity is 40% and the temperature is 30 deg C, 
  then  the saturation vapor pressure is 42.43 mb and the actual vapor pressure is 40% of 42.43 mb, which is 16.97 mb.
  
       (7b)      Pv = RH * Es
  
        where  Pv= pressure of water vapor (partial pressure) 
                  RH = relative humidity (expressed as a decimal value)
                  Es = saturation vapor pressure ( multiply mb by 100 to get Pascals)
    
  Some examples of calculations using air density:
  
  Example 1) The lift of an aircraft wing may be described mathematically (see ref 8) as:
    
  L = c1 * d * v^2/2 * a
  
  where:    L = lift
                c1 = lift coefficient
                 d = air density
                 v = velocity
                 a = wing area
  
  From the lift equation, we see that the lift of a wing is directly proportional to the air density. So if a certain wing can lift, 
  for example, 3000 pounds at sea level standard conditions where the density is 1.2250 kg/m3, then how much can the wing lift on a 
  warm summer day in Denver when the air temperature is 95 deg (35 deg C), the actual pressure is 24.45 in-Hg (828 mb) and the dew point 
  is 67 deg F (19.4 deg C)? The answer is about 2268 pounds.
    
  */
      p = (c0+T*(c1+T*(c2+T*(c3+T*(c4+T*(c5+T*(c6+T*(c7+T*(c8+T*(c9))))))))));
      Es = eso/(pow(p,8));
      Pv = RH * Es;
      D=(P/Rd*Tk)*(1-(0.378*Pv/P));
  /*
  
  >>>>>>>>>>>>>>>>
  Absolute Pressure is the actual air pressure, not corrected for altitude, and is also called the station pressure.
  
  Air Density is the mass per unit volume of the air. 
  For example, the standard air density at sea level is 1.225 kg/m3 (0.076474 pounds/ft3).
  
  The metric unit hPa (hectoPascal) is identical to the pressure unit called mb (milliBar).
  */
return result;
}
