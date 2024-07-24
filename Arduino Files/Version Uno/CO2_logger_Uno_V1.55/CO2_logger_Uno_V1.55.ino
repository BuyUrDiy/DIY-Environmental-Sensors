// Arduino UNO CO2 and pH Logger program 
// Verstion 1.55
// Made 7/2023 by A.Rok for MacroGas project
// This is a rough code that does the job. Anyone who wants is more than welcome to try and make a more efficient or cleaner code.
// This is a modification of the 'blank' generic logger files
// This is desgined for use with an Arduino Uno and the Adafruit Data logger shield
// CO2 sensor is from Sandbox electronics, PH is from DF Robot

// Code is modified heavily form the SleapyPi project

//----------------------------------------------------------------------------------------------------------------------------------------------------------


// ---------------------------------------------------------------Libraries -----------------------------------------------------------------
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "SleepyPi2.h"   // make sure to include the Spell Foundry PCF8523 library as well
#include <TimeLib.h>  // include time by Michael Margolis
#include <LowPower.h> // include lowpwer by LowPowerLab
# include <NDIR_I2C.h> // For Sandbox CO2 Sensor - can be found on sandbox electronics git hub if lost
#include <SC16IS750.h> // For Sandbox CO2 Sensor



// ------------------------------------------------ Timing Interval and Power Pins. CHANGE THIS-----------------------------------------------------
// .. Setup the Periodic Timer
// .. use either eTB_SECOND or eTB_MINUTE or eTB_HOUR
eTIMER_TIMEBASE  PeriodicTimer_Timebase     = eTB_MINUTE;   // e.g. Timebase set to seconds. Other options: eTB_MINUTE, eTB_HOUR, eTB_SECOND
uint8_t          PeriodicTimer_Value        = 15;           // Timer Interval in units of Timebase e.g 10 seconds






const int InterruptPin = 2;  // Amost arduino - DO NOT CHANGE if using UNO
int powerpin_pH = 8; // pin that powers the pH sensor on and off - needs to be digital pin









// ------------------------------------------------------------------- Variables----------------------------------------------------------

// Variable names for pH measurements
float rawpH; // raw measurement coming from the sensor
float voltage_pH; // create variable for voltage on pH meter
float sensorValue_pH; // create varaible for raw sensor data on pH meter
float sensorValue_CO2;

int buf1[10]; // temporary storage of 10 samples taken for pH
int buf2[10]; // temporary storage of 10 samples taken for CO2

unsigned long int avgValue_pH;  //Store the average value of the sensor feedback
unsigned long int avgValue_CO2;  //Store the average value of the sensor feedback


tmElements_t tm;  // used in support functions SleepyPi

File logfile; // placeholder name for what you are writing your data too - DO NOT CHANGE
int chipSelect = 10; // pin of SD card reader - DO NOT CHANGE



//------ Some code for I2C addresses and Time information
NDIR_I2C mySensor(0x4D); //Adaptor's I2C address (7-bit, default: 0x4D) - infot used by Sandbox CO2, don't change
SC16IS750 i2cuart = SC16IS750(SC16IS750_PROTOCOL_I2C,SC16IS750_ADDRESS_BB); // some code for establishing the I2C protocol for CO2 sensors

// Helps with SleepyPI time functions
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

// legacy code from SleepyPI; unclear if needed
void alarm_isr()
{
    // Just a handler for the alarm interrupt.
    // You could do something here

}

//-------------------------------------------------------------------- Setup Code----------------------------------------------------------
void setup()
{ 
    // initialize serial communication: In Arduino IDE use "Serial Monitor"
  Serial.begin(115200); // begin serial communictation on Baud rate 115200
  i2cuart.begin(9600);   // begin I2C communication
  Serial.println("Starting");
  delay(50);
  SleepyPi.rtcInit(true);

// code to check for SD card reader communication
 while (!Serial) {
     ; // wait for serial port to connect. Needed for native USB port only
   }
  Serial.print("Initializing SD card..."); // this will print out on serial port if sd card is reading
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

;
// Code to initilize the CO2 Sensor communication
if (mySensor.begin()) {
        Serial.println("Wait 10 seconds for sensor initialization...");
        delay(10000);
    } else {
        Serial.println("ERROR: Failed to connect to the sensor.");
        while(1);
    }


   // Default the clock to the time this was compiled.
   // Comment out if the clock is set by other means
   // ...get the date and time the compiler was run
    if (getDate(__DATE__) && getTime(__TIME__)) {
  //     // and configure the RTC with this info
       SleepyPi.setTime(DateTime(F(__DATE__), F(__TIME__)));
   } 
  
  printTimeNow();   

pinMode(InterruptPin, INPUT_PULLUP); // establish the interrupt pin for RTC wake cycles

  Serial.print("Periodic Interval Set for: ");
  Serial.print(PeriodicTimer_Value);
  switch(PeriodicTimer_Timebase)
  {
    case eTB_SECOND:
      Serial.println(" seconds");
      break;
    case eTB_MINUTE:
      Serial.println(" minutes");
      break;
    case eTB_HOUR:
      Serial.println(" hours");
    default:
        Serial.println(" unknown timebase");
        break;
  }

// Open and print to logfile. This should print out headers for the data collected below.
// This is not necessary, just here for convience 
logfile = SD.open("data.txt", FILE_WRITE); // change the .txt file name here if desired, but try and keep the name short. Seems to work better than way.
  if(logfile){
  logfile.print("Date");
  logfile.print(";");
  logfile.print("Time");
  logfile.print(';');
  logfile.print("Rough pH");
  logfile.print(';');
  logfile.print("Raw pH Voltage");
  logfile.print(";");
  logfile.print("Raw ppm CO2 val 1");
  logfile.print(";");
  logfile.print("Raw ppm CO2 val 2");
  logfile.print(";");
  logfile.println("Raw ppm CO2 val 3");
  logfile.close();
  }

//mySensor.disableAutoCalibration();  // turn off auto-calibration of the CO2 sensor. Not a usfull feature with the power cycles
delay(100);
power(0); // power off the CO2 sensor before sleep loop
delay(100);

}

// ------------------------------------------------------ Logger Loop code------------------------------------

void loop() 
{
    SleepyPi.rtcClearInterrupts();

    // Allow wake up alarm to trigger interrupt on falling edge.
    attachInterrupt(digitalPinToInterrupt(InterruptPin), alarm_isr, FALLING);    // Alarm pin

    // Set the Periodic Timer
    SleepyPi.setTimer1(PeriodicTimer_Timebase, PeriodicTimer_Value);

    delay(500);

    // Enter power down state with ADC and BOD module disabled.
    // Wake up when wake up pin is low.
    SleepyPi.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 

    // Disable external pin interrupt on wake up pin.
    detachInterrupt(0);
    
    SleepyPi.ackTimer1();

// ------------------------------ Sensor Code: Powers on sensors and records thier data --------------------------------------------------

DateTime now;  // get a date and time
pinMode(powerpin_pH,OUTPUT);  // power pin as output for PH
digitalWrite(powerpin_pH, HIGH);  // turn on power to pH sensor assuming pwr on pin 8
power(1); // turn on power to CO2 sensor
delay(100);
mySensor.begin();  // begin co2 sensor
delay (12000); // wait 12 seconds for pH sensor and CO2 sensor to power up and 'Warm up';

// Take your sample
for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf1[i]=analogRead(A0); // Read and store 10 measurments from pH sensor
    mySensor.measure(); // Read and store 10 measurement of CO2 sensor
    buf2[i] = mySensor.ppm; // store each measurment to the buffer code. 
    delay(500); // wait .5 sec between each measurment. Overall sampling interaval takes 5 seconds here
  }

avgValue_pH=0;
//avgValue_CO2=0;
  for(int i=0;i<10;i++)
  {                     
    avgValue_pH+=buf1[i]; // add up each of the 10 samples taken by pH
    //avgValue_CO2+=buf2[i];
  }


voltage_pH = avgValue_pH*5.0/1024/10; // convert raw data to voltage data and average the pH measurements
sensorValue_pH = 3.5*voltage_pH;  // convert voltage data to rough pH values that need calibrated
//sensorValue_CO2 = avgValue_CO2/10; // average the CO2 measurements

now = SleepyPi.readTime(); // take time reading from RTC

mySensor.measure(); // take one quick real time CO2 measurement before logging data. Used as a check of what is going on with the sensor

// Write data to SD card:
  logfile = SD.open("data.txt", FILE_WRITE); 
  if(logfile){
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(";");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print(';');
  logfile.print(sensorValue_pH,3);
  logfile.print(';');
  logfile.print(voltage_pH,3);
  logfile.print(";");
  //logfile.println(sensorValue_CO2,3);
  logfile.print(mySensor.ppm); // Last recorded CO2 measurement
  logfile.print(";");
  logfile.print(buf2[2]); // 2nd recorded co2 measuremnt
  logfile.print(";");
  logfile.println(buf2[8]); // 8th recorded co2 measurment (appears to be the most accurate)

  logfile.close();
  
  }
// Echo data to serial port
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(";");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print(';');
  Serial.print(sensorValue_pH,3);
  Serial.print(';');
  Serial.print(voltage_pH,3);
  Serial.print(";");
  //Serial.println(sensorValue_CO2);
   Serial.print(mySensor.ppm);
   Serial.print(";");
   Serial.print(buf2[2]);
   Serial.print(";");
   Serial.println(buf2[8]);



delay(1000); // wait one second to ensure that SD card file is closed. Overall sample time is 18 seconds - 12 sec pwr up, 5 sec sample, 1 sec wait
digitalWrite(powerpin_pH, LOW);  // turn off power to pH sensor
power(0); // turn off power to CO2 sensor

    // Just a handler for the pin interrupt.
    Serial.println("Woken Up");
    delay(50);
}











//**********************************************************************

 //- Helper routines

//**********************************************************************
void printTimeNow()
{
    // Read the time
    DateTime now = SleepyPi.readTime();
    
    // Print out the time
    Serial.print("Ok, Time = ");
    print2digits(now.hour());
    Serial.write(':');
    print2digits(now.minute());
    Serial.write(':');
    print2digits(now.second());
    Serial.print(", Date (D/M/Y) = ");
    Serial.print(now.day());
    Serial.write('/');
    Serial.print(now.month()); 
    Serial.write('/');
    Serial.print(now.year(), DEC);
    Serial.println();

    return;
}
bool getTime(const char *str)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

bool getDate(const char *str)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.write('0');
  }
  Serial.print(number);
}

void power (uint8_t state) {
    i2cuart.pinMode(0, INPUT);      //set up for the power control pin
 
    if (state) {
        i2cuart.pinMode(0, INPUT);  //turn on the power of MH-Z16
    } else {
        i2cuart.pinMode(0, OUTPUT);
        i2cuart.digitalWrite(0, 0); //turn off the power of MH-Z16
    }
}