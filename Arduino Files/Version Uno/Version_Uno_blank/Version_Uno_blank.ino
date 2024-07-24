// Blank Arduino UNO Logger program
// Made 7/2023 by A.Rok
// This is a template program; It will need to be modified to suit your specific sensors / needs. 
// This is a rough code that does the job. Anyone who wants is more that welcome to try and make a more efficient or cleaner code.



// ---------------------------------------------------------------Libraries -----------------------------------------------------------------
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "SleepyPi2.h"   // make sure to include the Spell Foundry PCF8523 library as well
#include <TimeLib.h>  // include time by Michael Margolis
#include <LowPower.h> // include lowpwer by LowPowerLab



// ------------------------------------------------ Timing Interval and Power Pins. CHANGE THIS-----------------------------------------------------
// .. Setup the Periodic Timer
// .. use either eTB_SECOND or eTB_MINUTE or eTB_HOUR
eTIMER_TIMEBASE  PeriodicTimer_Timebase     = eTB_MINUTE;   // e.g. Timebase set to seconds. Other options: eTB_MINUTE, eTB_HOUR
uint8_t          PeriodicTimer_Value        = 1;           // Timer Interval in units of Timebase e.g 10 seconds














const int InterruptPin = 2;  // Amost arduino - DO NOT CHANGE if using UNO
// Include any additional pins used for powering sensors here


// ------------------------------------------------------------------- Variables----------------------------------------------------------

tmElements_t tm;  // used in support functions SleepyPi

File logfile; // placeholder name for what you are writing your data too
int chipSelect = 10; // pin of SD card reader - DO NOT CHANGE

// Helps with SleepyPI time functions
const char *monthName[12] = {
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

// legacy code from SleepyPI; Pretty sure this is not needed
void alarm_isr()
{
    // Just a handler for the alarm interrupt.
    // You could do something here

}

//-------------------------------------------------------------------- Setup Code----------------------------------------------------------
void setup()
{ 
    // initialize serial communication: In Arduino IDE use "Serial Monitor"
  Serial.begin(9600); // begin serial communictation on Baud rate 115200
  Serial.println("Starting");
  delay(50);
  SleepyPi.rtcInit(true);


// SD card reader setup:
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

// RTC setup:
// Default the clock to the time this was compiled.
// Comment out if the clock is set by other means
 // ...get the date and time the compiler was run
if (getDate(__DATE__) && getTime(__TIME__)) {
//     // and configure the RTC with this info
    SleepyPi.setTime(DateTime(F(__DATE__), F(__TIME__)));
   } 
  
 printTimeNow();   


// Some Interrupt alarm setup:

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
// This is not necessary. Just shows you can do it if desired

logfile = SD.open("data.txt", FILE_WRITE); // change the .txt file name here if desired, but try and keep the name short. Seems to work better than way.
  if(logfile){
  logfile.print("Date");
  logfile.print(";");
  logfile.print("Time");
  logfile.print(';');
  logfile.println("XXXXX");
  logfile.close();
  }

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


// -------Sensor Code: Powers on sensors and records thier data --------------------------------------------------

DateTime now;  // get a date and time


//----------------------------- ADD YOUR SENSOR MEASUREMENT CODE HERE------------------------


now = SleepyPi.readTime(); // take time reading from RTC


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
  logfile.println(now.second(), DEC);
  
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
  Serial.println(';');
 

delay(1000); // wait one second to ensure that SD card file is closed. This can be much less time. 

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