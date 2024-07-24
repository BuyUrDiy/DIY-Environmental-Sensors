// Arduino Pro Mini data logger / sensor code.  Version 1 made on 2/1/2024 by A.Rok
// This code is totally interchangable with the Nano data logger setup as well. It would also work with an Uno if you use a DS2321 RTC module.

// This code is a blank template for use with a Pro Mini data logger (and the Nano version too). 
// This code will put the microprocessor into deep sleep; wake it up via interrupt pin;
// Take a reading from what ever sensors have been attatched; write that data to a text file; and then return to low power sleep.

// This is for use with the Arduino Pro Mini (Or Nano) and DS3231 RTC. 

// Feel free to improve, modify, or change any code; I'm sure there is a better way (all code and hardware is open source). 

// YOU WILL need to add in additional code to reflect the sensors you are using.

//---------------------------------------------------------------------------------------------------------------------------------------------------

// LIBRARIES
#include <SPI.h>  
#include <SD.h>
#include "RTClib.h" // RTC library
#include <avr/sleep.h> // Sleep library

// Add aditional libraries needed for your specific snesors here

#define interruptPin 2 //This is the digital pin that the RTC squarewave interrupt will be sent to. Change if needed


//------------------------------------------------------ VARIABLES ---------------------------------------------------------------------------------

const int time_interval = 1;// THIS SETS THE SAMPLING INTERVAL IN MINUTES. Change as needed.

// Included any temporary variable you will need for storage of your sensors measurment values
// For example for temperature you may want to defint an int Temp as a place to store tempreature values

int XXXX; // place holder varaible; CHANGE TO MATCH YOUR SENSORS

// Arduino nano data logger additional parameters
RTC_DS3231 rtc; //for real time clock
File mydata; // name holder for what you are writing your data too
int chipSelect = 10; // chip select pin that is wired to your sd card reader. Change if using different pin
#define DS3231_ADDRESS     0x68 // define the I2C address of the RTC. Can help if using multiple I2C devices.


//-------------------------------------------------------SETUP--------------------------------------------------------------------------------------

void setup() {
Serial.begin(9600);  //turn on serial port, baud rate is 9600 in this case


  Serial.print("Initializing SD card..."); // Check to see if SD reader is working and communicating.
  if (!SD.begin(10)) {
    Serial.println("initialization failed!"); // if this fails the code loop will stop here. Check for connection issue or power issues with SD reader
    while (1);
  }
  Serial.println("initialization done.");

  if (! rtc.begin()) {                      // Begin communicaiton with RTC. Check to see if it is attatched.
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }
  rtc.disable32K(); // dont need the 32k pin


  //All of the code below is for the RTC system

    if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

//Set alarms for system:
 pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), wakeUp, FALLING);  // sets the interrrupt pin to wake the system up from sleep
    
    // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
    // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);
    
    // stop oscillating signals at SQW Pin
    // otherwise setAlarm1 will fail
    rtc.writeSqwPinMode(DS3231_OFF);
    
    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    rtc.disableAlarm(2);
    
    // schedule an alarm XXX time in the future
    if(!rtc.setAlarm1(
            rtc.now() + TimeSpan(0,0,time_interval,0),      // Time span is (Days, Hours, Minutes, Seconds). Move your time_interval to the appropriate location as needed
            DS3231_A1_Minute // this mode triggers the alarm when the seconds match. See Doxygen for other options
    )) {
        Serial.println("Error, alarm wasn't set!");
    }else {
        Serial.println("Alarm set");  
    }


delay(50);
//------------------------
// If using an I2C sensors; place code for intilizing communcation with it here.

// If using analog sensor place code for assignment of power and communcaiton pins here.
//------------------------
  
Serial.println("Power cycle");// will print when the system cycles power or resets the alarm
  
}

//----------------------------------------------------------------------INITIAL LOOP--------------------------------------------------------------
void loop() {

delay(4000); //wait 4 seconds before going to sleep
Going_to_sleep(); // goes to sleep

}


//--------------------------------------------------------------------SLEEP FUNCTION------------------------------------------------------------
// function for putting sensor to sleep and resetting the alarms

void Going_to_sleep(){
sleep_enable();
set_sleep_mode(SLEEP_MODE_PWR_DOWN);// set for full sleep
attachInterrupt(digitalPinToInterrupt(interruptPin), wakeUp,LOW);//attach an interrupt to pin d2
sleep_cpu();// activating sleep mode

sample_data();// This if the fuction that will power on and record data from your sensors

// clears the alarm
if(rtc.alarmFired(1)) {  
        rtc.clearAlarm(1);
        Serial.println("Alarm cleared");
    }
// reset the alarm
if(!rtc.setAlarm1( 
            rtc.now() + TimeSpan(0,0,time_interval,0), // remember to move your time_interval as needed
            DS3231_A1_Minute // this mode triggers the alarm when the seconds match. See Doxygen for other options
    )) {
        Serial.println("Error, alarm wasn't set!");
    }else {
        Serial.println("Alarm set");  
    }

}

//----------------------------------------------------------------------WAKE UP FUNCTION-------------------------------------------------------------------------
// wakes up the system
void wakeUp(){
  Serial.println("Interrrupt Fired");//Print message to serial monitor
   sleep_disable();//Disable sleep mode
  detachInterrupt(digitalPinToInterrupt(interruptPin)); //Removes the interrupt from pin 2;
 
}

//--------------------------------------------------------------------SAMPLE DATA FUNCTION; CHANGE THIS!!!!--------------------------------------------------------
// takes sample from sensors and prints to the serial port
// you need to edit this code to fit your specific sneosrs
void sample_data(){   
delay(50);
  
// ADD code to power on your sensor systems here

delay(50);

// The delay below is only necessary if you have sensors that need a 'warm up' time before their readings are stable
unsigned long start = millis();
while (millis() - start < 10000); // set this number to be your warm up time in miliseconds


// // ADD IN YOU CODE for sampling from your sensors and recording data to temporary variable



delay(100);
// power off your sensors here
delay(1000); // Delay to give voltage drop from powering off sensors time to occur. This can be much shorter
//------------------------------

write_file(XXXX);//sends data to write file. XXXXX would be your place holder variable such as Temp. CHANGE


DateTime now = rtc.now(); 
Serial.print(now.month(), DEC);  // print the date, time, and measurmenets to the serial port
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print('/');
    Serial.print(now.year(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.print(' ');
   // add aditional things to be printed to serial here; CHANGE
   //Serial.print(Temp) for example

}




//---------------------------------------------------------------------------WRITE FILE FUNCTION-----------------------------------------------------------------
// writes the data to the SD card
// CHANGE to reflect you sensors / recorded values


void write_file(float XXXX){  // change the float variable here to match your other temporary variables (Temp as example); CHANGE

  DateTime now = rtc.now(); 

  mydata = SD.open("datalog.txt", FILE_WRITE);  // opens file to write to on SD card. Will save to the text file called datalog.
  
   // if the file is available, write to it:
  if (mydata) {
    mydata.print(now.month(), DEC);
    mydata.print('/');
    mydata.print(now.day(), DEC);
    mydata.print('/');
    mydata.print(now.year(), DEC);
    mydata.print(';');
    mydata.print(now.hour(), DEC);
    mydata.print(':');
    mydata.print(now.minute(), DEC);
    mydata.print(':');
    mydata.print(now.second(), DEC);
    mydata.println(';');
    // add aditional information you want written here. CHANGE
   // mydata.println(temp); for example
  }
delay(100);

  }


//--------- Add any Aditional functions needed for your sensors or project here----------------------------------------


