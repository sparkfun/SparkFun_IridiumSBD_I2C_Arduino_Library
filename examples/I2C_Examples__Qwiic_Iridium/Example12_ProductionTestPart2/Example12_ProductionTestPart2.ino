#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C
#include <time.h>
#include <Wire.h> //Needed for I2C communication

/*
 * Production Test Part 2
 * 
 * LED_BUILTIN will flash at 0.5Hz (1sec on, 1sec off) if the test succeeds.
 * If the LED is off, the test is either in progress or has failed.
 *
 * This sketch demonstrates how to retrieve the Iridium system time
 * from the modem using the getSystemTime method. This uses
 * the Iridium command AT-MSSTM to acquire the time.  The method will
 * fail if the Iridium network has not yet been acquired.
 *
 * Assumptions
 *
 * The sketch assumes the SparkFun Qwiic Iridium 9603N is connected via I2C.
 */

#define IridiumWire Wire
#define DIAGNOSTICS false // Change this to see diagnostics

// Declare the IridiumSBD object using default I2C address
IridiumSBD modem(IridiumWire);

// Globals to hold the current and previous satellite time
struct tm old_t; // struct tm is defined in time.h
struct tm new_t; // struct tm is defined in time.h
time_t old_secs, new_secs;

void setup()
{
  int err;

  // Latest epoch began at May 11, 2014, at 14:23:55 UTC.
  old_t.tm_year = 2014 - 1900;
  old_t.tm_mon = 5 - 1;
  old_t.tm_mday = 11;
  old_t.tm_hour = 14;
  old_t.tm_min = 23;
  old_t.tm_sec = 55;
  old_secs = mktime(&old_t); // Convert to seconds

  new_t.tm_year = 2014 - 1900;
  new_t.tm_mon = 5 - 1;
  new_t.tm_mday = 11;
  new_t.tm_hour = 14;
  new_t.tm_min = 23;
  new_t.tm_sec = 55;
  new_secs = mktime(&new_t); // Convert to seconds

  // Start the console serial port
  Serial.begin(115200);
  //while (!Serial); // Wait for the user to open the serial monitor
  Serial.println(F("Qwiic Iridium 9603N - Production Test Part 2"));

  //empty the serial buffer
  while(Serial.available() > 0) Serial.read();

  //wait for the user to press any key before beginning
  //Serial.println(F("Press any key to start example."));
  //while(Serial.available() == 0);

  //clean up
  while(Serial.available() > 0) Serial.read();

  // Disable the LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Start the I2C wire port connected to the satellite modem
  Wire.begin();
  Wire.setClock(400000); //Set I2C clock speed to 400kHz

  // Check that the Qwiic Iridium is attached
  if (!modem.isConnected())
  {
    Serial.println(F("Qwiic Iridium is not connected! Please check wiring. Freezing."));
    while(1);
  }

  // Enable the supercapacitor charger
  Serial.println(F("Enabling the supercapacitor charger..."));
  modem.enableSuperCapCharger(true);

  // Wait for the supercapacitor charger PGOOD signal to go high
  while (!modem.checkSuperCapCharger())
  {
    Serial.println(F("Waiting for supercapacitors to charge..."));
    delay(1000);
  }
  Serial.println(F("Supercapacitors charged!"));

  // Enable power for the 9603N
  Serial.println(F("Enabling 9603N power..."));
  modem.enable9603Npower(true);

  // Begin satellite modem operation
  Serial.println(F("Starting modem..."));
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE); // Assume 'USB' power (slow recharge)
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("Begin failed: error "));
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println(F("No modem detected: check wiring."));
    digitalWrite(LED_BUILTIN, LOW); // Disable the LED
    while(1); // Do nothing more
  }
}

void loop()
{
   int err = modem.getSystemTime(new_t); // Ask the 9603N for the system time
   if (err == ISBD_SUCCESS) // Was it successful?
   {
      old_secs = mktime(&old_t); // Convert to seconds
      new_secs = mktime(&new_t);

      if (new_secs > old_secs)
      {
        digitalWrite(LED_BUILTIN, (new_secs % 2)); // Flash the LED at 0.5Hz

        char buf[32];
        sprintf(buf, "%d-%02d-%02d %02d:%02d:%02d",
           new_t.tm_year + 1900, new_t.tm_mon + 1, new_t.tm_mday, new_t.tm_hour, new_t.tm_min, new_t.tm_sec);
        Serial.print(F("Iridium date/time is "));
        Serial.println(buf);
      }

      old_t = new_t; // Update the old time
   }

   else if (err == ISBD_NO_NETWORK) // Did it fail because the 9603N has not yet seen the network?
   {
      Serial.println(F("No network detected."));
      digitalWrite(LED_BUILTIN, LOW);
   }

   else
   {
      Serial.print(F("Unexpected error "));
      Serial.println(err);
      digitalWrite(LED_BUILTIN, LOW);
   }

   // Delay
   delay(100UL);
}

#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
#endif
