#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C
#include <time.h>
#include <Wire.h> //Needed for I2C communication

/*
 * Time
 *
 * This sketch demonstrates how to retrieve the Iridium system time
 * from the modem using the getSystemTime method. This uses
 * the Iridium command AT-MSSTM to acquire the time.  The method will
 * fail if the Iridium network has not yet been acquired.
 *
 * Assumptions
 *
 * The sketch assumes an Arduino Mega or other Arduino-like device with
 * a serial console and a hardware I2C (Wire) port. It assumes
 * the SparkFun Qwiic Iridium 9603N is connected via I2C.
 */

#define IridiumWire Wire
#define DIAGNOSTICS false // Change this to see diagnostics

// Declare the IridiumSBD object using default I2C address
IridiumSBD modem(IridiumWire);

void setup()
{
  int err;

  // Start the console serial port
  Serial.begin(115200);
  while (!Serial); // Wait for the user to open the serial monitor
  Serial.println(F("Iridium SBD Time I2C"));

  //empty the serial buffer
  while(Serial.available() > 0) Serial.read();

  //wait for the user to press any key before beginning
  Serial.println(F("Press any key to start example."));
  while(Serial.available() == 0);

  //clean up
  while(Serial.available() > 0) Serial.read();

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
    return;
  }
}

void loop()
{
   struct tm t; // struct tm is defined in time.h
   int err = modem.getSystemTime(t); // Ask the 9603N for the system time
   if (err == ISBD_SUCCESS) // Was it successful?
   {
      char buf[32];
      sprintf(buf, "%d-%02d-%02d %02d:%02d:%02d",
         t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
      Serial.print(F("Iridium date/time is "));
      Serial.println(buf);
   }

   else if (err == ISBD_NO_NETWORK) // Did it fail because the 9603N has not yet seen the network?
   {
      Serial.println(F("No network detected.  Waiting 10 seconds."));
   }

   else
   {
      Serial.print(F("Unexpected error "));
      Serial.println(err);
      return;
   }

   // Delay 10 seconds
   delay(10 * 1000UL);
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
