#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C

/*
 * Sleep
 *
 * This sketch demonstrates how to use the RockBLOCK Sleep pin.  The
 * general strategy is to call modem.begin() to start, then modem.sleep()
 * to stop.
 *
 * Assumptions
 *
 * The sketch assumes an Arduino Mega or other Arduino-like device with
 * multiple HardwareSerial ports.  It assumes the satellite modem is
 * connected to Serial1.  Change this as needed.  SoftwareSerial on an Uno
 * works fine as well.
 *
 * This sketch also assumes that pin 4 is connected to ON/OFF
 */

#define IridiumSerial Serial1
#define SLEEP_PIN 4
#define DIAGNOSTICS false // Change this to see diagnostics

// Declare the IridiumSBD object (note SLEEP pin)
IridiumSBD modem(IridiumSerial, SLEEP_PIN);

void setup()
{
  // Start the console serial port
  Serial.begin(115200);
  while (!Serial);

  // Start the serial port connected to the satellite modem
  IridiumSerial.begin(19200);
}

void loop()
{
  // Begin satellite modem operation
  Serial.println(F("Starting modem..."));
  int err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("Begin failed: error "));
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println(F("No modem detected: check wiring."));
    return;
  }

  // Send the message
  Serial.println(F("Trying to send a message.  This might take several minutes."));
  err = modem.sendSBDText("Hello, world! (Sleep test)");
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("sendSBDText failed: error "));
    Serial.println(err);
    if (err == ISBD_SENDRECEIVE_TIMEOUT)
      Serial.println(F("Try again with a better view of the sky."));
  }

  else
  {
    Serial.println(F("Hey, it worked!"));
  }

  // Clear the Mobile Originated message buffer
  Serial.println(F("Clearing the MO buffer."));
  err = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("clearBuffers failed: error "));
    Serial.println(err);
  }

  // Put modem to sleep
  Serial.println(F("Putting modem to sleep."));
  err = modem.sleep();
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("sleep failed: error "));
    Serial.println(err);
  }

  // Demonstrate that device is asleep
  Serial.println(F("Trying to send while asleep."));
  err = modem.sendSBDText("This shouldn't work.");
  if (err == ISBD_IS_ASLEEP)
  {
    Serial.println(F("Couldn't send: device asleep."));
  }
  else
  {
    Serial.print(F("Send failed for some other reason: error "));
    Serial.println(err);
  }

  Serial.println(F("Sleeping for a minute."));
  delay(60 * 1000UL);
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
