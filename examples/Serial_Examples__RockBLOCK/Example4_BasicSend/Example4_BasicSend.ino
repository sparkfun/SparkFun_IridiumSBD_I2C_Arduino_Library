#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C

/*
 * BasicSend
 *
 * This sketch sends a "Hello, world!" message from the satellite modem.
 * If you have activated your account and have credits, this message
 * should arrive at the endpoints (delivery group) you have configured
 * (email address or HTTP POST).
 *
 * Assumptions
 *
 * The sketch assumes an Arduino Mega or other Arduino-like device with
 * multiple HardwareSerial ports.  It assumes the satellite modem is
 * connected to Serial1.  Change this as needed.  SoftwareSerial on an Uno
 * works fine as well.
 */

#define IridiumSerial Serial1
#define DIAGNOSTICS false // Change this to see diagnostics

// Declare the IridiumSBD object
IridiumSBD modem(IridiumSerial);

void setup()
{
  int signalQuality = -1;
  int err;

  // Start the console serial port
  Serial.begin(115200);
  while (!Serial);

  // Start the serial port connected to the satellite modem
  IridiumSerial.begin(19200);

  // Begin satellite modem operation
  Serial.println(F("Starting modem..."));
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("Begin failed: error "));
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println(F("No modem detected: check wiring."));
    return;
  }

  // Example: Print the firmware revision
  char version[12];
  err = modem.getFirmwareVersion(version, sizeof(version));
  if (err != ISBD_SUCCESS)
  {
     Serial.print(F("FirmwareVersion failed: error "));
     Serial.println(err);
     return;
  }
  Serial.print(F("Firmware Version is "));
  Serial.print(version);
  Serial.println(F("."));

  // Example: Print the IMEI
  char IMEI[16];
  err = modem.getIMEI(IMEI, sizeof(IMEI));
  if (err != ISBD_SUCCESS)
  {
     Serial.print(F("getIMEI failed: error "));
     Serial.println(err);
     return;
  }
  Serial.print(F("IMEI is "));
  Serial.print(IMEI);
  Serial.println(F("."));

  // Example: Test the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  err = modem.getSignalQuality(signalQuality);
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("SignalQuality failed: error "));
    Serial.println(err);
    return;
  }

  Serial.print(F("On a scale of 0 to 5, signal quality is currently "));
  Serial.print(signalQuality);
  Serial.println(F("."));

  // Send the message
  Serial.println(F("Trying to send the message.  This might take several minutes."));
  err = modem.sendSBDText("Hello, world!");
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

  Serial.println(F("Done!"));
}

void loop()
{
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
