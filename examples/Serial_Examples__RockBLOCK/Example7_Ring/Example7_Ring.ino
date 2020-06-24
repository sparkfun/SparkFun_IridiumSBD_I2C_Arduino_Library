#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C

/*
 * Ring
 *
 * This sketch demonstrates how to use the Iridium RING line to detect
 * when inbound messages are available and retrieve them.
 *
 * Assumptions
 *
 * The sketch assumes an Arduino Mega or other Arduino-like device with
 * multiple HardwareSerial ports.  It assumes the satellite modem is
 * connected to Serial1.  Change this as needed.  SoftwareSerial on an Uno
 * works fine as well.
 *
 * This sketch assumes the Ring Indicator pin is connected to Arduino pin 5
 *
 */

#define IridiumSerial Serial1
#define RING_PIN 5
#define DIAGNOSTICS false // Change this to see diagnostics

IridiumSBD modem(IridiumSerial, -1, RING_PIN);

void setup()
{
  int signalQuality = -1;

  // Start the serial ports
  Serial.begin(115200);
  while (!Serial);
  IridiumSerial.begin(19200);

  // Setup the Iridium modem
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);
  if (modem.begin() != ISBD_SUCCESS)
  {
    Serial.println(F("Couldn't begin modem operations."));
    exit(0);
  }

  // Check signal quality for fun.
  int err = modem.getSignalQuality(signalQuality);
  if (err != 0)
  {
    Serial.print(F("SignalQuality failed: error "));
    Serial.println(err);
    return;
  }

  Serial.print(F("Signal quality is "));
  Serial.println(signalQuality);
  Serial.println(F("Begin waiting for RING..."));
}


void loop()
{
  static int err = ISBD_SUCCESS;
  bool ring = modem.hasRingAsserted();
  if (ring || modem.getWaitingMessageCount() > 0 || err != ISBD_SUCCESS)
  {
    if (ring)
      Serial.println(F("RING asserted!  Let's try to read the incoming message."));
    else if (modem.getWaitingMessageCount() > 0)
      Serial.println(F("Waiting messages available.  Let's try to read them."));
    else
      Serial.println(F("Let's try again."));

    // Clear the Mobile Originated message buffer - just in case it has an old message in it!
    Serial.println(F("Clearing the MO buffer (just in case)."));
    err = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
    if (err != ISBD_SUCCESS)
    {
      Serial.print(F("clearBuffers failed: error "));
      Serial.println(err);
      return;
    }

    uint8_t buffer[200];
    size_t bufferSize = sizeof(buffer);
    err = modem.sendReceiveSBDText(NULL, buffer, bufferSize);
    if (err != ISBD_SUCCESS)
    {
      Serial.print(F("sendReceiveSBDBinary failed: error "));
      Serial.println(err);
      return;
    }

    Serial.println(F("Message received!"));
    Serial.print(F("Inbound message size is "));
    Serial.println(bufferSize);
    for (int i=0; i<(int)bufferSize; ++i)
    {
      Serial.print(buffer[i], HEX);
      if (isprint(buffer[i]))
      {
        Serial.print(F("("));
        Serial.write(buffer[i]);
        Serial.print(F(")"));
      }
      Serial.print(F(" "));
    }
    Serial.println();
    Serial.print(F("Messages remaining to be retrieved: "));
    Serial.println(modem.getWaitingMessageCount());
  }
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
