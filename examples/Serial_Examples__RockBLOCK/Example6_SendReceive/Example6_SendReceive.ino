#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C

/*
 * SendReceive
 *
 * This sketch demonstrates a basic bidirectional transmission.  Before
 * loading this sketch, send a message to your Iridium modem from your
 * control panel or via email.  This sketch will send a binary buffer
 * of 11 bytes, then attempt to read the next incoming messages (if any).
 * It stops when there are no more messages to read.
 *
 * Assumptions
 *
 * The sketch assumes an Arduino Mega or other Arduino-like device with
 * multiple HardwareSerial ports.  It assumes the satellite modem is
 * connected to Serial1.  Change this as needed.  SoftwareSerial on an Uno
 * works fine as well.
 *
 */

#define IridiumSerial Serial1
#define DIAGNOSTICS false // Change this to see diagnostics

IridiumSBD modem(IridiumSerial);

// Define the binary test message (Fibonacci sequence)
uint8_t buffer[200] =
{ 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89 };

void setup()
{
  int signalQuality = -1;

  // Start the serial ports
  Serial.begin(115200);
  while (!Serial);
  IridiumSerial.begin(19200);

  // If desired, you can set the SBD session timeout to prevent unhelpful ISBD_PROTOCOL_ERRORs.
  // The default is 0 (no timeout).
  #define SATELLITE_TIMEOUT_SBDIX 30
  //modem.adjustATTimeout(SATELLITE_TIMEOUT_SBDIX + 2); // Set the AT timeout slightly longer
  //modem.adjustSBDSessionTimeout(SATELLITE_TIMEOUT_SBDIX); // Set the SBD timeout to 30 seconds

  // Setup the Iridium modem
  modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);
  if (modem.begin() != ISBD_SUCCESS)
  {
    Serial.println(F("Couldn't begin modem operations."));
    exit(0);
  }

  // Check the signal quality (optional)
  int err = modem.getSignalQuality(signalQuality);
  if (err != 0)
  {
    Serial.print(F("SignalQuality failed: error "));
    Serial.println(err);
    exit(1);
  }

  Serial.print(F("Signal quality is "));
  Serial.println(signalQuality);
}

static bool messageSent = false;

void loop()
{
  int err;

  // Read/Write the first time or if there are any remaining messages
  if (!messageSent || modem.getWaitingMessageCount() > 0)
  {
    size_t bufferSize = sizeof(buffer);

    // First time through send+receive; subsequent loops receive only (send a NULL message)
    if (!messageSent)
      err = modem.sendReceiveSBDBinary(buffer, 11, buffer, bufferSize);
    else
      err = modem.sendReceiveSBDText(NULL, buffer, bufferSize);

    if (err != ISBD_SUCCESS)
    {
      Serial.print(F("sendReceiveSBD* failed: error "));
      Serial.println(err);
    }
    else // success!
    {
      messageSent = true;
      Serial.print(F("Inbound buffer size is "));
      Serial.println(bufferSize);
      for (int i=0; i<bufferSize; ++i)
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

    // Clear the Mobile Originated message buffer to avoid re-sending the message during subsequent loops
    Serial.println(F("Clearing the MO buffer."));
    err = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
    if (err != ISBD_SUCCESS)
    {
      Serial.print(F("clearBuffers failed: error "));
      Serial.println(err);
    }
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
