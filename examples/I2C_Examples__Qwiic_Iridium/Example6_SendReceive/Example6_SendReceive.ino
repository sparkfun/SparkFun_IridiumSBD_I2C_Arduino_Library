#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C

#include <Wire.h> //Needed for I2C communication

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
 * a serial console and a hardware I2C (Wire) port. It assumes
 * the SparkFun Qwiic Iridium 9603N is connected via I2C.
 */

#define IridiumWire Wire
#define DIAGNOSTICS false // Change this to enable diagnostics

// Declare the IridiumSBD object using default I2C address
IridiumSBD modem(IridiumWire);

// Define the binary test message (Fibonacci sequence)
uint8_t buffer[200] =
{ 1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89 };

void setup()
{
  int signalQuality = -1;

  // Start the console serial port
  Serial.begin(115200);
  while (!Serial); // Wait for the user to open the serial monitor
  Serial.println(F("Iridium SBD SendReceive I2C"));

  //empty the serial buffer
  while(Serial.available() > 0) Serial.read();

  //wait for the user to press any key before beginning
  Serial.println(F("Press any key to start example."));
  while(Serial.available() == 0);

  //clean up
  while(Serial.available() > 0) Serial.read();

  // If desired, you can set the SBD session timeout to prevent unhelpful ISBD_PROTOCOL_ERRORs.
  // The default is 0 (no timeout).
  #define SATELLITE_TIMEOUT_SBDIX 30
  //modem.adjustATTimeout(SATELLITE_TIMEOUT_SBDIX + 2); // Set the AT timeout slightly longer
  //modem.adjustSBDSessionTimeout(SATELLITE_TIMEOUT_SBDIX); // Set the SBD timeout to 30 seconds

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
  int err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("Begin failed: error "));
    Serial.println(err);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println(F("No modem detected: check wiring."));
    return;
  }

  // Check the signal quality (optional)
  err = modem.getSignalQuality(signalQuality);
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
