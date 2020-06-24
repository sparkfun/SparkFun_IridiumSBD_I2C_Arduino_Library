#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C

#include <Wire.h> //Needed for I2C communication

/*
 * Sleep
 *
 * This sketch demonstrates how to put the Qwiic Iridium to sleep.  The
 * general strategy is to call modem.begin() to start, then modem.sleep()
 * to stop.
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
  // Start the console serial port
  Serial.begin(115200);
  while (!Serial); // Wait for the user to open the serial monitor
  Serial.println(F("Iridium SBD Sleep I2C"));

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
}

void loop()
{
  int err;

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

  // Send the message
  Serial.println(F("Trying to send the message.  This might take several minutes."));
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

  // Power down the modem
  Serial.println(F("Putting the 9603N to sleep."));
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
