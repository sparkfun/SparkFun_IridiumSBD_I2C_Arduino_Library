#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C

#include <Wire.h> //Needed for I2C communication

/*
 * Loop Back
 *
 * This sketch demonstrates how to forward a message to another RockBLOCK
 * via the Rock7 RockBLOCK Gateway. The message is looped back to your
 * RockBLOCK by prefixing the message with "RB" and the RockBLOCK serial
 * number of your transceiver padded out to seven digits.
 *
 * Assumptions
 *
 * The sketch assumes an Arduino Mega or other Arduino-like device with
 * a serial console and a hardware I2C (Wire) port. It assumes
 * the SparkFun Qwiic Iridium 9603N is connected via I2C.
 *
 * Open the Serial Monitor and set the Baud Rate to 115200
 * and the line ending to Carriage Return
 *
 * Part of the code is based on Tom Igoe's Serial Event example:
 * https://www.arduino.cc/en/Tutorial/SerialEvent
 */

#define IridiumWire Wire
#define DIAGNOSTICS false // Change this to enable diagnostics

// Declare the IridiumSBD object using default I2C address
IridiumSBD modem(IridiumWire);

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
#define max_length 10 // Define maximum inputString length to avoid buffer overruns

void setup()
{
  // Start the console serial port
  Serial.begin(115200);
  while (!Serial); // Wait for the user to open the serial monitor
  Serial.println(F("Iridium SBD Loop Back I2C"));

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

  // Enable 9603N power
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
    exit(0);
  }

  inputString.reserve(max_length); // Reserve max_length bytes for the input string
}

/*
   SerialEvent occurs whenever a new data comes in the
  hardware serial RX.  This routine is run between each
  time loop() runs, so using delay inside loop can delay
  response.  Multiple bytes of data may be available.
*/
void serialEvent() {
   while (Serial.available()) {
     // get the new byte:
     char inChar = (char)Serial.read();
     // check how many characters have already been received
     if (inputString.length() < max_length) {
       // we have room so add inChar to the inputString:
       inputString += inChar;
     }
     // if the incoming character is a carriage return, set a flag
     // so the main loop can do something about it:
     if (inChar == '\r') {
       stringComplete = true;
     }
   }
}

void loop()
{
  // Clear the serial string and flag
  inputString = "";
  stringComplete = false;

  Serial.println(F("Please enter the RockBLOCK serial number of your transceiver followed by carriage return:"));
  while (!stringComplete)
  {
    serialEvent(); // Check for serial event
  }
  // Process the string when a carriage return arrives:
  int RockBLOCK = inputString.toInt(); // Convert the serial number to int
  Serial.println(RockBLOCK);

  // Construct the message
  // Prefix with "RB" and the RockBLOCK serial number padded out to seven digits
  char outBuffer[50]; // Always try to keep message short
  sprintf(outBuffer, "RB%07iThis is a test message!", RockBLOCK);

  Serial.print(F("Transmitting message: '"));
  Serial.print(outBuffer);
  Serial.println(F("'"));

  int err = modem.sendSBDText(outBuffer);
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("Transmission failed with error code "));
    Serial.println(err);
  }
  else
  {
    Serial.println(F("Message sent!"));
  }

  // Clear the MO buffer
  Serial.println(F("Clearing the MO buffer."));
  err = modem.clearBuffers(ISBD_CLEAR_MO); // Clear the MO buffer
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("clearBuffers failed: error "));
    Serial.println(err);
  }

  Serial.println(F("Waiting for 30 seconds..."));
  delay(30000);

  // Now retrieve the loop backed message
  Serial.println(F("Checking if the message looped back successfully."));

  uint8_t buffer[60];
  size_t bufferSize = sizeof(buffer);

  err = modem.sendReceiveSBDText(NULL, buffer, bufferSize);

  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("sendReceiveSBD* failed: error "));
    Serial.println(err);
  }
  else // success!
  {
    Serial.print(F("Inbound buffer size is "));
    Serial.println(bufferSize);
    if (bufferSize > 0)
    {
      Serial.print(F("The received message is: '"));
      for (int i=0; i<bufferSize; ++i)
      {
        if (isprint(buffer[i]))
        {
          Serial.write(buffer[i]);
        }
      }
      Serial.println(F("'"));
    }
    else
    {
      Serial.println(F("The loop back failed. Are you sure you entered the correct serial number?"));
    }
  }

  // Clear the MT buffer
  Serial.println(F("Clearing the MT buffer."));
  err = modem.clearBuffers(ISBD_CLEAR_MT); // Clear the MT buffer
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("clearBuffers failed: error "));
    Serial.println(err);
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
