#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C

#include <Wire.h> //Needed for I2C communication

/*
 * PassThru_I2C
 *
 * This sketch allows you to send data directly from the serial console
 * to the modem.  Characters typed in the console are relayed to the
 * modem and vice versa.
 *
 * The sketch assumes an Arduino Mega or other Arduino-like device with
 * a serial console and a hardware I2C (Wire) port. It assumes
 * the SparkFun Qwiic Iridium 9603N is connected via I2C.
 *
 * Open the Serial Monitor and set the Baud Rate to 115200
 * and the line ending to Carriage Return
 *
 * Command    Response                Description
 * AT+CGMI    Iridium                 Get manufacturer identification
 * AT+CGMM    <model ID>              Get model identification
 * AT+CGMR    <version information>   Get revision
 * AT+CGSN    <IMEI>                  Get serial number (IMEI)
 * AT+CSQ     <signal quality 0-5>    Get signal quality
 *
 * To send a Mobile Originated text message and check for a new Mobile Terminated message:
 * AT+SBDWT=Testing123                Write a text message to the MO buffer
 * AT+SBDS                            Read the MO and MT buffer status (MO flag should be '1')
 * AT+SBDIX                           Initiate an SBD session (wait for it to complete - could take several minutes!)
 * AT+SBDS                            Read the MO and MT buffer status (MO flag should still be '1';
 *                                    MT flag will be '1' if a new message was received)
 * AT+SBDD0                           Clear the MO buffer
 * AT+SBDRT                           Read the received text message (if there was one)
 * AT+SBDD1                           Clear the MT buffer
 * AT+SBDS                            Read the MO and MT buffer status (Both MO and MT flags should be '0')
 *
 * The full AT command set can be found here:
 * http://www.rock7mobile.com/downloads/IRDM_ISU_ATCommandReferenceMAN0009_Rev2.0_ATCOMM_Oct2012.pdf
 * (Only a small sub-set of the commands apply to the 9603N)
 */

#define IridiumWire Wire
#define DIAGNOSTICS false // Change this to see diagnostics

// Declare the IridiumSBD object using default I2C address
IridiumSBD modem(IridiumWire);

#define bufferSize 100 // Tx/Rx Buffer size

void setup()
{
  int signalQuality = -1;
  int err;

  // Start the console serial port
  Serial.begin(115200);
  while (!Serial); // Wait for the user to open the serial monitor
  Serial.println(F("Iridium SBD PassThru I2C"));

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

  // Test the signal quality.
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

  Serial.println();
  Serial.println(F("Enter commands terminated by Carriage Return ('\\r'):"));
}

void loop()
{
  int err;

  uint8_t TxRxBuffer[bufferSize]; // Create a buffer
  size_t TxRxBufferSize = 0; // Set the buffer size to zero
  // Check if there is any serial data to be sent to the 9603N (and that we haven't filled the buffer)
  while ((Serial.available() > 0) && (TxRxBufferSize < bufferSize))
  {
    TxRxBuffer[TxRxBufferSize] = Serial.read(); // Read a byte and store in the buffer
    TxRxBufferSize++; // Increment the buffer size
  }
  if (TxRxBufferSize > 0) // If there is data in the outBuffer
  {
    err = modem.passThruI2Cwrite(TxRxBuffer, TxRxBufferSize); // Write it using pass thru
    if (err != ISBD_SUCCESS) // If there was an error, print the error code
    {
      Serial.print(F("passThruI2Cwrite failed: error "));
      Serial.println(err);
    }
  }

  // Set the buffer size to the maximum size of the buffer
  // so passThruI2Cread knows how big the buffer is (and doesn't overflow it!)
  TxRxBufferSize = bufferSize;
  size_t bytesRead = 0; // passThruI2Cread will set this to the number of serial bytes written into the buffer
  err = modem.passThruI2Cread(TxRxBuffer, TxRxBufferSize, bytesRead); // Read any waiting data using pass thru
  if (err != ISBD_SUCCESS) // If there was an error, print the error code
  {
    Serial.print(F("passThruI2Cread returned error "));
    Serial.println(err);
  }
  if (bytesRead > 0) // If some data was received
  {
    size_t ptr = 0;
    while (ptr < bytesRead)
    {
      Serial.write(TxRxBuffer[ptr]); // Send the data one byte at a time to the serial port
      ptr++;
    }
  }

  delay(5); // Delay to avoid thrashing the I2C bus
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
