#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C

/*
 * PassThru
 *
 * This sketch allows you to send data directly from the Serial console
 * to the modem.  Characters typed in the console are relayed to the
 * modem and vice versa.
 *
 * The sketch assumes an Arduino Mega or other Arduino-like device with
 * multiple HardwareSerial ports.  It assumes the satellite modem is
 * connected to Serial1.  Change this as needed.  SoftwareSerial on an Uno
 * works fine as well.
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

#define IridiumSerial Serial1
#define DIAGNOSTICS false // Change this to see diagnostics

IridiumSBD modem(IridiumSerial);

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

  Serial.println(F("Enter commands terminated by Carriage Return ('\\r'):"));
}

void loop()
{
  if (Serial.available())
    IridiumSerial.write(Serial.read());
  if (IridiumSerial.available())
    Serial.write(IridiumSerial.read());
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
