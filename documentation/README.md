# IridiumSBD Arduino Library: Documentation

Gratefully adapted from [Mikal Hart's documentation](https://github.com/mikalhart/IridiumSBD/tree/master/extras) for version 2.0 of the library.

## Overview

The Iridium 9602 and 9603N are fascinating communications modules that give devices like Arduino or Raspberry Pi&trade; access to the Iridium satellite network.
This is a big deal, because it means that your application can now easily and inexpensively communicate from any point on or above the globe,
from the heart of the Amazon to the Siberian tundra, from the Arctic to the Antarctic.
This library, **IridiumSBD**, uses Iridium's **SBD** ("Short Burst Data") protocol to send and receive short messages to/from the Iridium hub.
SBD is a "text message"-like technology that supports the transmission of text or binary messages up to a certain maximum size (270 bytes received, 340 bytes transmitted).

Breakout boards for the 9602 and 9603N are available in different formats. Most of these breakouts use serial (UART) interfacing but I2C is possible too and this version of
the library supports both. A full set of [examples](../examples) demonstrate how to use both interfaces.

## Serial "3-wire" Interfacing

Rock 7 have made interfacing the RockBLOCK to the Arduino quite simple.  Each RockBLOCK conveniently exposes many signal lines for the client device, but it's actually
only necessary to connect 4 or 5 of these to get your application up and running. In our configuration we ignore the flow control lines and talk to the RockBLOCK over what
Iridium calls a "3-wire" TTL serial interface.  In the wiring table below, we assume that the RockBLOCK is being powered from the Arduino 5V power bus.

| RockBLOCK Connection | Arduino Connection |
| --- | --- |
| +5V (Power) | +5V (Power) |
| GND | GND  |
| TX | TX Serial Pin |
| RX | RX Serial Pin |
| SLEEP (optional) | GPIO pin |
| RING (optional) | GPIO pin |

The TX and RX lines are labeled on the RockBLOCK as viewed from the Arduino, so the TX line would be transmitting serial data _to_ the RockBLOCK.  These lines support
TTL-level serial (default 19200 baud), so you can either connect it directly to a built-in UART or create a "soft" serial on any two suitable pins. We usually opt for the
latter on smaller devices like Uno to free up the UART(s) for diagnostic and other console communications.

The active low SLEEP wire may be pulled high (indicating that the device is perpetually awake), but it's a good power saving technique to connect it to a general-purpose pin,
allowing the library to put the RockBLOCK into a low power "sleep" state when its services are not needed.  The RING line is used to alert the client that a message is available.

## I2C Interfacing

Connections to the Qwiic Iridium are made via SparkFun's standard 4-pin Qwiic connector. The four pins are:

| Pin | Signal | Usual Wire Color |
| --- | --- | --- |
| 1 | GND / 0V | Black |
| 2 | **3.3V** Power | Red |
| 3 | I2C Data SDA | Blue |
| 4 | I2C Clock SCL | Yellow |

The Qwiic Iridium has a default I2C address of 0x63; all I2C communication is done transparently through the library.
If the I2C address has been changed, you can use the new value when calling the constructor.

Access to the 9603N's Ring Indicator and Network Available pins is also done through the library.

## Non-blocking Retry Strategy

The nature of satellite communications is such that it often takes quite a long time to establish a link.  Satellite communications are line-of-sight, so having a clear view
of an unclouded sky greatly improves speed and reliability; however, establishing contact may be difficult even under ideal conditions for the simple reason that at a given time
satellites are not always overhead. In these cases, the library initiates a behind-the-scenes series of retries, waiting for satellites to appear.

With a clear sky, transmissions eventually almost always succeed, but the entire process may take up to several minutes. Since most microcontroller applications cannot tolerate
blocking delays of this length, **IridiumSBD** provides a callback mechanism to ensure that the Arduino can continue performing critical tasks. Specifically, if the library user
provides a global C++ function with the signature

```
bool ISBDCallback();
```

(and this is recommended for all but the most trivial applications), then that function will be called repeatedly while the library is waiting for long operations to complete.
In it you can take care of activities that need doing while you're waiting for the transmission to complete. As a simple example, this blinks an LED during the library operations:

```
bool ISBDCallback()
{
  unsigned ledOn = (millis() / 1000) % 2;
  digitalWrite(ledPin, ledOn ? HIGH : LOW); // Blink LED every second
  return true;
}

// This transmission may take a long time, but the LED keeps blinking
modem.sendSBDText("Hello, mother!");
```

**Note:** Most IridiumSBD methods are not available from within the callback and return the error code **ISBD_REENTRANT** when called.

**Note:** Your application can prematurely terminate a pending IridiumSBD operation by returning **false** from the callback.  This causes the pending operation to immediately
return **ISBD_CANCELLED**.

## Power Considerations

The RockBLOCK module uses a "super capacitor" to supply power to the Iridium 9602/9603.  As the capacitor is depleted through repeated transmission attempts, the host device's
power bus replenishes it. Under certain low power conditions it is important that the library not retry too quickly, as this can drain the capacitor and render the device inoperative.
In particular, when powered by a low-power 90 mA max USB supply, the interval between transmit retries should be extended to as much as 60 seconds, compared to 20 for, say, a high-current
battery solution.

To transparently support these varying power profiles, **IridiumSBD** provides the ability to fine-tune the delay between retries.  This is done by calling

```
// For USB "low current" applications
modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);
```

or

```
// For "high current" (battery-powered) applications
modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);
```

## Construction and Startup

### Serial Interfacing

To begin using the library with the serial interface, first create an **IridiumSBD** object. The **IridiumSBD** constructor binds the new object to an Arduino **Stream** (i.e. the device's
serial port) and, optionally, its SLEEP and RING lines:

```
IridiumSBD(Stream &stream, int sleepPinNo = -1, int ringPinNo = -1);
```

Example startup:

```
#include <IridiumSBD.h>
#include <SoftwareSerial.h>

SoftwareSerial ssIridium(18, 19); // RockBLOCK serial port on 18/19
IridiumSBD modem(ssIridium, 10);  // SLEEP pin on 10, RING pin not connected

void setup()
{
   modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);
   modem.begin(); // Wake up the 9602 and prepare it for communications.
   ...
```

### I2C Interfacing

To begin using the library with the I2C interface, create an **IridiumSBD** object binding the object to Wire instead of serial:

```
IridiumSBD(TwoWire &wirePort = Wire, uint8_t deviceAddress = 0x63);
```

Example startup:

```
#include <IridiumSBD.h>
#include <Wire.h>

IridiumSBD modem(Wire);

void setup()
{
   modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);
   modem.begin(); // Wake up the 9603N and prepare it for communications.
   ...
```

## Data transmission

The methods that make up the core of the **IridiumSBD** public interface, are, naturally, those that send and receive data. There are four such methods in **IridiumSBD**:
two "send-only" functions (text and binary), and two "send-and-receive" functions (again, text and binary):

```
// Send a text message
int sendSBDText(const char *message);

// Send a binary message
int sendSBDBinary(const uint8_t *txData, size_t txDataSize);

// Send a text message and receive one (if available)
int sendReceiveSBDText(const char *message, uint8_t *rxBuffer, size_t &rxBufferSize);

// Send a binary message and receive one (if available)
int sendReceiveSBDBinary(const uint8_t *txData, size_t txDataSize, uint8_t *rxBuffer, size_t &rxBufferSize);
```

Applications using serial and I2C interfaces make use of the same four methods, the low level communication is routed automatically to the correct interface by the library.

## Send-only and Receive-only applications

Note that at the lowest-level, Iridium SBD transactions always involve the sending _and_ receiving of exactly one message (if one is available). That means that if you call
the send-only variants **sendSBDText** or **sendSBDBinary** and messages happen to be waiting in your incoming (RX) message queue, the first of these is discarded and irrevocably lost.
This may be perfectly acceptable for an  application that doesn't care about inbound messages, but if there is some chance that you will need to process one, call **sendReceiveSBDText**
or **sendReceiveSBDBinary** instead.

If your application is _receive-only_, call **sendReceiveSBDText** with a **NULL** outbound message parameter.

If no inbound message is available, the **sendReceive** messages indicate this by returning **ISBD_SUCCESS** and setting **rxBufferSize** to 0.

## Diagnostics  

**IridiumSBD** operates by maintaining a serial or I2C dialog with the Iridium 9602/3. To diagnose failures it is often useful to "spy" on this conversation. If you provide a callback function
with the signature

```
void ISBDConsoleCallback(IridiumSBD *device, char c);
```

the library will call it repeatedly with data in this conversation, and your application can log it. Similarly, to inspect the run state of the library as it is working, simply provide a
callback like this:

```
void ISBDDiagsCallback(IridiumSBD *device, char c);
```

These callbacks allow the host application to monitor Iridium traffic and the library's diagnostic messages. The typical usage is to simply forward both to the Arduino serial port
for display in the serial monitor:

```
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
```

## Receiving Multiple Messages

After every successful SBD send/receive operation, the Iridium satellite system informs the client how many messages remain in the inbound message queue. The library reports this value
with the **getWaitingMessageCount** method. Here's an example of a loop that reads all the messages in the inbound message queue:

```
do
{
   char rxBuffer[100];
   size_t bufferSize = sizeof(rxBuffer);
   int status = modem.sendReceiveText(NULL, rxBuffer, bufferSize);
   if (status != ISBD_SUCCESS)
   {
      /* ...process error here... */
      break;
   }
   if (bufferSize == 0)
      break; // all done!
   /* ...process message in rxBuffer here... */
} while (modem.getWaitingMessageCount() > 0);
```

Note that **getWaitingMessageCount** is only valid after a successful send/receive operation.

## Qwiic Iridium (I2C) Power Control

Like the RockBLOCK, power for the 9603N transceiver on the Qwiic Iridium is provided by super capacitors. The correct power sequence is:
- Enable the super capacitor charger
- Wait for the capacitors to charge
- Enable power for the 9603N
- begin() the modem
- Send/receive message(s)
- sleep() the modem
- Disable 9603N power
- Disable the super capacitor charger
- Place the Qwiic Iridium's ATtiny841 into low power mode (optional)

The library contains a full set of methods to control the power and check the status of the Qwiic Iridium. In their most condensed form, the methods should be called as follows:

```
if (modem.isConnected()) // Check that the Qwiic Iridium is connected
{
  modem.enableSuperCapCharger(true); // Enable the super capacitor charger
  while (!modem.checkSuperCapCharger()) ; // Wait for the capacitors to charge
  modem.enable9603Npower(true); // Enable power for the 9603N
  modem.begin(); // Wake up the modem
  modem.sendSBDText("Hello, world!"); // Send a message
  modem.sleep(); // Put the modem to sleep
  modem.enable9603Npower(false); // Disable power for the 9603N
  modem.enableSuperCapCharger(false); // Disable the super capacitor charger
  modem.enable841lowPower(true); // Enable the ATtiny841's low power mode (optional)
}
```

See below for a full description of each method.

## Qwiic Iridium (I2C) Pass Thru

If you want to communicate directly with the transceiver, so you can explore the AT command set manually, this is trivial when using serial. For the Qwiic Iridium, two new methods
allow you to parcel up your commands, send them to the transceiver via I2C, and unparcel the replies:
- **passThruI2Cread** allows you to read serial data directly from the 9603N without going through the higher level library methods.
- **passThruI2Cwrite** allows you to write directly to the transceiver as if you were connected via serial.

[Example 9](../examples/I2C_Examples__Qwiic_Iridium/Example9_PassThru/Example9_PassThru.ino) makes use of these methods.

These methods will return a ISBD_REENTRANT error if you attempt to call them while another method is in progress.

See below for a full description of each method.

## Erratum Workaround

In May, 2013, Iridium identified a potential problem that could cause a satellite modem like the RockBLOCK to lock up unexpectedly. This issue only affects devices with firmware older
than version TA13001: use **getFirmwareVersion()** to check yours. The library automatically employs the workaround recommended by Iridium - a successful check of the system time with
AT-MSSTM before each transmission - by default for firmware that is older, but you can disable this check with:

```
modem.useMSSTMWorkaround(false);
```

## Error return codes

Many **IridiumSBD** methods return an integer error status code, with ISBD_SUCCESS (0) indicating successful completion. These include **begin**, **sendSBDText**, **sendSBDBinary**,
**sendReceiveSBDText**, **sendReceiveSBDBinary**, **getSignalQuality**, and **sleep**.  Here is a complete list of the possible error return codes:

```
#define ISBD_SUCCESS             0
#define ISBD_ALREADY_AWAKE       1
#define ISBD_SERIAL_FAILURE      2
#define ISBD_PROTOCOL_ERROR      3
#define ISBD_CANCELLED           4
#define ISBD_NO_MODEM_DETECTED   5
#define ISBD_SBDIX_FATAL_ERROR   6
#define ISBD_SENDRECEIVE_TIMEOUT 7
#define ISBD_RX_OVERFLOW         8
#define ISBD_REENTRANT           9
#define ISBD_IS_ASLEEP           10
#define ISBD_NO_SLEEP_PIN        11
#define ISBD_NO_NETWORK          12
#define ISBD_MSG_TOO_LONG        13
```

## IridiumSBD Interface

---
### Constructor (Serial)
---
```
IridiumSBD(Stream &stream, int sleepPinNo = -1, int ringPinNo = -1)
```

- Description: Creates an IridiumSBD library object
- Returns: N/A
- Parameter: **stream** - The serial port that the RockBLOCK is connected to.
- Parameter: **sleepPin** - The number of the Arduino pin connected to the RockBLOCK SLEEP line.

Notes:
- Connecting and using the sleepPin is recommended for battery-based solutions. Use **sleep()** to put the RockBLOCK into a low-power state, and **begin()** to wake it back up.

---
### Constructor (I2C)
---
```
IridiumSBD(TwoWire &wirePort = Wire, uint8_t deviceAddress = 0x63)
```

- Description: Creates an IridiumSBD library object
- Returns: N/A
- Parameter: **wirePort** - The I2C (Wire) port that the Qwiic Iridium is connected to.
- Parameter: **deviceAddress** - The address used by the ATtiny841 for I2C communication.

Notes:
- Access to the sleepPin is provided internally through the library.

---
### Startup
---
```
int begin()
```

- Description: Starts (or wakes) the RockBLOCK modem.
- Returns:  ISBD_SUCCESS if successful, a non-zero code otherwise.
- Parameter: None.

Notes:
- **begin()** also serves as the way to wake a RockBLOCK that is asleep.
- At initial power up, this method make take several tens of seconds as the device charges.  When waking from sleep the process should be faster.
- If provided, the user's **ISBDCallback** function is repeatedly called during this operation.
- This function should be called before any transmit/receive operation

---
### Data transmission
---
```
int sendSBDText(const char *message)
```

- Description: Transmits a text message to the global satellite system.
- Returns: ISBD_SUCCESS if successful, a non-zero code otherwise
- Parameter: **message** - A 0-terminated string message.

Notes:
- The library calculates retries the operation for up to 300 seconds by default. (To change this value, call **adjustSendReceiveTimeout**.)
- The maximum size of a transmitted packet (including header and checksum) is 340 bytes.
- If there are any messages in the RX queue, the first of these is discarded when this function is called.
- If provided, the user's **ISBDCallback** function is repeatedly called during this operation.

---
```
int sendSBDBinary(const uint8_t *txData, size_t txDataSize)
```

- Description: Transmits a binary message to the global satellite system.
- Returns: ISBD_SUCCESS if successful, a non-zero code otherwise
- Parameter: **txData** - The buffer containing the binary data to be transmitted.
- Parameter: **txDataSize** - The size of the buffer in bytes.

Notes:
- The library calculates and transmits the required headers and checksums and retries the operation for up to 300 seconds by default. (To change this value, call **adjustSendReceiveTimeout**.)   
- The maximum size of a transmitted packet (including header and checksum) is 340 bytes.
- If there are any messages in the RX queue, the first of these is discarded when this function is called.
- If provided, the user's **ISBDCallback** function is repeatedly called during this operation.

---
```
int sendReceiveSBDText(const char *message, uint8_t *rxBuffer, size_t &rxBufferSize)
```

- Description: Transmits a text message to the global satellite system and receives a message if one is available.
- Returns: ISBD_SUCCESS if successful, a non-zero code otherwise
- Parameter: **message** - A 0-terminated string message.
- Parameter: **rxBuffer** - The buffer to receive the inbound message.
- Parameter: **rxBufferSize** - The size of the buffer in bytes.

Notes:
- The library calculates retries the operation for up to 300 seconds by default. (To change this value, call **adjustSendReceiveTimeout**.)
- The maximum size of a transmitted packet (including header and checksum) is 340 bytes.
- The maximum size of a received packet is 270 bytes.
- If provided, the user's **ISBDCallback** function is repeatedly called during this operation.
- The library returns the size of the buffer actually received into rxBufferSize. This value should always be set to the actual buffer size before calling sendReceiveSBDText.

---
```
int sendReceiveSBDBinary(const uint8_t *txData, size_t txDataSize, uint8_t *rxBuffer, size_t &rxBufferSize)
```

- Description: Transmits a binary message to the global satellite system and receives a message if one is available.
- Returns: ISBD_SUCCESS if successful, a non-zero code otherwise
- Parameter: **txData** - The buffer containing the binary data to be transmitted.
- Parameter: **txDataSize** - The size of the outbound buffer in bytes.
- Parameter: **rxBuffer** - The buffer to receive the inbound message.
- Parameter: **rxBufferSize** - The size of the buffer in bytes.

Notes:
- The library calculates and transmits the required headers and checksums and retries the operation for up to 300 seconds by default. (To change this value, call **adjustSendReceiveTimeout**.)
- The maximum size of a transmitted packet (including header and checksum) is 340 bytes.
- The maximum size of a received packet is 270 bytes.
- If provided, the user's **ISBDCallback** function is repeatedly called during this operation.
- The library returns the size of the buffer actually received into rxBufferSize. This value should always be set to the actual buffer size before calling sendReceiveSBDText.

---
### Utilities
---
```
int getSignalQuality(int &quality)
```

- Description: Queries the signal strength and visibility of satellites
- Returns: ISBD_SUCCESS if successful, a non-zero code otherwise
- Parameter: **quality** - Return value: the strength of the signal (0=nonexistent, 5=high)

Notes:
- If provided, the user's **ISBDCallback** function is repeatedly called during this operation.
- This method is mostly informational.  It is not strictly necessary for the user application to verify that a signal exists before calling one of the transmission functions, as these check signal quality themselves.

---
```
int getWaitingMessageCount()
```

- Description: Returns the number of waiting messages on the Iridium servers.
- Returns: The number of messages waiting, or -1 if unknown.
- Parameter: None.

Notes:
- This number is only valid if one of the send/receive methods has previously completed successfully. If not, the value returned from **getWaitingMessageCount** is -1 ("unknown").

---
```
int getSystemTime(struct tm &tm)
```

- Description: Returns the system time from the Iridium network.
- Returns: ISBD_SUCCESS if successful, a non-zero code otherwise.
- Parameter: **tm** - the time structure to be filled in

Notes:
- This method returns the Iridium network time in tm.
- "tm" is a C standard library structure defined in <time.h>
- Note that the tm_mon field is zero-based, i.e. January is 0
- This function uses AT-MSSTM, which might report "Network not found". In this case, the function returns ISBD_NO_NETWORK.

---
```
int sleep()
```

- Description: Puts the RockBLOCK into low power "sleep" mode
- Returns: ISBD_SUCCESS if successful, a non-zero code otherwise
- Parameter: **None**.

Notes:
- This method gracefully shuts down the RockBLOCK and puts it into low-power standby mode by bringing the active low SLEEP line low.
- Wake the device by calling **begin()**.
- If provided, the user's **ISBDCallback** function is repeatedly called during this operation.

---
```
bool isAsleep()
```

- Description: indicates whether the RockBLOCK is in low-power standby mode.
- Returns: **true** if the device is asleep
- Parameter: **None**.

---
```
bool hasRingAsserted()
```

- Description: indicates whether the RockBLOCK's RING line has asserted  
- Returns: **true** if RING has asserted
- Parameter: **None**.

---
```
int getFirmwareVersion(char *version, size_t bufferSize)
```

- Description: Returns a string representing the firmware revision number.
- Returns: ISBD_SUCCESS if successful, a non-zero code otherwise.
- Parameter: **version** - the buffer to contain the version string
- Parameter: **bufferSize** - the size of the buffer to be filled

Notes:
- This method returns the version string in the version buffer.
- bufferSize should be at least 8 to contain strings like TA13001 with the 0 terminator.

---
```
void setPowerProfile(POWERPROFILE profile)
```

- Description: Defines the device power profile
- Returns: **None**.
- Parameter: **profile** - USB_POWER_PROFILE for low-current USB power source, DEFAULT_POWER_PROFILE for default (battery) power

Notes:
- This method defines the internal delays between retransmission.  Low current applications may require longer delays.

---
```
void adjustATTimeout(int seconds)
```

- Description: Adjusts the internal timeout timer for serial AT commands
- Returns: **None**.
- Parameter: **seconds** - The maximum number of seconds to wait for a response to an AT command (default=20).

Notes:
- The Iridium 9602 frequently does not respond immediately to an AT command.  This value indicates the number of seconds IridiumSBD should wait before giving up.
- It is not expected that this method will be commonly used.

---
```
void adjustSendReceiveTimeout(int seconds)
```

- Description: Adjusts the internal timeout timer for the library send/receive commands
- Returns: **None**.
- Parameter: **seconds** - The maximum number of seconds to continue attempting retransmission of messages (default=300).

Notes:
- This setting indicates how long IridiumSBD will continue to attempt to communicate with the satellite array before giving up. The default value of 300 seconds (5 minutes)
seems to be a reasonable choice for many applications, but higher values might be more appropriate for others.

---
```
void useMSSTMWorkaround(bool useWorkaround)
```

- Description: Defines whether the library should use the technique described in the Iridium Product Advisor of 13 May 2013 to avoid possible lockout.
- Returns: **None**.
- Parameter: **useWorkaround** - "true" if the workaround should be employed; false otherwise. This value is set internally to "true" by default, on the assumption that the attached device may have an older firmware.

Notes:
- Affected firmware versions include TA11002 and TA12003. If your firmware version is later than these, you can save some time by setting this value to false.

---
```
void enableRingAlerts(bool enable)
```

- Description: Overrides whether the library should enable the RING alert signal pin and the unsolicited SBDRING notification.
- Returns: **None**.
- Parameter: **enable** - "true" if RING alerts should be enabled.

Notes:
- This method uses the Iridium AT+SBDMTA to enable or disable alerts.
- This method take effect at the next call to **begin()**, so typically you would call this before you start your device.  
- RING alerts are enabled by default if the library user has specified a RING pin for the IridiumSBD constructor.  Otherwise they are disabled by default.  Use this method to override that as needed.

---
```
int getIMEI(char *IMEI, size_t bufferSize)
```

- Description: Returns a string representing the IMEI.
- Returns: ISBD_SUCCESS if successful, a non-zero code otherwise.
- Parameter: **IMEI** - the buffer to contain the IMEI string.
- Parameter: **bufferSize** - the size of the buffer to be filled.

Notes:
- This method returns the IMEI string in the IMEI buffer.
- bufferSize should be at least 16 to contain the 15 digit IMEI with the 0 terminator.

---
```
int clearBuffers(int buffers = ISBD_CLEAR_MO)
```

- Description: Clears the Mobile Originated (MO), Mobile Terminated (MT) or Both message buffers.
- Returns: ISBD_SUCCESS if successful, a non-zero code otherwise.
- Parameter: **buffers** - the buffer(s) to be cleared.

Notes:
- Defaults to clearing the MO buffer (**ISBD_CLEAR_MO**).
- **buffers** can also be set to: **ISBD_CLEAR_MT** to clear the MT buffer; or **ISBD_CLEAR_BOTH** to clear both MO and MT buffers.

---
### Qwiic Iridium (I2C) Methods
---
```
void enableSuperCapCharger(bool enable)
```

- Description: Enables or disables the Qwiic Iridium's super capacitor charger.
- Returns: None.
- Parameter: **enable** - **true** will enable the charger, **false** will disable it.

---
```
bool checkSuperCapCharger()
```

- Description: Checks the status of the super capacitors.
- Returns: **true** if the capacitors are charged, **false** if not.
- Parameter: None.

Notes:
- Returns **true** when the capacitors are >= 94% charged.
- If the super capacitor charger is disabled, this method will return **true** (due to the PGOOD pin being an open collector). Ensure the charger is enabled before using this method.

---
```
void enable9603Npower(bool enable)
```

- Description: Enables power for the 9603N from the super capacitors.
- Returns: None.
- Parameter: **enable** - **true** will enable 9603N power, **false** will disable it.

Notes:
- Ensure the super capacitors are charged before enabling 9603N power. Use **checkSuperCapCharger** to confirm.

---
```
void enable9603(bool enable)
```

- Description: Enables the 9603N via its SLEEP (ON/OFF) pin.
- Returns: None.
- Parameter: **enable** - **true** will enable the 9603N, **false** will place it in sleep mode.

Notes:
- The user should not need to call this function. It should probably be private. The **begin** and **sleep** methods will automatically set the pin to the correct state.

---
```
bool checkRingIndicator()
```

- Description: Checks if the Qwiic Iridium has seen a Ring Indication.
- Returns: **true** if a ring indication has been seen, **false** if not.
- Parameter: None.

Notes:
- The user should not need to call this function. It should probably be private. The **hasRingAsserted** method will automatically call **checkRingIndicator** when using I2C.
- The Qwiic Iridium uses an interrupt to detect when the ring indicator signal goes low. Although the **hasRingAsserted** method will clear the flag, the flag can be cleared
manually using **clearRingIndicator**. This can be useful as the flag can be re-set by a second ring indication which the user may want to ignore.

---
```
void clearRingIndicator()
```

- Description: Clears the Qwiic Iridium Ring Indicator flag.
- Returns: None.
- Parameter: None.

Notes:
- The Qwiic Iridium uses an interrupt to detect when the ring indicator signal goes low. Although the **hasRingAsserted** method will clear the flag, the flag can be cleared
manually using **clearRingIndicator**. This can be useful as the flag can be re-set by a second ring indication which the user may want to ignore.

---
```
bool checkNetworkAvailable()
```

- Description: Checks the status of the 9603N's Network Available signal.
- Returns: **true** if the network is available, **false** if not.
- Parameter: None.

Notes:
- Network Available is true when the 9603N is able to receive the ring channel. It can take seconds, or sometimes up to two minutes, for NA to become true depending on the history of satellite visibility.
- The send message methods will automatically re-try if the network is not available. The user does not need to call **checkNetworkAvailable** before sending a message.

---
```
void enable841lowPower(bool enable)
```

- Description: Enables the Qwiic Iridium ATtiny841's low power mode to save power.
- Returns: None.
- Parameter: **enable** - **true** will enable the low power mode, **false** will disable it.

Notes:
- The Qwiic Iridium's current draw can be reduced to approximately 1 microamp when in low power mode, if the 9603N, super capacitor charger and power LED are disabled.
- The ATtiny841 will automatically wake up on I2C or serial activity or if a ring indicator interrupt is received. It is safe to leave the low power mode permanently enabled.

---
```
bool isConnected()
```

- Description: Checks if the Qwiic Iridium is connected.
- Returns: **true** if the Qwiic Iridium is connected, **false** if not.
- Parameter: None.

Notes:
- It can be useful to call this method to confirm the Qwiic Iridium is connected when starting an application. **checkSuperCapCharger** will never return **true** if the Qwiic Iridium is not connected.

---
```
int passThruI2Cwrite(uint8_t *txBuffer, size_t &txBufferSize)
```

- Description: Passes the binary data in **txBuffer** directly to the 9603N's serial pin without going through the higher level library methods.
- Returns: ISBD_SUCCESS if successful, a non-zero code otherwise.
- Parameter: **txBuffer** - The buffer containing the binary data to be transmitted.
- Parameter: **txBufferSize** - The size (length) of the _binary data_ in bytes (not the size of the buffer itself).

Notes:
- This method will return ISBD_REENTRANT if another method is already in progress and ISBD_SERIAL_FAILURE if the serial constructor is in use.

---
```
int passThruI2Cread(uint8_t *rxBuffer, size_t &rxBufferSize, size_t &numBytes)
```

- Description: Passes the binary data from the 9603N's serial pin directly to the user, via **rxBuffer**, without going through the higher level library methods.
- Returns: ISBD_SUCCESS if successful, a non-zero code otherwise.
- Parameter: **rxBuffer** - The buffer to receive the serial data from the 9603N.
- Parameter: **rxBufferSize** - The size (length) of the _buffer_ in bytes.
- Parameter: **numBytes** - The number of bytes returned in the buffer (>= 0, <= rxBufferSize).

Notes:
- This method will return ISBD_REENTRANT if another method is already in progress and ISBD_SERIAL_FAILURE if the serial constructor is in use.
- The method uses the **rxBufferSize** to prevent overflowing the buffer. If the 9603N returns more serial data than the buffer can hold, a ISBD_RX_OVERFLOW error is returned.

---
### Callbacks (optional)
---
```
bool ISBDCallback()
```

- Description: An optional user-supplied callback to help provide the appearance of responsiveness during lengthy Iridium operations
- Returns: **true** if the calling library operation should continue, **false** to terminate it.
- Parameter: **None**.

Notes:
- If this function is not provided the library methods will appear to block.
- This is not a library method, but an optional user-provided callback function.

---
```
void ISBDConsoleCallback(IridiumSBD *device, char c)
```

- Description: An optional user-supplied callback to sniff the conversation with the Iridium 9602/3.  
- Returns: **None**.
- Parameter: **device** - a handle to the modem device
- Parameter: **c** - a character in the conversation

Notes:
- Typical usage is to write c to a console for diagnostics.
- This is not a library method, but an optional user-provided callback function.

---
```
void ISBDDiagsCallback(IridiumSBD *device, char c)
```

- Description: An optional user-supplied callback to monitor the library's run state.
- Returns: **None**.
- Parameter: **device** - a handle to the modem device
- Parameter: **c** - a character in the run log

Notes:
- Typical usage is to write **c** to a console for diagnostics
- This is not a library method, but an optional user-provided callback function.
---

## License

This library is distributed under the terms of the GNU LGPL license.  

## Document revision history

| Version | Date | Author | Reason |
| --- | --- | --- | --- |
| 1.0 | 2013 | Mikal Hart | Initial draft submitted to Rock 7 for review |
| 1.1 | 2014 | Mikal Hart | Added text about the AT-MSSTM erratum/workaround and changing the minimum required signal quality. Also documented related new methods **setMinimumSignalQuality** and **useMSSTMWorkaround()**. |
| 2.0 | 21 October 2017 | Mikal Hart | Several API revisions. Removed **setMinimumSignalQuality** (no longer used), added support for RING monitoring (**enableRingAlerts** method, and new RING alert pin on constructor), and changed the way diagnostics are done by replacing the **attachConsole** and **attachDiags** methods with user-supplied callbacks **ISBDConsoleCallback** and **ISBDDiagsCallback**. Added getSystemTime and getFirmwareVersion utility functions.  Add explanation that MSSTM workaround is no longer enabled by default if firmware is sufficiently new (TA13001 or newer). |
| 3.0 | October 2019 | Paul Clark | Added I2C support for the Qwiic Iridium. Restructured the examples. Converted Mikal's documentation to markdown format. Added the feature requests (**getIMEI** and **clearBuffers**) and corrected the issues identified with version 2.0. Included a fix to let the serial Ring example work properly (**hasRingAsserted** now checks the ringPin itself instead of assuming **cancelled** will be able to do it). |
