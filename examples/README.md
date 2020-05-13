# IridiumSBD Arduino Library Examples

This folder contains:
- **Serial examples:** these are enhanced versions of the examples Mikal Hart wrote for Version 2.0 of this library. They will work with the Rock7 RockBLOCK, connected via Serial
- **I2C examples:** these are the same examples plus new ones which are written for the Qwiic Iridium, connected via I2C

### Example 1: Get IMEI

This example opens a connection to the Iridium transceiver and requests its IMEI (serial number). It will check if your transceiver is connected correctly and that the
serial port is working. You do not need line rental or message credits to run this example.

The Qwiic Iridium I2C example: enables the super capacitor charger; waits for the capacitors to charge (this could take approximately 20 seconds if the capacitors are
fully discharged); enables power for the 9603N; gets the IMEI; puts the 9603N to sleep and powers everything down again.

Iridium transceivers bought from Rock7 have two serial numbers: the International Mobile Equipment Identity is a 15 digit serial number which identifies the transceiver
on the Iridium network; a five digit RockBLOCK serial number which is used to access the transceiver via the RockBLOCK Gateway. Example 10 uses the RockBLOCK serial number
(not the IMEI) to loop a message back to the transceiver.

### Example 2: Check CSQ

This example builds on example 1 but also checks the Iridium signal quality. It will check if your transceiver's antenna is connected correctly. You do not need line
rental or message credits to run this example.

The Qwiic Iridium I2C example also checks if the transceiver can receive the ring channel by checking the status of the Network Available (NA) signal.

You will notice that the signal quality is not returned immediately. It takes the transceiver a few seconds to measure the signal and send a reply.

### Example 3: Get Time

This example builds on example 2 but also gets the Iridium system time. The time is always returned in GMT as Iridium does not support time zones or daylight saving.
You do not need line rental or message credits to run this example.

### Example 4: Basic Send

This example sends a Mobile Originated (MO) "Hello, world!" test message. You will need line rental and message credits to run this example. Each message will
use one message credit. You should also set up a delivery group for your transceiver so you can receive the message via email (or HTTP POST).

### Example 5: Sleep

This example builds on example 4 but sends the test message once per minute, putting the transceiver to sleep between sends.

_Be careful not to leave this example running for too long as it will use one message credit each time a message is sent._

### Example 6: Send Receive

This example demonstrates how to receive Mobile Terminated (MT) messages sent _to_ the transceiver. In Rock7 Operations, you can use the _Send a Message_ option to
send a text message to your transceiver. Multiple messages will be queued in the Iridium system until the transceiver downloads them (one at a time). The messages
will expire if not downloaded within 48 hours.

During each Short Burst Data session, the transceiver can send one Mobile Originated message and in return will receive the first Mobile Terminated message from
the queue (if there is one). If you only want to check for MT messages, you can do this by sending an empty (NULL) MO message.

The first time around the loop, the code sends a _binary_ message (the values sent are the start of the Fibonacci sequence). The MT message, if there is one,
will be downloaded and displayed in the console. The code checks the length of the MT queue and will keep going around the loop sending empty (NULL) messages and
downloading MT messages until the queue is empty. In the Rock7 Operations message log, you will see these as _\[No payload\]_ Mobile Originated messages.

Rock7 will charge you one message credit for every 50 bytes (or part thereof) of a message sent or received. Sending a NULL message will still use one credit even
if there is nothing in the MT queue to be downloaded.

### Example 7: Ring

This example demonstrates how to use the transceiver Ring Indicator to show when a new MT message is waiting to be downloaded.

The code will enable the tranceiver, wait until it receives a ring indication and then download any messages in the queue. Start the code and wait until it says
_Begin waiting for RING..._. Now send a new MT message from Rock7 Operations. After a few seconds you will see the code acknowledge the ring alert and it will
begin downloading and displaying all the messages in the MT queue.

The Iridium system needs to know your approximate location to send you ring alerts. This means you will need to have transmited at least one message before this example
will work successfully.

### Example 8: Beacon

This example demonstrates how to create a beacon or tracker that will transmit your GNSS (GPS) location via Iridium from _anywhere_.

The serial example assumes you have a GNSS receiver connected via serial. It uses [Mikal's TinyGPS++ library](https://github.com/mikalhart/TinyGPSPlus) to parse the
NMEA messages and extract your location before transmitting it via Iridium. You can install TinyGPS++ through the Arduino IDE Library Manager (search for _TinyGPS++_).

The I2C example assumes you have a GNSS receiver connected via I2C. Good choices are the [SparkFun SAM-M8Q Breakout](https://www.sparkfun.com/products/15210) and the
[SparkFun ZOE-M8Q Breakout](https://www.sparkfun.com/products/15193). The example uses the [SparkFun u-blox library](https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library)
to obtain your location.

The code will send your location every 10 minutes. You can change the interval by editing the value for _BEACON_INTERVAL_.

### Example 9: Pass Thru

This example is for advanced users who want to send commands directly to the transceiver to investigate how the AT command set works. Commands typed into the serial
console are passed through to the transceiver and the replies are passed back. You can find example commands in the code plus a link to the full command reference.

Set the serial monitor Baud rate to 115200 and the line ending to carriage return.

### Example 10: Loop Back

The Rock7 RockBLOCK Gateway can be used to automatically forward messages to other RockBLOCKs _registered to the same account_. This example takes advantage of that
by sending a message that asks the Gateway to loop the message back again to the same transceiver. It does this by prefixing the message with the letters "RB"
and the RockBLOCK serial number of the transceiver padded out to seven digits. E.g. if the RockBLOCK serial number of your transceiver is _12345_ the message will
be prefixed by _RB0012345_

The RockBLOCK Gateway is a really powerful feature as it means you can send messages from one RockBLOCK to another _anywhere_ without needing an internet connection.
Please be aware that this does mean you will be charged for the message _twice_; once to transmit it and once to receive it.

### Example 11: Low Power

This example demonstrates how to put the Qwiic Iridium into low power mode. With the 9603N, super capacitor charger and power LED disabled, the Qwiic Iridium will draw
approximately 1 microamp when in low power mode. The ATtiny841 processor will wake up normally when it next sees activity on the I2C bus or the serial interface or
receives an interrupt from the Ring Indicator signal.
