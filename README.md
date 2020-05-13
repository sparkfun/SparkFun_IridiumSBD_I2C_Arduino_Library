# SparkFun IridiumSBD I2C Arduino Library

- This library can be installed through the Arduino Library Manager: please search for **IridiumSBDi2c**

- _Iridium SBD_ is Mikal's original library which does not (currently) support I2C.

The Iridium 9602 and 9603N are fascinating communications modules that give devices like Arduino or Raspberry Pi&trade; access to the Iridium satellite network.
This is a big deal, because it means that your application can now easily and inexpensively communicate from any point on or above the globe,
from the heart of the Amazon to the Siberian tundra, from the Arctic to the Antarctic.
This library, **IridiumSBD**, uses Iridium's **SBD** ("Short Burst Data") protocol to send and receive short messages to/from the Iridium hub.
SBD is a "text message"-like technology that supports the transmission of text or binary messages up to a certain maximum size (270 bytes received, 340 bytes transmitted).

Breakout boards for the 9602 and 9603N are available in different formats. Most of these breakouts use serial (UART) interfacing but **I2C (Qwiic)** is possible too and this version of
the library supports both.

Grateful thanks go to:

- [Mikal Hart](https://github.com/mikalhart) for writing the original versions of **IridiumSBD** and for **TinyGPSPlus**

## Repository Contents

- **/documentation** - Full documentation for the library (.md).
- **/examples** - Example sketches for the library (.ino). Run these from the Arduino IDE. They demonstrate how to use both serial and I2C interfaces.
- **/src** - Source files for the library (.cpp, .h).
- **keywords.txt** - Keywords from this library that will be highlighted in the Arduino IDE.
- **library.properties** - General library properties for the Arduino package manager.

## Documentation

You can find documentation for the library in the [documentation folder](documentation)

## Products That Use This Library

- [WRL-13745](https://www.sparkfun.com/products/13745) - The original Rock7 RockBLOCK, available from your friends at SparkFun.
- [WRL-14498](https://www.sparkfun.com/products/14498) - The Rock7 RockBLOCK 9603, available from your friends at SparkFun.
- [SPX-16394](https://www.sparkfun.com/products/16394) - Qwiic Iridium 9603N: provides I2C support instead of serial, allowing you to add Iridium SBD communication to any Qwiic or I2C project.
- [SPX-16469](https://www.sparkfun.com/products/16469) - Artemis Global Tracker: combining the Iridium 9603N, u-blox ZOE-M8Q and TE MS8607 into a single compact tracker.

## History

[Versions 1 and 2](https://github.com/mikalhart/IridiumSBD) of this library were written by Mikal Hart with generous support from [Rock 7 Mobile](http://rock7mobile.com).

This version is based on Version 2.0 of IridiumSBD but has been modified to provide I2C support for the Qwiic Iridium 9603N. Serial will still work too of course.
New [examples](examples) demonstrate how to use the I2C interface. The serial examples have also been restructured and enhanced.

This version of the library contains a new function called _clearBuffers_ which can be used to clear the Mobile Originated or Mobile Terminated message buffers.
This allows you to overcome the 'feature' (bug?) where the 9603N will automatically re-transmit the last MO message when checking for new MT messages.
This closes Issues [#10](https://github.com/mikalhart/IridiumSBD/issues/10) and [#11](https://github.com/mikalhart/IridiumSBD/issues/11) in Mikal's repo.

- _clearBuffers(ISBD_CLEAR_MO)_ will clear the MO buffer (default)
- _clearBuffers(ISBD_CLEAR_MT)_ will clear the MT buffer
- _clearBuffers(ISBD_CLEAR_BOTH)_ will clear both buffers

The library also includes Pull Requests [#14 Get IMEI](https://github.com/mikalhart/IridiumSBD/pull/14) and [#21 header import guards](https://github.com/mikalhart/IridiumSBD/pull/21).
There are new serial and I2C examples for _getIMEI_.

Also included is a correction for Issue [#12 weak diagnostics](https://github.com/mikalhart/IridiumSBD/issues/12).

Also includes a fix to let the serial Ring example work properly.

## License

Like versions 1 and 2 of Mikal's library, this version is also distributed under a
[GNU Lesser General Public Licence v2.1](LICENSE.md).
