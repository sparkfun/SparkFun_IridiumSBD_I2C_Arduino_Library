/*
IridiumSBD - An Arduino library for Iridium SBD ("Short Burst Data") Communications
Suggested and generously supported by Rock Seven Location Technology
(http://rock7mobile.com), makers of the brilliant RockBLOCK satellite modem.
Copyright (C) 2013-2017 Mikal Hart
All rights reserved.

Updated by Paul Clark to provide I2C support for the Qwiic Iridium 9603N

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <WString.h> // for FlashString
#include <Stream.h> // for Stream

#ifndef IRIDIUM_SBD // https://github.com/mikalhart/IridiumSBD/pull/14
#define IRIDIUM_SBD

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

//Platform specific configurations

//Define the size of the I2C buffer based on the platform the user has
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

//I2C_BUFFER_LENGTH is defined in Wire.H
#define I2C_BUFFER_LENGTH BUFFER_LENGTH

#elif defined(__SAMD21G18A__)

//SAMD21 uses RingBuffer.h
#define I2C_BUFFER_LENGTH SERIAL_BUFFER_SIZE

//#elif __MK20DX256__
//Teensy

#endif

#ifndef I2C_BUFFER_LENGTH

//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32

#endif
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#define ISBD_LIBRARY_REVISION           3 // Changed by Paul to reflect substantial update for I2C functions
#define ISBD_DEFAULT_AT_TIMEOUT         30
#define ISBD_MSSTM_RETRY_INTERVAL       10
#define ISBD_DEFAULT_SBDIX_INTERVAL     10
#define ISBD_DEFAULT_SBDSESSION_TIMEOUT 0
#define ISBD_USB_SBDIX_INTERVAL         30
#define ISBD_DEFAULT_SENDRECEIVE_TIME   300
#define ISBD_STARTUP_MAX_TIME           240
#define ISBD_MAX_MESSAGE_LENGTH         340
#define ISBD_MSSTM_WORKAROUND_FW_VER    13001

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

#define ISBD_CLEAR_MO			0
#define ISBD_CLEAR_MT			1
#define ISBD_CLEAR_BOTH			2

typedef const __FlashStringHelper *FlashString;

class IridiumSBD
{
public:
   int begin();
   int sendSBDText(const char *message);
   int sendSBDBinary(const uint8_t *txData, size_t txDataSize);
   int sendReceiveSBDText(const char *message, uint8_t *rxBuffer, size_t &rxBufferSize);
   int sendReceiveSBDBinary(const uint8_t *txData, size_t txDataSize, uint8_t *rxBuffer, size_t &rxBufferSize);
   int getSignalQuality(int &quality);
   int getSystemTime(struct tm &tm);
   int getFirmwareVersion(char *version, size_t bufferSize);
   int getWaitingMessageCount();
   bool isAsleep();
   bool hasRingAsserted();
   int sleep();

   typedef enum { DEFAULT_POWER_PROFILE = 0, USB_POWER_PROFILE = 1 } POWERPROFILE;
   void setPowerProfile(POWERPROFILE profile); // 0 = direct connect (default), 1 = USB
   void adjustATTimeout(int seconds);          // default value = 30 seconds
   int adjustSBDSessionTimeout(int seconds);   // 0 = infinite (default)
   void adjustSendReceiveTimeout(int seconds); // default value = 300 seconds
   void adjustStartupTimeout(int seconds); // default value = 240 seconds
   void useMSSTMWorkaround(bool useMSSTMWorkAround); // true to use workaround from Iridium Alert 5/7/13
   void enableRingAlerts(bool enable);

   int clearBuffers(int buffers = ISBD_CLEAR_MO);
   int getIMEI(char *IMEI, size_t bufferSize);

   // Functions for the Qwiic Iridium (only - not valid when using serial on the RockBLOCK)
   void enableSuperCapCharger(bool enable);
   bool checkSuperCapCharger();
   void enable9603Npower(bool enable);
   bool checkRingIndicator();
   void clearRingIndicator();
   bool checkNetworkAvailable();
   void enable9603(bool enable);
   void enable841lowPower(bool enable);
   bool isConnected();
   int passThruI2Cread(uint8_t *rxBuffer, size_t &rxBufferSize, size_t &numBytes);
   int passThruI2Cwrite(uint8_t *txBuffer, size_t &txBufferSize);

   // Weak functions to configure and set the sleep pin - user can overwrite with a custom functions if required
   void configureSleepPin() __attribute__((weak));
   void setSleepPin(uint8_t enable) __attribute__((weak));

   // Weak functions to begin and end the Serial port after power(true) and before power(false)
   void beginSerialPort() __attribute__((weak));
   void endSerialPort() __attribute__((weak));

   IridiumSBD(Stream &str, int sleepPinNo = -1, int ringPinNo = -1)
   {
      useSerial = true;
      stream = &str;
      sbdixInterval = ISBD_USB_SBDIX_INTERVAL;
      sbdSessionTimeout = ISBD_DEFAULT_SBDSESSION_TIMEOUT;
      atTimeout = ISBD_DEFAULT_AT_TIMEOUT;
      sendReceiveTimeout = ISBD_DEFAULT_SENDRECEIVE_TIME;
      startupTimeout = ISBD_STARTUP_MAX_TIME;
      remainingMessages = -1;
      asleep = true;
      reentrant = false;
      sleepPin = sleepPinNo;
      sleepPinConfigured = false;
      ringPin = ringPinNo;
      msstmWorkaroundRequested = true;
      ringAlertsEnabled = {ringPinNo != -1};
      ringAsserted = false;
      lastPowerOnTime = 0UL;
      head = SBDRING;
      tail = SBDRING;
      nextChar = -1;
      i2c_ser_buffer_tail = 0;
      i2c_ser_buffer_head = 0;
      if (sleepPin != -1)
         pinMode(sleepPin, OUTPUT);
      if (ringPin != -1)
         pinMode(ringPin, INPUT);
   }

   IridiumSBD(TwoWire &wirePort = Wire, uint8_t deviceAddress = 0x63)
   {
      useSerial = false;
      wireport = &wirePort;
      deviceaddress = deviceAddress;
      sbdixInterval = ISBD_USB_SBDIX_INTERVAL;
      sbdSessionTimeout = ISBD_DEFAULT_SBDSESSION_TIMEOUT;
      atTimeout = ISBD_DEFAULT_AT_TIMEOUT;
      sendReceiveTimeout = ISBD_DEFAULT_SENDRECEIVE_TIME;
      startupTimeout = ISBD_STARTUP_MAX_TIME;
      remainingMessages = -1;
      asleep = true;
      reentrant = false;
      sleepPin = -1;
      sleepPinConfigured = false;
      ringPin = -1;
      msstmWorkaroundRequested = false;
      ringAlertsEnabled = true;
      ringAsserted = false;
      lastPowerOnTime = 0UL;
      head = SBDRING;
      tail = SBDRING;
      nextChar = -1;
      i2c_ser_buffer_tail = 0;
      i2c_ser_buffer_head = 0;
   }

private:
   Stream * stream; // Communicating with the Iridium
   TwoWire  * wireport;
   uint8_t deviceaddress;
   bool useSerial;

   //Create the I2C_Serial buffer
   #define I2C_SER_MAX_BUFF 64 // RX buffer size
   char i2c_ser_buffer[I2C_SER_MAX_BUFF];
   int i2c_ser_buffer_tail;
   int i2c_ser_buffer_head;

   //Qwiic Iridium ATtiny841 I2C buffer length
   #define TINY_I2C_BUFFER_LENGTH 32

   //Define the maximum number of serial bytes to be requested from the ATtiny841
   #define SER_PACKET_SIZE 8

   // Timings
   int sbdixInterval;
   int sbdSessionTimeout;
   int atTimeout;
   int sendReceiveTimeout;
   int startupTimeout;
   unsigned long lastCheck = 0; // The time in millis when the I2C bus was last checked (limits I2C traffic)
   const unsigned long I2C_POLLING_WAIT_MS = 5; //Limit checking of new characters to every 5 ms (roughly 10 chars at 19200 baud)
   const unsigned long AT_RESPONSE_LOOP_DELAY = 2; //Resolves #19 - 2 ms (roughly 4 chars at 19200 baud)

   // State variables
   int remainingMessages;
   bool asleep;
   bool reentrant;
   int  sleepPin;
   bool sleepPinConfigured;
   int  ringPin;
   bool msstmWorkaroundRequested;
   bool ringAlertsEnabled;
   bool ringAsserted;
   unsigned long lastPowerOnTime;

   // Internal utilities
   bool noBlockWait(int seconds);
   bool waitForATResponse(char *response=NULL, int responseSize=0, const char *prompt=NULL, const char *terminator="OK\r\n");

   int  internalBegin();
   int  internalSendReceiveSBD(const char *txTxtMessage, const uint8_t *txData, size_t txDataSize, uint8_t *rxBuffer, size_t *prxBufferSize);
   int  internalGetSignalQuality(int &quality);
   int  internalMSSTMWorkaround(bool &okToProceed);
   int  internalSleep();

   int  doSBDIX(uint16_t &moCode, uint16_t &moMSN, uint16_t &mtCode, uint16_t &mtMSN, uint16_t &mtLen, uint16_t &mtRemaining);
   int  doSBDRB(uint8_t *rxBuffer, size_t *prxBufferSize); // in/out
   void power(bool on);

   void send(FlashString str, bool beginLine = true, bool endLine = true);
   void send(const char *str);
   void sendlong(const char *str);
   void send(uint16_t n);

   bool cancelled(); // call ISBDCallback and see if client cancelled the operation

   void diagprint(FlashString str);
   void diagprint(const char *str);
   void diagprint(uint16_t n);

   void consoleprint(FlashString str);
   void consoleprint(const char *str);
   void consoleprint(uint16_t n);
   void consoleprint(char c);
   void SBDRINGSeen();

   // Unsolicited SBDRING filter technology
   static const char SBDRING[];
   static const int FILTERTIMEOUT = 10;
   const char *head, *tail;
   int nextChar;
   void filterSBDRING();
   int filteredavailable();
   int filteredread();

   // I2C_SER buffer functions
   int i2cSerAvailable();
   int i2cSerRead();
   void i2cSerPoke(char serChar);

   // Read serial data from the 9603
   void check9603data();

   // Read the state of the Iridium pins and update IO_REGISTER
   void check9603pins();
   // Set the Iridium pins
   void set9603pins(uint8_t pins);

   //Define the I2C 'registers'
   #define IO_REG  0x10 // Read/write to/from the I/O pins
   #define LEN_REG 0xFD // The serial length regsiter: 2 bytes (MSB, LSB) indicating how many serial characters are available to be read
   #define DATA_REG 0xFF // The serial data register: used to read and write serial data from/to the 9603N

   //Create the IO 'register'
   //A '1' in any of the bits indicates that the pin is ON (not necessarily that it is HIGH!)
   byte IO_REGISTER;

   //These are the bit definitions for the IO 'register'
   const uint8_t IO_SHDN    = (1 << 0); // LTC3225 !SHDN : Read / Write
   const uint8_t IO_PWR_EN  = (1 << 1); // 9603N power enable via the P-FET : Read / Write
   const uint8_t IO_ON_OFF  = (1 << 2); // 9603N ON_OFF pin : Read / Write
   const uint8_t IO_RI      = (1 << 3); // 9603N Ring Indicator _flag_ : Read / Write (Set if RI has been seen, cleared by writing a 0 to this bit)
   const uint8_t IO_NA      = (1 << 4); // 9603N Network Available : Read only
   const uint8_t IO_PGOOD   = (1 << 5); // LTC3225 PGOOD : Read only
   const uint8_t IO_LOW_PWR = (1 << 6); // ATtiny841 low power mode : Read / Write : Set to enable low power mode

   // Clear the MO/MT/Both buffers
   int internalClearBuffers(int buffers = 0);

   // Get the IMEI
   // https://github.com/mikalhart/IridiumSBD/pull/21
   int internalGetIMEI(char *IMEI, size_t bufferSize);

   // Functions to support I2C to serial pass thru
   int internalPassThruI2Cread(uint8_t *rxBuffer, size_t &rxBufferSize, size_t &numBytes);
   int internalPassThruI2Cwrite(uint8_t *txBuffer, size_t &txBufferSize);
};

#endif
