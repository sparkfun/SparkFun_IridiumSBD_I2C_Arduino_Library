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

#include <time.h>
#include "IridiumSBD.h"


bool ISBDCallback() __attribute__((weak));
void ISBDConsoleCallback(IridiumSBD *device, char c) __attribute__((weak));
void ISBDDiagsCallback(IridiumSBD *device, char c) __attribute__((weak));

bool ISBDCallback() { return true; }
void ISBDConsoleCallback(IridiumSBD *device, char c) { }
void ISBDDiagsCallback(IridiumSBD *device, char c) { }

// Power on the RockBLOCK or return from sleep
int IridiumSBD::begin()
{
   if (this->reentrant)
      return ISBD_REENTRANT;

   this->reentrant = true;
   int ret = internalBegin();
   this->reentrant = false;

   // Absent a successful startup, keep the device turned off
   if (ret != ISBD_SUCCESS)
   {
      if (this->useSerial)
         endSerialPort(); // Apollo3 v2.1 Serial fix
      power(false);
   }

   return ret;
}

// Transmit a binary message
int IridiumSBD::sendSBDBinary(const uint8_t *txData, size_t txDataSize)
{
   if (this->reentrant)
      return ISBD_REENTRANT;

   this->reentrant = true;
   int ret = internalSendReceiveSBD(NULL, txData, txDataSize, NULL, NULL);
   this->reentrant = false;
   return ret;
}

// Transmit and receive a binary message
int IridiumSBD::sendReceiveSBDBinary(const uint8_t *txData, size_t txDataSize, uint8_t *rxBuffer, size_t &rxBufferSize)
{
   if (this->reentrant)
      return ISBD_REENTRANT;

   this->reentrant = true;
   int ret = internalSendReceiveSBD(NULL, txData, txDataSize, rxBuffer, &rxBufferSize);
   this->reentrant = false;
   return ret;
}

// Transmit a text message
int IridiumSBD::sendSBDText(const char *message)
{
   if (this->reentrant)
      return ISBD_REENTRANT;

   this->reentrant = true;
   int ret = internalSendReceiveSBD(message, NULL, 0, NULL, NULL);
   this->reentrant = false;
   return ret;
}

// Transmit a text message and receive reply
int IridiumSBD::sendReceiveSBDText(const char *message, uint8_t *rxBuffer, size_t &rxBufferSize)
{
   if (this->reentrant)
      return ISBD_REENTRANT;

   this->reentrant = true;
   int ret = internalSendReceiveSBD(message, NULL, 0, rxBuffer, &rxBufferSize);
   this->reentrant = false;
   return ret;
}

// High-level wrapper for AT+CSQ
int IridiumSBD::getSignalQuality(int &quality)
{
   if (this->reentrant)
      return ISBD_REENTRANT;

   this->reentrant = true;
   int ret = internalGetSignalQuality(quality);
   this->reentrant = false;
   return ret;
}

// Gracefully put device to lower power mode (if sleep pin provided)
int IridiumSBD::sleep()
{
   if (this->reentrant)
      return ISBD_REENTRANT;

   if (this->useSerial && (this->sleepPin == -1))
      return ISBD_NO_SLEEP_PIN;

   this->reentrant = true;
   int ret = internalSleep();
   this->reentrant = false;

   if (ret == ISBD_SUCCESS)
   {
      if (this->useSerial)
         endSerialPort(); // Apollo3 v2.1 Serial fix
      power(false); // power off
   }

   return ret;
}

// Return sleep state
bool IridiumSBD::isAsleep()
{
   return this->asleep;
}

// Return number of pending messages
int IridiumSBD::getWaitingMessageCount()
{
   return this->remainingMessages;
}

// Define capacitor recharge times
void IridiumSBD::setPowerProfile(POWERPROFILE profile) // 0 = direct connect (default), 1 = USB
{
   switch(profile)
   {
   case DEFAULT_POWER_PROFILE:
      this->sbdixInterval = ISBD_DEFAULT_SBDIX_INTERVAL;
      break;

   case USB_POWER_PROFILE:
      this->sbdixInterval = ISBD_USB_SBDIX_INTERVAL;
      break;
   }
}

// Tweak AT timeout
void IridiumSBD::adjustATTimeout(int seconds)
{
   this->atTimeout = seconds;
}

// Tweak SBD Session Timeout
int IridiumSBD::adjustSBDSessionTimeout(int seconds)
{
   if ((seconds >= this->atTimeout) || seconds == 0)
      diagprint(F("SBD commands that do not complete before AT timeout will return ISBD_PROTOCOL_ERROR\r\n"));

   if (!this->asleep)
   {
      send(F("AT+SBDST="), true, false);
      send(seconds);
      send(F("\r"), false);
      if (!waitForATResponse())
         return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;
   }
   this->sbdSessionTimeout = seconds;
   return ISBD_SUCCESS;
}

// Tweak Send/Receive SBDIX process timeout
void IridiumSBD::adjustSendReceiveTimeout(int seconds)
{
   this->sendReceiveTimeout = seconds;
}

// Tweak ISBD startup timeout
void IridiumSBD::adjustStartupTimeout(int seconds)
{
   this->startupTimeout = seconds;
}

void IridiumSBD::useMSSTMWorkaround(bool useWorkAround) // true to use workaround from Iridium Alert 5/7
{
   this->msstmWorkaroundRequested = useWorkAround;
}

void IridiumSBD::enableRingAlerts(bool enable) // true to enable SBDRING alerts and RING signal pin
{
   this->ringAlertsEnabled = enable;
   if (enable)
   {
      this->ringAsserted = false;
      if (!this->useSerial) // If we are using I2C, clear the ring indicator flag
      {
        clearRingIndicator();
      }
   }
}

bool IridiumSBD::hasRingAsserted()
// This should only be called occasionally as it will force an I2C transaction each time if we are using I2C
{
   if (!ringAlertsEnabled)
      return false;

   if (!reentrant)
   {
      // It's possible that the SBDRING message comes while we're not doing anything
      filterSBDRING();
   }

   bool ret = ringAsserted;
   this->ringAsserted = false;

   if (!this->useSerial) // If we are using I2C, check the RI flag now
   {
      if (checkRingIndicator()) // If the RI flag is set
      {
          ret = true; // Return true
		  diagprint(F("RI flag seen!\r\n"));
          clearRingIndicator(); // Clear the flag
      }
   }
   else // If we are using serial then let's check the ringPin manually instead of assuming cancelled() will be able to do it
   {
      if ((ringPin != -1) && digitalRead(ringPin) == LOW)
      {
	      ret = true; // Return true
	      //diagprint(F("ringPin seen!\r\n"));
      }
   }

   return ret;
}

int IridiumSBD::getSystemTime(struct tm &tm)
{
   char msstmResponseBuf[24];

   send(F("AT-MSSTM\r"));
   if (!waitForATResponse(msstmResponseBuf, sizeof(msstmResponseBuf), "-MSSTM: "))
      return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;

   if (!isxdigit(msstmResponseBuf[0]))
      return ISBD_NO_NETWORK;

   // From the AT Command Reference:
   //   The system time as received through the Iridium Air
   //   Interface, is a 32 bit integer count of the number of 90 millisecond
   //   intervals that have elapsed since the epoch. The return value is
   //   formatted as an ASCII hexadecimal number. The counter will rollover
   //   approximately every 12 years or be changed to prevent a rollover and
   //   as a result should not be used as a time source for user applications   

   // Current epoch: May 11, 2014, at 14:23:55 UTC.
   struct tm epoch_start_current;
   epoch_start_current.tm_year = 2014 - 1900;
   epoch_start_current.tm_mon = 5 - 1;
   epoch_start_current.tm_mday = 11;
   epoch_start_current.tm_hour = 14;
   epoch_start_current.tm_min = 23;
   epoch_start_current.tm_sec = 55;
   epoch_start_current.tm_isdst = 0; // Resolves #16

   // Future epoch: February 14, 2025, at 18:14:17 UTC.
   struct tm epoch_start_future;
   epoch_start_future.tm_year = 2025 - 1900;
   epoch_start_future.tm_mon = 2 - 1;
   epoch_start_future.tm_mday = 14;
   epoch_start_future.tm_hour = 18;
   epoch_start_future.tm_min = 14;
   epoch_start_future.tm_sec = 17;
   epoch_start_future.tm_isdst = 0; // Resolves #16

   unsigned long ticks_since_epoch = strtoul(msstmResponseBuf, NULL, 16);

   /* Strategy: we'll convert to seconds by finding the largest number of integral
      seconds less than the equivalent ticks_since_epoch. Subtract that away and
      we'll be left with a small number that won't overflow when we scale by 90/1000.

      Many thanks to Scott Weldon for this suggestion.
   */
   unsigned long secs_since_epoch = (ticks_since_epoch / 1000) * 90;
   unsigned long small_ticks = ticks_since_epoch - (secs_since_epoch / 90) * 1000;
   secs_since_epoch += small_ticks * 90 / 1000;

   time_t epoch_time = mktime(&epoch_start_current); // Try current epoch first
   time_t now = epoch_time + secs_since_epoch;
   struct tm * local_now = localtime(&now);

   // Extract the current year from __DATE__
   int build_year = (((int)(__DATE__[7] - 0x30) * 1000) + ((int)(__DATE__[8] - 0x30) * 100)
                   + ((int)(__DATE__[9] - 0x30) * 10)  + ((int)(__DATE__[10] - 0x30) * 1));

   // Check if year is within bounds
   if ((local_now->tm_year + 1900) < build_year)
   {
      epoch_time = mktime(&epoch_start_future); // Use future epoch
      now = epoch_time + secs_since_epoch;
   }

   memcpy(&tm, localtime(&now), sizeof tm);

   return ISBD_SUCCESS;
}

int IridiumSBD::getFirmwareVersion(char *version, size_t bufferSize)
{
   if (bufferSize < 8)
      return ISBD_RX_OVERFLOW;

   send(F("AT+CGMR\r"));
   if (!waitForATResponse(version, bufferSize, "Call Processor Version: "))
      return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;

   return ISBD_SUCCESS;
}

void IridiumSBD::enableSuperCapCharger(bool enable)
{
  if (useSerial) // Do nothing if we are using serial (the user will have to enable the charger manually)
  {
    diagprint(F("enableSuperCapCharger is only valid when using I2C on the Qwiic Iridium\r\n"));
    return;
  }

  // Enable/disable the supercapacitor charger by pulling its SHDN pin high/low
  check9603pins(); // Update IO_REGISTER
  if (enable)
  {
    IO_REGISTER |= IO_SHDN; // Set the SHDN bit
  }
  else
  {
    IO_REGISTER &= ~IO_SHDN; // Clear the SHDN bit
  }
  set9603pins(IO_REGISTER); // Update the pins
}

bool IridiumSBD::checkSuperCapCharger()
{
  if (useSerial) // Do nothing if we are using serial (the user will have to check PGOOD manually)
  {
    diagprint(F("checkSuperCapCharger is only valid when using I2C on the Qwiic Iridium\r\n"));
    return(false);
  }

  // Check the status of the supercapacitor charger PGOOD pin
  check9603pins(); // Update IO_REGISTER
  if (IO_REGISTER &= IO_PGOOD) // If the PGOOD bit is set, return true
  {
    return(true);
  }
  else
  {
    return(false);
  }
}

void IridiumSBD::enable9603Npower(bool enable)
{
  if (useSerial) // Do nothing if we are using serial (the user will have to enable the 9603N power manually)
  {
    diagprint(F("enable9603Npower is only valid when using I2C on the Qwiic Iridium\r\n"));
    return;
  }

  // Enable/disable power to the 9603N by pulling PWR_EN high/low
  check9603pins(); // Update IO_REGISTER
  if (enable)
  {
    IO_REGISTER |= IO_PWR_EN; // Set the PWR_EN bit
  }
  else
  {
    IO_REGISTER &= ~IO_PWR_EN; // Clear the PWR_EN bit
  }
  set9603pins(IO_REGISTER); // Update the pins
}

void IridiumSBD::enable9603(bool enable)
{
  if (useSerial) // Do nothing if we are using serial (the user will have to enable the 9603N manually)
  {
    diagprint(F("enable9603 is only valid when using I2C on the Qwiic Iridium\r\n"));
    return;
  }

  // Enable/disable the 9603 by pulling ON_OFF high/low
  check9603pins(); // Update IO_REGISTER
  if (enable)
  {
    IO_REGISTER |= IO_ON_OFF; // Set the ON_OFF bit
  }
  else
  {
    IO_REGISTER &= ~IO_ON_OFF; // Clear the ON_OFF bit
  }
  set9603pins(IO_REGISTER); // Update the pins
}

void IridiumSBD::enable841lowPower(bool enable)
{
  if (useSerial) // Do nothing if we are using serial
  {
    diagprint(F("enable841lowPower is only valid when using I2C on the Qwiic Iridium\r\n"));
    return;
  }

	// Enable/disable the Qwiic Iridium ATtiny841's low power mode
	check9603pins(); // Update IO_REGISTER
	if (enable)
	{
		IO_REGISTER |= IO_LOW_PWR; // Set the LOW_PWR bit
	}
	else
	{
		IO_REGISTER &= ~IO_LOW_PWR; // Clear the LOW_PWR bit
	}
	set9603pins(IO_REGISTER); // Update the pins
}

bool IridiumSBD::checkRingIndicator()
{
  if (useSerial) // Do nothing if we are using serial (the user will have to use hasRingAsserted instead)
  {
    diagprint(F("checkRingIndicator is only valid when using I2C on the Qwiic Iridium\r\n"));
    return(false);
  }

  // Check the status of the 9603 Ring Indicator flag
  check9603pins(); // Update IO_REGISTER
  if (IO_REGISTER &= IO_RI) // If the RI bit is set, return true
  {
    return(true);
  }
  else
  {
    return(false);
  }
}

void IridiumSBD::clearRingIndicator()
{
  if (useSerial) // Do nothing if we are using serial
  {
    diagprint(F("clearRingIndicator is only valid when using I2C on the Qwiic Iridium\r\n"));
    return;
  }

  // Clear the 9603 RI flag
  check9603pins(); // Update IO_REGISTER
  IO_REGISTER &= ~IO_RI; // Clear the RI bit
  set9603pins(IO_REGISTER); // Update the pins
  this->ringAsserted = false; // Also clear the ringAsserted flag
}

bool IridiumSBD::checkNetworkAvailable()
{
  if (useSerial) // Do nothing if we are using serial
  {
    diagprint(F("checkNetworkAvailable is only valid when using I2C on the Qwiic Iridium\r\n"));
    return(false);
  }

  // Check the status of the 9603 Network Available pin
  check9603pins(); // Update IO_REGISTER
  if (IO_REGISTER &= IO_NA) // If the NA bit is set, return true
  {
    return(true);
  }
  else
  {
    return(false);
  }
}

// High-level wrapper for AT+SBDD
int IridiumSBD::clearBuffers(int buffers)
{
   if (this->reentrant)
      return ISBD_REENTRANT;

   this->reentrant = true;
   int ret = internalClearBuffers(buffers);
   this->reentrant = false;
   return ret;
}

// High-level wrapper for AT+CGSN
int IridiumSBD::getIMEI(char *IMEI, size_t bufferSize)
{
   if (this->reentrant)
      return ISBD_REENTRANT;

   this->reentrant = true;
   int ret = internalGetIMEI(IMEI, bufferSize);
   this->reentrant = false;
   return ret;
}

//Returns true if the I2C device is connected
//Always returns true for serial
bool IridiumSBD::isConnected()
{
   if (this->useSerial) // If we are using Serial
   {
		return true;
   }
   else
   {
		wireport->beginTransmission((uint8_t)deviceaddress);
		return (wireport->endTransmission() == 0); // Check that the device ack's
   }
}

// High-level wrapper for passThruI2Cread
int IridiumSBD::passThruI2Cread(uint8_t *rxBuffer, size_t &rxBufferSize, size_t &numBytes)
// rxBuffer is a pointer to the receive buffer which will store the read serial data
// rxBufferSize is the size of the receive buffer (so we don't overflow it)
// On return, numBytes contains the number of bytes written into the rxBuffer (<= rxBufferSize)
// If there was too much data for the rxBuffer, the function will return an ISBD_RX_OVERFLOW error
{
  if (useSerial) // Do nothing if we are using serial
  {
    diagprint(F("passThruI2Cread is only valid when using I2C on the Qwiic Iridium\r\n"));
    return(0);
  }

	if (this->reentrant)
	return ISBD_REENTRANT;

	this->reentrant = true;
	int ret = internalPassThruI2Cread(rxBuffer, rxBufferSize, numBytes);
	this->reentrant = false;
	return ret;
}

// High-level wrapper for passThruI2Cwrite
int IridiumSBD::passThruI2Cwrite(uint8_t *txBuffer, size_t &txBufferSize)
// txBuffer is a pointer to the transmit buffer which contains the serial data to be written
// txBufferSize is the number of bytes to be written
{
  if (useSerial) // Do nothing if we are using serial
  {
    diagprint(F("passThruI2Cwrite is only valid when using I2C on the Qwiic Iridium\r\n"));
    return(0);
  }

	if (this->reentrant)
	return ISBD_REENTRANT;

	this->reentrant = true;
	int ret = internalPassThruI2Cwrite(txBuffer, txBufferSize);
	this->reentrant = false;
	return ret;
}

/*
Private interface
*/

int IridiumSBD::internalBegin()
{
   diagprint(F("Calling internalBegin\r\n"));

   if (!this->asleep)
      return ISBD_ALREADY_AWAKE;

   if (!this->useSerial) // If we are using I2C
   {
      check9603pins(); // Update IO_REGISTER with the status of the 9603 pins
      check9603data(); // Get any waiting 9603 serial data
   }

   power(true); // power on

   bool modemAlive = false;

   unsigned long startupTime = 500; //ms
   for (unsigned long start = millis(); millis() - start < startupTime;)
      if (cancelled())
         return ISBD_CANCELLED;

   if (this->useSerial) // If we are using Serial
      beginSerialPort(); // Apollo3 v2.1 Serial fix - begin the Serial port 500ms after power(true)

   // Turn on modem and wait for a response from "AT" command to begin
   for (unsigned long start = millis(); !modemAlive && millis() - start < 1000UL * this->startupTimeout;)
   {
      send(F("AT\r"));
      modemAlive = waitForATResponse();
      if (cancelled())
         return ISBD_CANCELLED;
   }

   if (!modemAlive)
   {
      diagprint(F("No modem detected.\r\n"));
      return ISBD_NO_MODEM_DETECTED;
   }

   // The usual initialization sequence
   const char *strings[3] = { "ATE1\r", "AT&D0\r", "AT&K0\r" };
   for (int i=0; i<3; ++i)
   {
      send(strings[i]);
      if (!waitForATResponse())
         return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;
   }

   // Enable or disable RING alerts as requested by user
   // By default they are on if a RING pin was supplied on constructor
   diagprint(F("Ring alerts are")); diagprint(ringAlertsEnabled ? F("") : F(" NOT")); diagprint(F(" enabled.\r\n"));

   if (ringAlertsEnabled) enableRingAlerts(true); // This will clear ringAsserted and the Ring Indicator flag
   else {
	   if (!this->useSerial) clearRingIndicator(); // If ring alerts are not enabled and using I2C, make sure the Ring Indicator flag is clear
   }

   send(ringAlertsEnabled ? F("AT+SBDMTA=1\r") : F("AT+SBDMTA=0\r"));
   if (!waitForATResponse())
      return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;

   // Decide whether the internal MSSTM workaround should be enforced on TX/RX
   // By default it is unless the firmware rev is >= TA13001
   char version[8];
   int ret = getFirmwareVersion(version, sizeof(version));
   if (ret != ISBD_SUCCESS)
   {
      diagprint(F("Unknown FW version\r\n"));
      msstmWorkaroundRequested = true;
   }
   else
   {
      diagprint(F("Firmware version is ")); diagprint(version); diagprint(F("\r\n"));
      if (version[0] == 'T' && version[1] == 'A')
      {
         unsigned long ver = strtoul(version + 2, NULL, 10);
         msstmWorkaroundRequested = ver < ISBD_MSSTM_WORKAROUND_FW_VER;
      }
   }
   diagprint(F("MSSTM workaround is")); diagprint(msstmWorkaroundRequested ? F("") : F(" NOT")); diagprint(F(" enforced.\r\n"));

   // Set SBD session timeout only if it has been changed from the default (to avoid regressions)
   // ISU AT Command Reference: "The <timeout> setting is stored only while the SBD Modem is powered up, and defaults to zero (meaning infinite timeout) after a power-cycle."
   if (this->sbdSessionTimeout != ISBD_DEFAULT_SBDSESSION_TIMEOUT)
   {
      ret = adjustSBDSessionTimeout(this->sbdSessionTimeout);
      if (ret != ISBD_SUCCESS)
      {
         diagprint(F("adjustSBDSessionTimeout: failed\r\n"));
      }
      else
      {
         diagprint(F("adjustSBDSessionTimeout: success!\r\n"));
      }
   }

   // Done!
   diagprint(F("InternalBegin: success!\r\n"));
   return ISBD_SUCCESS;
}

int IridiumSBD::internalSendReceiveSBD(const char *txTxtMessage, const uint8_t *txData, size_t txDataSize, uint8_t *rxBuffer, size_t *prxBufferSize)
{
   diagprint(F("internalSendReceive\r\n"));

   if (this->asleep)
      return ISBD_IS_ASLEEP;

   // Binary transmission?
   if (txData && txDataSize)
   {
      if (txDataSize > ISBD_MAX_MESSAGE_LENGTH)
         return ISBD_MSG_TOO_LONG;

      // send will use serial or wire as appropriate
      send(F("AT+SBDWB="), true, false);
      send(txDataSize);
      send(F("\r"), false);
      if (!waitForATResponse(NULL, 0, NULL, "READY\r\n"))
         return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;

      uint16_t checksum = 0;

      if (this->useSerial)
      {
         for (size_t i=0; i<txDataSize; ++i)
         {
            stream->write(txData[i]);
            checksum += (uint16_t)txData[i];
         }
         stream->write((uint8_t)(checksum >> 8));
         stream->write((uint8_t)(checksum & 0xFF));
      }
      else
      {
         //lastCheck = millis(); // Update lastCheck so we enforce a full I2C_POLLING_WAIT
         // We need to make sure we don't send too much I2C data in one go (otherwise we will overflow the ATtiny841's I2C buffer)
         size_t bytes_to_send = txDataSize; // Send this many bytes in total
         size_t i=0;
         size_t nexti;
         while (bytes_to_send > (TINY_I2C_BUFFER_LENGTH - 3)) // If there are too many bytes to send all in one go
         {
            nexti = i + (TINY_I2C_BUFFER_LENGTH - 3);
            wireport->beginTransmission((uint8_t)deviceaddress);
            wireport->write(DATA_REG); // Point to the serial data 'register'
            for (; i<nexti; ++i)
            {
               wireport->write(txData[i]); // Write each byte
               checksum += (uint16_t)txData[i];
            }
            bytes_to_send = bytes_to_send - (TINY_I2C_BUFFER_LENGTH - 3); // Decrease the number of bytes still to send
            wireport->endTransmission(); // Send data and release the bus (the 841 (WireS) doesn't like it if the Master holds the bus!)
         }
         // There are now <= (TINY_I2C_BUFFER_LENGTH - 3) bytes left to send, so send them and then release the bus
         wireport->beginTransmission((uint8_t)deviceaddress);
         wireport->write(DATA_REG); // Point to the 'serial register'
         for (; i<txDataSize; ++i)
         {
            wireport->write(txData[i]);
            checksum += (uint16_t)txData[i];
         }
         wireport->write((uint8_t)(checksum >> 8));
         wireport->write((uint8_t)(checksum & 0xFF));
         if (wireport->endTransmission() != 0) //Send data and release bus
            diagprint(F("I2C write was not successful!\r\n"));
      }

      consoleprint(F("["));
      consoleprint((uint16_t)txDataSize);
      consoleprint(F(" bytes]"));

      diagprint(F("Checksum:"));
      diagprint(checksum);
      diagprint(F("\r\n"));

      if (!waitForATResponse(NULL, 0, NULL, "0\r\n\r\nOK\r\n"))
         return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;
   }

   else // Text transmission
   {
#if true // use long string implementation
      if (txTxtMessage == NULL) // It's ok to have a NULL txtTxtMessage if the transaction is RX only
      {
         send(F("AT+SBDWT=\r"));
         if (!waitForATResponse())
            return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;
      }
      else
      {
         // remove any embedded \r
         char *p = strchr(txTxtMessage, '\r');
         if (p) *p = 0;
         if (strlen(txTxtMessage) > ISBD_MAX_MESSAGE_LENGTH)
            return ISBD_MSG_TOO_LONG;
         send(F("AT+SBDWT\r"));
         if (!waitForATResponse(NULL, 0, NULL, "READY\r\n"))
            return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;
         sendlong(txTxtMessage);
         send("\r");
         if (!waitForATResponse(NULL, 0, NULL, "0\r\n\r\nOK\r\n"))
            return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;
      }
#else
      send(F("AT+SBDWT="), true, false);
      if (txTxtMessage != NULL) // It's ok to have a NULL txtTxtMessage if the transaction is RX only
         sendlong(txTxtMessage);
      send(F("\r"), false);
      if (!waitForATResponse())
         return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;
#endif
   }

   // Long SBDIX loop begins here
   for (unsigned long start = millis(); millis() - start < 1000UL * this->sendReceiveTimeout;)
   {
      bool okToProceed = true;
      if (this->msstmWorkaroundRequested)
      {
         okToProceed = false;
         int ret = internalMSSTMWorkaround(okToProceed);
         if (ret != ISBD_SUCCESS)
            return ret;
      }

      if (okToProceed)
      {
         uint16_t moCode = 0, moMSN = 0, mtCode = 0, mtMSN = 0, mtLen = 0, mtRemaining = 0;
         int ret = doSBDIX(moCode, moMSN, mtCode, mtMSN, mtLen, mtRemaining);
         if (ret != ISBD_SUCCESS)
            return ret;

         diagprint(F("SBDIX MO code: "));
         diagprint(moCode);
         diagprint(F("\r\n"));

         if (moCode <= 4) // this range indicates successful return!
         {
            diagprint(F("SBDIX success!\r\n"));

            this->remainingMessages = mtRemaining;
            if (mtCode == 1 && rxBuffer) // retrieved 1 message
            {
               diagprint(F("Incoming message!\r\n"));
               return doSBDRB(rxBuffer, prxBufferSize);
            }

            else
            {
               // No data returned
               if (prxBufferSize)
                  *prxBufferSize = 0;
            }
            return ISBD_SUCCESS;
         }

         else if (moCode == 12 || moCode == 14 || moCode == 16) // fatal failure: no retry
         {
            diagprint(F("SBDIX fatal!\r\n"));
            return ISBD_SBDIX_FATAL_ERROR;
         }

         else // retry
         {
            diagprint(F("Waiting for SBDIX retry...\r\n"));
            if (!noBlockWait(sbdixInterval))
               return ISBD_CANCELLED;
         }
      }

      else // MSSTM check fail
      {
         diagprint(F("Waiting for MSSTM retry...\r\n"));
         if (!noBlockWait(ISBD_MSSTM_RETRY_INTERVAL))
            return ISBD_CANCELLED;
      }
   } // big wait loop

   diagprint(F("SBDIX timeout!\r\n"));
   return ISBD_SENDRECEIVE_TIMEOUT;
}

int IridiumSBD::internalGetSignalQuality(int &quality)
{
   if (this->asleep)
      return ISBD_IS_ASLEEP;

   char csqResponseBuf[2];

   send(F("AT+CSQ\r"));
   if (!waitForATResponse(csqResponseBuf, sizeof(csqResponseBuf), "+CSQ:"))
      return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;

   if (isdigit(csqResponseBuf[0]))
   {
      quality = atoi(csqResponseBuf);
      return ISBD_SUCCESS;
   }

   return ISBD_PROTOCOL_ERROR;
}

int IridiumSBD::internalMSSTMWorkaround(bool &okToProceed)
{
   /*
   According to Iridium 9602 Product Bulletin of 7 May 2013, to overcome a system erratum:

   "Before attempting any of the following commands: +SBDDET, +SBDREG, +SBDI, +SBDIX, +SBDIXA the field application
   should issue the AT command AT-MSSTM to the transceiver and evaluate the response to determine if it is valid or not:

   Valid Response: "-MSSTM: XXXXXXXX" where XXXXXXXX is an eight-digit hexadecimal number.

   Invalid Response: "-MSSTM: no network service"

   If the response is invalid, the field application should wait and recheck system time until a valid response is
   obtained before proceeding.

   This will ensure that the Iridium SBD transceiver has received a valid system time before attempting SBD communication.
   The Iridium SBD transceiver will receive the valid system time from the Iridium network when it has a good link to the
   satellite. Ensuring that the received signal strength reported in response to AT command +CSQ and +CIER is above 2-3 bars
   before attempting SBD communication will protect against lockout.
   */
   char msstmResponseBuf[24];

   send(F("AT-MSSTM\r"));
   if (!waitForATResponse(msstmResponseBuf, sizeof(msstmResponseBuf), "-MSSTM: "))
      return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;

   // Response buf now contains either an 8-digit number or the string "no network service"
   okToProceed = isxdigit(msstmResponseBuf[0]);
   return ISBD_SUCCESS;
}

int IridiumSBD::internalSleep()
{
   if (this->asleep)
      return ISBD_IS_ASLEEP;

#if false // recent research suggest this is not what you should do when just sleeping
   // Best Practices Guide suggests this before shutdown
   send(F("AT*F\r"));

   if (!waitForATResponse())
      return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;
#endif

   return ISBD_SUCCESS;
}

bool IridiumSBD::noBlockWait(int seconds)
{
   for (unsigned long start=millis(); millis() - start < 1000UL * seconds;)
      if (cancelled())
         return false;

   return true;
}

// Wait for response from previous AT command.  This process terminates when "terminator" string is seen or upon timeout.
// If "prompt" string is provided (example "+CSQ:"), then all characters following prompt up to the next CRLF are
// stored in response buffer for later parsing by caller.
bool IridiumSBD::waitForATResponse(char *response, int responseSize, const char *prompt, const char *terminator)
{
   diagprint(F("Waiting for response "));
   diagprint(terminator);
   diagprint(F("\r\n"));

   if (response)
      memset(response, 0, responseSize);

   int matchPromptPos = 0; // Matches chars in prompt
   int matchTerminatorPos = 0; // Matches chars in terminator
   enum {LOOKING_FOR_PROMPT, GATHERING_RESPONSE, LOOKING_FOR_TERMINATOR};
   int promptState = prompt ? LOOKING_FOR_PROMPT : LOOKING_FOR_TERMINATOR;
   consoleprint(F("<< "));
   for (unsigned long start=millis(); millis() - start < 1000UL * atTimeout;)
   {
      if (cancelled())
         return false;

      while (filteredavailable() > 0)
      {
         char c = filteredread();
         if (prompt)
         {
            switch (promptState)
            {
            case LOOKING_FOR_PROMPT:
               if (c == prompt[matchPromptPos])
               {
                  ++matchPromptPos;
                  if (prompt[matchPromptPos] == '\0')
                     promptState = GATHERING_RESPONSE;
               }

               else
               {
                  matchPromptPos = c == prompt[0] ? 1 : 0;
               }

               break;
            case GATHERING_RESPONSE: // gathering response from end of prompt to first \r
               if (response)
               {
                  if (c == '\r' || responseSize < 2)
                  {
                     promptState = LOOKING_FOR_TERMINATOR;
                  }
                  else
                  {
                     *response++ = c;
                     responseSize--;
                  }
               }
               break;
            }
         }

         if (c == terminator[matchTerminatorPos])
         {
            ++matchTerminatorPos;
            if (terminator[matchTerminatorPos] == '\0')
               return true;
         }
         else
         {
            matchTerminatorPos = c == terminator[0] ? 1 : 0;
         }
      } // while (filteredavailable() > 0)

      delay(AT_RESPONSE_LOOP_DELAY); // Resolve #19
   } // timer loop
   return false;
}

bool IridiumSBD::cancelled()
{
   if (this->useSerial)
   {
      if ((ringPin != -1) && digitalRead(ringPin) == LOW)
	  {
         ringAsserted = true;
		 //diagprint(F("ringPin seen!\r\n"));
	  }
   }

   return !ISBDCallback();
}

int IridiumSBD::doSBDIX(uint16_t &moCode, uint16_t &moMSN, uint16_t &mtCode, uint16_t &mtMSN, uint16_t &mtLen, uint16_t &mtRemaining)
{
   // Returns xx,xxxxx,xx,xxxxx,xx,xxx
   char sbdixResponseBuf[32];
   send(F("AT+SBDIX\r"));
   if (!waitForATResponse(sbdixResponseBuf, sizeof(sbdixResponseBuf), "+SBDIX: "))
      return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;

   uint16_t *values[6] = { &moCode, &moMSN, &mtCode, &mtMSN, &mtLen, &mtRemaining };
   for (int i=0; i<6; ++i)
   {
      char *p = strtok(i == 0 ? sbdixResponseBuf : NULL, ", ");
      if (p == NULL)
         return ISBD_PROTOCOL_ERROR;
      *values[i] = atol(p);
   }
   return ISBD_SUCCESS;
}

int IridiumSBD::doSBDRB(uint8_t *rxBuffer, size_t *prxBufferSize)
{
   bool rxOverflow = false;

   send(F("AT+SBDRB\r"));
   if (!waitForATResponse(NULL, 0, NULL, "AT+SBDRB\r")) // waits for its own echo
      return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;

   if(!this->useSerial) check9603data(); // Check for any 9603 serial data

   // Time to read the binary data: size[2], body[size], checksum[2]
   unsigned long start = millis();
   while (millis() - start < 1000UL * atTimeout)
   {
      if(!this->useSerial) check9603data(); // Keep checking for new 9603 serial data
      if (cancelled())
         return ISBD_CANCELLED;
      if (this->useSerial && (stream->available() >= 2))
         break;
      if ((!this->useSerial) && (i2cSerAvailable() >= 2))
         break;
   }

   if (this->useSerial && (stream->available() < 2))
      return ISBD_SENDRECEIVE_TIMEOUT;
   if ((!this->useSerial) && (i2cSerAvailable() < 2))
      return ISBD_SENDRECEIVE_TIMEOUT;

   uint16_t size;
   if (this->useSerial)
   {
      size = 256 * stream->read() + stream->read();
   }
   else
   {
      size = 256 * i2cSerRead() + i2cSerRead();
   }
   consoleprint(F("[Binary size:"));
   consoleprint(size);
   consoleprint(F("]"));

   for (uint16_t bytesRead = 0; bytesRead < size;)
   {
      if (cancelled())
         return ISBD_CANCELLED;

      if(!this->useSerial) check9603data(); // Check for new 9603 serial data

      if ((this->useSerial && (stream->available())) || ((!this->useSerial) && i2cSerAvailable()))
      {
         uint8_t c;
         if (this->useSerial)
         {
            c = stream->read();
         }
         else
         {
            c = i2cSerRead();
         }
         bytesRead++;
         if (rxBuffer && prxBufferSize)
         {
            if (*prxBufferSize > 0)
            {
               *rxBuffer++ = c;
               (*prxBufferSize)--;
            }
            else
            {
               rxOverflow = true;
            }
         }
      }

      if (millis() - start >= 1000UL * atTimeout)
         return ISBD_SENDRECEIVE_TIMEOUT;
   }

   while (millis() - start < 1000UL * atTimeout)
   {
      if(!this->useSerial) check9603data(); // Check for new 9603 serial data
      if (cancelled())
         return ISBD_CANCELLED;
      if (this->useSerial && (stream->available() >= 2))
         break;
      if ((!this->useSerial) && (i2cSerAvailable() >= 2))
         break;
   }

   if (this->useSerial && (stream->available() < 2))
      return ISBD_SENDRECEIVE_TIMEOUT;
   if ((!this->useSerial) && (i2cSerAvailable() < 2))
      return ISBD_SENDRECEIVE_TIMEOUT;

   uint16_t checksum;
   if (this->useSerial)
   {
      checksum = 256 * stream->read() + stream->read();
   }
   else
   {
      checksum = 256 * i2cSerRead() + i2cSerRead();
   }
   consoleprint(F("[csum:"));
   consoleprint(checksum);
   consoleprint(F("]"));

   // Return actual size of returned buffer
   if (prxBufferSize)
      *prxBufferSize = (size_t)size;

   // Wait for final OK
   if (!waitForATResponse())
      return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;

   return rxOverflow ? ISBD_RX_OVERFLOW : ISBD_SUCCESS;
}

void IridiumSBD::power(bool on)
{
   this->asleep = !on;

   if (this->useSerial)
   {
      if (this->sleepPin == -1)
      {
         return;
      }
      else
      {
          if (this->sleepPinConfigured == false)
          {
             configureSleepPin();
             this->sleepPinConfigured = true;
          }
      }
   }

   if (on)
   {
      diagprint(F("Powering on modem...\r\n"));
      if (this->useSerial)
      {
         setSleepPin(HIGH); // HIGH = awake
      }
      else
      {
         enable9603(true);
      }
      lastPowerOnTime = millis();
   }
   else
   {
      // Best Practices Guide suggests waiting at least 2 seconds
      // before powering off again
      unsigned long elapsed = millis() - lastPowerOnTime;
      if (elapsed < 2000UL)
         delay(2000UL - elapsed);

      diagprint(F("Powering off modem...\r\n"));
      if (this->useSerial)
      {
         setSleepPin(LOW); // LOW = asleep
      }
      else
      {
         enable9603(false);
      }
   }
}

void IridiumSBD::configureSleepPin()
{
   pinMode(this->sleepPin, OUTPUT); // Make the sleep pin an output
   diagprint(F("configureSleepPin: sleepPin configured\r\n"));
}

void IridiumSBD::setSleepPin(uint8_t enable)
{
   digitalWrite(this->sleepPin, enable); // HIGH = awake, LOW = asleep
   diagprint(F("setSleepPin: sleepPin set "));
   if (enable == HIGH)
      diagprint(F("HIGH\r\n"));
   else
      diagprint(F("LOW\r\n"));
}

void IridiumSBD::beginSerialPort()
{
   diagprint(F("IridiumSBD::beginSerialPort\r\n"));
}

void IridiumSBD::endSerialPort()
{
   diagprint(F("IridiumSBD::endSerialPort\r\n"));
}

void IridiumSBD::send(FlashString str, bool beginLine, bool endLine)
{
   if (beginLine)
      consoleprint(F(">> "));
   consoleprint(str);
   if (endLine)
      consoleprint(F("\r\n"));
   if (this->useSerial)
   {
      stream->print(str);
   }
   else
   {
      //lastCheck = millis(); // Update lastCheck so we enforce a full I2C_POLLING_WAIT
      wireport->beginTransmission((uint8_t)deviceaddress);
      wireport->write(DATA_REG); // Point to the 'serial register'
      wireport->print(str);
      if (wireport->endTransmission() != 0) //Release bus
         diagprint(F("I2C write was not successful!\r\n"));
   }
}

void IridiumSBD::send(const char *str)
{
   consoleprint(F(">> "));
   consoleprint(str);
   consoleprint(F("\r\n"));
   if (this->useSerial)
   {
      stream->print(str);
   }
   else
   {
      //lastCheck = millis(); // Update lastCheck so we enforce a full I2C_POLLING_WAIT
      wireport->beginTransmission((uint8_t)deviceaddress);
      wireport->write(DATA_REG); // Point to the 'serial register'
      wireport->print(str);
      if (wireport->endTransmission() != 0) //Release bus
         diagprint(F("I2C write was not successful!\r\n"));
   }
}

void IridiumSBD::sendlong(const char *str)
// Send a long string that might need to be broken up for the I2C port
{
   consoleprint(F(">> "));
   consoleprint(str);
   consoleprint(F("\r\n"));

   if (this->useSerial)
   {
      stream->print(str); // If we are using serial then send it and don't worry about the long length
   }
   else
   {
      //lastCheck = millis(); // Update lastCheck so we enforce a full I2C_POLLING_WAIT
      // We need to make sure we don't send too much I2C data in one go (otherwise we will overflow the buffer)
      size_t bytes_to_send = strlen(str); // Send this many bytes in total
      size_t txDataSize = bytes_to_send;
      size_t i=0;
      size_t nexti;
      while (bytes_to_send > (TINY_I2C_BUFFER_LENGTH - 1)) // If there are too many bytes to send all in one go
      {
         nexti = i + (TINY_I2C_BUFFER_LENGTH - 1);
         wireport->beginTransmission((uint8_t)deviceaddress);
         wireport->write(DATA_REG); // Point to the 'serial register'
         for (; i<nexti; ++i)
         {
            wireport->write(str[i]); // Write each byte
         }
         bytes_to_send = bytes_to_send - (TINY_I2C_BUFFER_LENGTH - 1); // Decrease the number of bytes still to send
         wireport->endTransmission(); // Send data and release the bus (the 841 (WireS) doesn't like it if the Master holds the bus!)
      }
      // There are now <= (TINY_I2C_BUFFER_LENGTH - 1) bytes left to send, so send them and then release the bus
      wireport->beginTransmission((uint8_t)deviceaddress);
      wireport->write(DATA_REG); // Point to the 'serial register'
      for (; i<txDataSize; ++i)
      {
         wireport->write(str[i]);
      }
      if (wireport->endTransmission() != 0) //Send data and release bus
         diagprint(F("I2C write was not successful!\r\n"));
   }
}

void IridiumSBD::send(uint16_t n)
{
   consoleprint(n);
   if (this->useSerial)
   {
      stream->print(n);
   }
   else
   {
      //lastCheck = millis(); // Update lastCheck so we enforce a full I2C_POLLING_WAIT
      wireport->beginTransmission((uint8_t)deviceaddress);
      wireport->write(DATA_REG); // Point to the 'serial register'
      wireport->print(n);
      if (wireport->endTransmission() != 0) //Send data and release bus
         diagprint(F("I2C write was not successful!\r\n"));
   }
}

void IridiumSBD::diagprint(FlashString str)
{
   PGM_P p = reinterpret_cast<PGM_P>(str);
   while (1)
   {
      char c = pgm_read_byte(p++);
      if (c == 0) break;
      ISBDDiagsCallback(this, c);
   }
}

void IridiumSBD::diagprint(const char *str)
{
   while (*str)
      ISBDDiagsCallback(this, *str++);
}

void IridiumSBD::diagprint(uint16_t n)
{
   char str[10];
   sprintf(str, "%u", n);
   diagprint(str);
}

void IridiumSBD::consoleprint(FlashString str)
{
   PGM_P p = reinterpret_cast<PGM_P>(str);
   while (1)
   {
      char c = pgm_read_byte(p++);
      if (c == 0) break;
      ISBDConsoleCallback(this, c);
   }
}

void IridiumSBD::consoleprint(const char *str)
{
   while (*str)
      ISBDConsoleCallback(this, *str++);
}

void IridiumSBD::consoleprint(uint16_t n)
{
   char str[10];
   sprintf(str, "%u", n);
   consoleprint(str);
}

void IridiumSBD::consoleprint(char c)
{
   ISBDConsoleCallback(this, c);
}

void IridiumSBD::SBDRINGSeen()
{
   ringAsserted = true;
   diagprint(F("SBDRING alert seen!\r\n"));
}

// Read characters until we find one that doesn't match SBDRING
// If nextChar is -1 it means we are still entertaining a possible
// match with SBDRING\r\n.  Once we find a mismatch, stuff it into
// nextChar.
void IridiumSBD::filterSBDRING()
{
   if(!this->useSerial) check9603data(); // Check for new 9603 serial data
   while (((this->useSerial && (stream->available() > 0)) || ((!this->useSerial) && (i2cSerAvailable() > 0))) && nextChar == -1)
   {
      char c;
      if (this->useSerial)
      {
         c = stream->read();
      }
      else
      {
         c = i2cSerRead();
      }
      consoleprint(c);
      if (*head != 0 && c == *head)
      {
         ++head;
         if (*head == 0)
         {
            SBDRINGSeen();
            head = tail = SBDRING;
         }
         else
         {
            // Delay no more than 10 milliseconds waiting for next char in SBDRING
            for (unsigned long start = millis(); ((this->useSerial && (stream->available() == 0)) || ((!this->useSerial) && (i2cSerAvailable() == 0))) && millis() - start < FILTERTIMEOUT; );

            if(!this->useSerial) check9603data(); // Check for new 9603 serial data

            // If there isn't one, assume this ISN'T an unsolicited SBDRING
            if ((this->useSerial && (stream->available() == 0)) || ((!this->useSerial) && (i2cSerAvailable() == 0))) // pop the character back into nextChar
            {
               --head;
               nextChar = c;
            }
         }
      }
      else
      {
         nextChar = c;
      }
   }
}

const char IridiumSBD::SBDRING[] = "SBDRING\r\n";

int IridiumSBD::filteredavailable()
{
   filterSBDRING();
   return head - tail + (nextChar != -1 ? 1 : 0);
}

int IridiumSBD::filteredread()
{
   filterSBDRING();

   // Use up the queue first
   if (head > tail)
   {
      char c = *tail++;
      if (head == tail)
         head = tail = SBDRING;
      return c;
   }

   // Then the "extra" char
   else if (nextChar != -1)
   {
      char c = (char)nextChar;
      nextChar = -1;
      return c;
   }

   return -1;
}

//Checks the number of available serial bytes
//Reads the available serial bytes (if any) and stores them in i2c_ser_buffer
void IridiumSBD::check9603data()
{
  if (millis() - lastCheck >= I2C_POLLING_WAIT_MS)
  {
    //Check how many serial bytes are waiting to be read
    uint16_t bytesAvailable = 0;
    wireport->beginTransmission((uint8_t)deviceaddress); // Talk to the I2C device
    wireport->write(LEN_REG); // Point to the serial buffer length
    wireport->endTransmission(); // Send data and release the bus (the 841 (WireS) doesn't like it if the Master holds the bus!)
    if (wireport->requestFrom((uint8_t)deviceaddress, (uint8_t)2) == 2) // Request two bytes
    {
      uint8_t msb = wireport->read();
      uint8_t lsb = wireport->read();
      bytesAvailable = (((uint16_t)msb) << 8) | lsb;
    }

    //Now read the serial bytes (if any)
    if (bytesAvailable > 0)
    {
      // Request the bytes
      // Poke them into the i2c_serial buffer
      // Release the bus afterwards
      wireport->beginTransmission((uint8_t)deviceaddress); // Talk to the I2C device
      wireport->write(DATA_REG); // Point to the serial buffer
      wireport->endTransmission(); // Send data and release the bus (the 841 (WireS) doesn't like it if the Master holds the bus!)
      while (bytesAvailable > SER_PACKET_SIZE) // If there are _more_ than SER_PACKET_SIZE bytes to be read
      {
        wireport->requestFrom((uint8_t)deviceaddress, (uint8_t)SER_PACKET_SIZE, (uint8_t)false); // Request SER_PACKET_SIZE bytes, don't release the bus
        while (wireport->available())
        {
          i2cSerPoke(wireport->read()); // Read and store each byte
        }
        bytesAvailable -= SER_PACKET_SIZE; // Decrease the number of bytes available by SER_PACKET_SIZE
      }
      wireport->requestFrom((uint8_t)deviceaddress, (uint8_t)bytesAvailable); // Request remaining bytes, release the bus
      while (wireport->available())
      {
        i2cSerPoke(wireport->read()); // Read and store each byte
      }
    }

    lastCheck = millis(); //Put off checking to avoid excessive I2C bus traffic
  }
}

//Reads the IO pins and update IO_REGISTER
void IridiumSBD::check9603pins()
{
  //Read the 'IO_REGISTER'
  wireport->beginTransmission((uint8_t)deviceaddress); // Talk to the I2C device
  wireport->write(IO_REG); // Point to the 'IO register'
  wireport->endTransmission(); // Send data and release the bus (the 841 (WireS) doesn't like it if the Master holds the bus!)
  if (wireport->requestFrom((uint8_t)deviceaddress, (uint8_t)1) == 1) // Request one byte from the IO register
  {
    IO_REGISTER = wireport->read(); // Read the IO register
  }
}

//Set the IO pins
void IridiumSBD::set9603pins(uint8_t pins)
{
  //Write to the 'IO_REGISTER'
  wireport->beginTransmission((uint8_t)deviceaddress); // Talk to the I2C device
  wireport->write(IO_REG); // Point to the 'IO register'
  wireport->write(pins); // Set the pins
  wireport->endTransmission(); // Send data and surrender the bus
}

// Functions to support I2C to serial pass thru
int IridiumSBD::internalPassThruI2Cread(uint8_t *rxBuffer, size_t &rxBufferSize, size_t &numBytes)
{
  if (this->asleep)
    return ISBD_IS_ASLEEP;

  if (this->useSerial)
    return ISBD_SERIAL_FAILURE; // This function is for I2C only

  //Check how many serial bytes are waiting to be read
  uint16_t bytesAvailable = 0;
  wireport->beginTransmission((uint8_t)deviceaddress); // Talk to the I2C device
  wireport->write(LEN_REG); // Point to the serial buffer length
  wireport->endTransmission(); // Send data and release the bus (the 841 (WireS) doesn't like it if the Master holds the bus!)
  if (wireport->requestFrom((uint8_t)deviceaddress, (uint8_t)2) == 2) // Request two bytes
  {
    uint8_t msb = wireport->read();
    uint8_t lsb = wireport->read();
    bytesAvailable = (((uint16_t)msb) << 8) | lsb;
  }

  numBytes = (size_t)bytesAvailable; //Store bytesAvailable in numBytes

  size_t bufferPtr = 0; // Initialise buffer pointer

  //Now read the serial bytes (if any)
  if (bytesAvailable > 0)
  {
    // Request the bytes
    // Poke them into rxBuffer
    // Release the bus afterwards
    wireport->beginTransmission((uint8_t)deviceaddress); // Talk to the I2C device
    wireport->write(DATA_REG); // Point to the serial buffer
    wireport->endTransmission(); // Send data and release the bus (the 841 (WireS) doesn't like it if the Master holds the bus!)
    while (bytesAvailable > SER_PACKET_SIZE) // If there are _more_ than SER_PACKET_SIZE bytes to be read
    {
      wireport->requestFrom((uint8_t)deviceaddress, (uint8_t)SER_PACKET_SIZE, (uint8_t)false); // Request SER_PACKET_SIZE bytes, don't release the bus
      while (wireport->available())
      {
        uint8_t dbyte = wireport->read(); // Read a byte
        if (bufferPtr < rxBufferSize) // If storing the byte won't overflow the buffer
		{
		  rxBuffer[bufferPtr] = dbyte; // Store the byte
		  bufferPtr++; // Increment the pointer
		}
      }
      bytesAvailable -= SER_PACKET_SIZE; // Decrease the number of bytes available by SER_PACKET_SIZE
    }
    wireport->requestFrom((uint8_t)deviceaddress, (uint8_t)bytesAvailable); // Request remaining bytes, release the bus
    while (wireport->available())
    {
      uint8_t dbyte = wireport->read(); // Read a byte
      if (bufferPtr < rxBufferSize) // If storing the byte won't overflow the buffer
      {
        rxBuffer[bufferPtr] = dbyte; // Store the byte
        bufferPtr++; // Increment the pointer
      }
    }
  }

  //If there were more bytes available than rxBuffer could hold, return ISBD_RX_OVERFLOW
  if (numBytes > bufferPtr)
  {
	  diagprint(F("rxBuffer is too small to hold all available data!\r\n"));
	  numBytes = bufferPtr;
	  return (ISBD_RX_OVERFLOW);
  }
  else
  {
	  return(ISBD_SUCCESS);
  }
}

int IridiumSBD::internalPassThruI2Cwrite(uint8_t *txData, size_t &txDataSize)
{
  if (this->asleep)
    return ISBD_IS_ASLEEP;

  if (this->useSerial)
    return ISBD_SERIAL_FAILURE; // This function is for I2C only

  // We need to make sure we don't send too much I2C data in one go (otherwise we will overflow the ATtiny841's I2C buffer)
  size_t bytes_to_send = txDataSize; // Send this many bytes in total
  size_t i=0;
  size_t nexti;
  while (bytes_to_send > (TINY_I2C_BUFFER_LENGTH - 1)) // If there are too many bytes to send all in one go
  {
    nexti = i + (TINY_I2C_BUFFER_LENGTH - 1);
    wireport->beginTransmission((uint8_t)deviceaddress);
    wireport->write(DATA_REG); // Point to the serial data 'register'
    for (; i<nexti; ++i)
    {
      wireport->write(txData[i]); // Write each byte
    }
    bytes_to_send = bytes_to_send - (TINY_I2C_BUFFER_LENGTH - 1); // Decrease the number of bytes still to send
    wireport->endTransmission(); // Send data and release the bus (the 841 (WireS) doesn't like it if the Master holds the bus!)
  }
  // There are now <= (TINY_I2C_BUFFER_LENGTH - 1) bytes left to send, so send them and then release the bus
  wireport->beginTransmission((uint8_t)deviceaddress);
  wireport->write(DATA_REG); // Point to the 'serial register'
  for (; i<txDataSize; ++i)
  {
    wireport->write(txData[i]);
  }
  if (wireport->endTransmission() != 0) //Send data and release bus
  {
    diagprint(F("I2C write was not successful!\r\n"));
	return(ISBD_PROTOCOL_ERROR);
  }
  else
    return(ISBD_SUCCESS);
}

// I2C_SER functions
int IridiumSBD::i2cSerAvailable()
{
  return (i2c_ser_buffer_tail + I2C_SER_MAX_BUFF - i2c_ser_buffer_head) % I2C_SER_MAX_BUFF;
}

int IridiumSBD::i2cSerRead()
{
  // Empty buffer?
  if (i2c_ser_buffer_head == i2c_ser_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = i2c_ser_buffer[i2c_ser_buffer_head]; // grab next byte
  i2c_ser_buffer_head = (i2c_ser_buffer_head + 1) % I2C_SER_MAX_BUFF; // update head
  return d;
}

void IridiumSBD::i2cSerPoke(char serChar)
{
  // Calculate the new value for the tail
  int next = (i2c_ser_buffer_tail + 1) % I2C_SER_MAX_BUFF;
  // If the buffer is not full (i.e. we are not about to overwrite the head byte)
  // If the buffer is full, the byte is lost
  if (next != i2c_ser_buffer_head)
  {
    // save new data in buffer: tail points to where byte goes
    i2c_ser_buffer[i2c_ser_buffer_tail] = serChar; // save new byte
    i2c_ser_buffer_tail = next;
  }
}

int IridiumSBD::internalClearBuffers(int buffers)
// Clear the MO/MT/Both buffers
// Defaults to clearing the MO buffer to avoid resending old messages
{
   if (this->asleep)
      return ISBD_IS_ASLEEP;

   if (buffers == ISBD_CLEAR_MT) // Clear MT buffer
   {
      send(F("AT+SBDD1\r"));
   }
   else if (buffers == ISBD_CLEAR_BOTH) // Clear both buffers
   {
      send(F("AT+SBDD2\r"));
   }
   else // Clear MO buffer
   {
      send(F("AT+SBDD0\r"));
   }
   if (!waitForATResponse())
      return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;

   return ISBD_SUCCESS;
}

int IridiumSBD::internalGetIMEI(char *IMEI, size_t bufferSize)
// Get the IMEI
// https://github.com/mikalhart/IridiumSBD/pull/21
{
   if (this->asleep)
      return ISBD_IS_ASLEEP;

   if (bufferSize < 16) // IMEI is 15 digits
      return ISBD_RX_OVERFLOW;

   send(F("AT+CGSN\r"));
   if (!waitForATResponse(IMEI, bufferSize, "\n"))
      return cancelled() ? ISBD_CANCELLED : ISBD_PROTOCOL_ERROR;

   return ISBD_SUCCESS;
}
