/*
 *  @filename   :   BLE_Accelerometer_Central.ino
 *  @brief      :   This software example creates a BLE central that
 *                  pairs with the BLE_Accelerometer peripheral to:
 *                  1. use NOTIFY to receive Acceleration and Gyro data
 *                  2. use READ or WRITE to receive or send Acceleration/Gyro sample rate. 
 *
 *  @hardware   :   Arduino Nano 33 BLE (nRF52840) + built-in Arduino LSM9DS1 sensor
 *  
 *  @author     :   Sazamore
 *
 *  Created 05 Feb 2025 
source:
https://community.element14.com/challenges-projects/project14/nano-rama/b/blog/posts/hatching-a-hockey-stick-hack-in-a-hurry
*/

#include <ArduinoBLE.h>
#include "IEEE11073float.h"

void setup() {
   Serial.begin(9600);
  while (!Serial);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  Serial.println("BLE Accelerometer Central");

  // start scanning for peripheral
  BLE.scan();   
  // Buggy but fast:
  //BLE.scanForUuid("19B10010-E8F2-537E-4F6C-D104768A1214");
}

void loop() {
  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // print the local name, if present
    if (peripheral.hasLocalName()) {
      if (peripheral.localName()!="AccelPeriph"){
        return;
      }
    }
    // stop scanning
    BLE.stopScan();

    readLed(peripheral); //pull & convert data

    // peripheral disconnected, start scanning again
    BLE.scan();
    //BLE.scanForUuid("19B10010-E8F2-537E-4F6C-D104768A1214");
  } 
}

void readLed(BLEDevice peripheral) {
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
    //TODO: turn off LED
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  BLECharacteristic moveCharacteristic = peripheral.characteristic("19B10011-E8F2-537E-4F6C-D104768A1214");

  if (!ledCharacteristic) {
    Serial.println("Peripheral does not have LED characteristic!");
    peripheral.disconnect();
    return;
  } else if (!ledCharacteristic.canWrite()) {
    Serial.println("Peripheral does not have a writable LED characteristic!");
    peripheral.disconnect();
    return;
  }

  while (peripheral.connected()) {
    // while the peripheral is connected

    // read the button pin
    int buttonState = digitalRead(buttonPin);

    if (oldButtonState != buttonState) {
      // button changed
      oldButtonState = buttonState;

      if (buttonState) {
        Serial.println("button pressed");

        // button is pressed, write 0x01 to turn the LED on
        ledCharacteristic.writeValue((byte)0x01);
      } else {
        Serial.println("button released");

        // button is released, write 0x00 to turn the LED off
        ledCharacteristic.writeValue((byte)0x00);
      }
    }
  }

  Serial.println("Peripheral disconnected");
}

float IEEE11073_2float(uint8_t *dat)
{
  int32_t Mantissa = (dat[2] << 16 | dat[1] << 8 | dat[0]);
  uint8_t Neg = bitRead(dat[2],7);
  int8_t fExp = dat[3];
  if (Neg) Mantissa |= 255 << 24;
  return (float(Mantissa) * pow(10, fExp));
}
