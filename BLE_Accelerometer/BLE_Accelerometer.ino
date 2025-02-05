/**
 *  @filename   :   Element14_NanoRama_Nano33BLE_HinaH_HockeyStick.ino
 *  @brief      :   This software example creates a BLE peripheral with 
 *                  a custom service that contains 2 custom characteristics 
 *                  1. Central to use NOTIFY to receive Acceleration and Gyro data
 *                  2. Central to use READ to receive Acceleration/Gyro sample rate. They are made to be the same
 *                     Central can also use WRITE to change the sampling rate.
 *
 *  @hardware   :   Arduino Nano 33 BLE (nRF52840) + built-in Arduino LSM9DS1 sensor
 *  
 *  @author     :   Gerrikoio
 *
 *  Copyright (C) Gerrikoio for Application     17 May 2020
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documnetation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to  whom the Software is
 * furished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.

 Source:
 https://community.element14.com/challenges-projects/project14/nano-rama/b/blog/posts/hatching-a-hockey-stick-hack-in-a-hurry
 */


#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>
#include "IEEE11073float.h"

#define DEBUG

uint16_t SampleRateHz = 0;

BLEService MovementService("19B10010-E8F2-537E-4F6C-D104768A1214");   // create our custom GATT service

// create movement data characteristic (this is a Byte Array) and allow remote device to use Notify
BLECharacteristic moveDataCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 24, true);

// create acceleration & gyro sample rate (Hz) characteristic and allow remote device to get notifications
BLEShortCharacteristic SampleRateCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void setup() {
  #ifdef DEBUG
  Serial.begin(38400);
  while (!Serial);
  #endif
  
  //pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, HIGH);
  
  // begin BLE module initialization
  if (!BLE.begin()) {
    #ifdef DEBUG
    Serial.println("starting BLE failed!");
    #endif
    while (1);
  }

  // begin IMU module initialization
  if (!IMU.begin()) {
    #ifdef DEBUG
    Serial.println("Failed to initialize IMU!");
    #endif
    while (1);
  }

  SampleRateHz = (uint16_t)IMU.accelerationSampleRate();

  if (SampleRateHz != (uint16_t)IMU.gyroscopeSampleRate()) {
    #ifdef DEBUG
    Serial.print(F("Gyroscope sample is not the same as Accelerometer sample rate = "));
    Serial.print(SampleRateHz);
    Serial.println(" Hz");
    #endif
  }
  else {
    #ifdef DEBUG
    Serial.print(F("The Acceleration & Gyro sample rate is "));
    Serial.print(SampleRateHz);
    Serial.println(" Hz");
    #endif
  }
  #ifdef DEBUG
  Serial.println();
  #endif
  
  // set the local name peripheral advertises
  BLE.setLocalName("HaniH2020");
  // set the UUID for the service this peripheral advertises:
  BLE.setAdvertisedService(MovementService);

  // add the characteristics to the service
  MovementService.addCharacteristic(moveDataCharacteristic);
  MovementService.addCharacteristic(SampleRateCharacteristic);

  // add the GATT service
  BLE.addService(MovementService);

  // Create a temporary array for the movement data
  uint8_t mArray[24];
  memset(mArray, '\0', 24);
  moveDataCharacteristic.writeValue(mArray, 24);

  SampleRateCharacteristic.writeValue(SampleRateHz);

  // start advertising
  BLE.advertise();

  #ifdef DEBUG
  Serial.println("Bluetooth device active, waiting for connections...");
  #endif
}

void loop() {
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    #ifdef DEBUG
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    #endif

    // check the battery level every 200ms
    // while the central is connected:
    while (central.connected()) {
      // Update the BLE data
      updateMovementData();
    }
    #ifdef DEBUG
    // when the central disconnects, turn off the LED:
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    #endif
  }
}

void updateMovementData() {
  /* Read the current data from the Arduino LSM9DS1 sensor.
  */
  float Mvmnt[6];
  memset(Mvmnt, '\0', 6);
  uint8_t BLE_mArray[24];
  memset(BLE_mArray, '\0', 24);


  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(Mvmnt[0], Mvmnt[1], Mvmnt[2]);
    #ifdef DEBUG
    //Serial.print("Ax ");
    Serial.print(Mvmnt[0]);
    Serial.print(", ");
    Serial.print(Mvmnt[1]);
    Serial.print(", ");
    Serial.print(Mvmnt[2]);
    #endif
  }
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(Mvmnt[3], Mvmnt[4], Mvmnt[5]);
    #ifdef DEBUG
    Serial.print(", ");
    Serial.print(Mvmnt[3]);
    Serial.print(", ");
    Serial.print(Mvmnt[4]);
    Serial.print(", ");
    Serial.println(Mvmnt[5]);
    #endif
  }
  #ifdef DEBUG
  Serial.flush();
  #endif
  float2IEEE11073(double(Mvmnt[0]), &BLE_mArray[0]);
  float2IEEE11073(double(Mvmnt[1]), &BLE_mArray[4]);
  float2IEEE11073(double(Mvmnt[2]), &BLE_mArray[8]);
  float2IEEE11073(double(Mvmnt[3]), &BLE_mArray[12]);
  float2IEEE11073(double(Mvmnt[4]), &BLE_mArray[16]);
  float2IEEE11073(double(Mvmnt[5]), &BLE_mArray[20]);
  moveDataCharacteristic.writeValue(BLE_mArray, 24);
}