// I2C device class (I2Cdev) demonstration Arduino sketch for ADXL345 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2011-10-07 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include "Wire.h"

// I2Cdev and ADXL345 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "ADXL345.h"

// class default I2C address is 0x53
// specific I2C addresses may be passed as a parameter here
// ALT low = 0x53 (default for SparkFun 6DOF board)
// ALT high = 0x1D
ADXL345 accel;

int16_t ax, ay, az;

#define LED_PIN 13 // (Arduino is 13, Teensy is 6)
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accel.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accel.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");

    // configure LED for output
    
    accel.setRange(ADXL345_RANGE_8G);
    accel.setFullResolution(1);
    accel.setOffsetY(63);
    accel.setOffsetZ(-128);
}

float xmin=32700, xmax=-32700, ymin=32700, ymax=-32700, zmin=32700, zmax=-32700;
//int xmin=32700, xmax=-32700, ymin=32700, ymax=-32700, zmin=32700, zmax=-32700;

void loop() {
    // read raw accel measurements from device
    accel.getAcceleration(&ax, &ay, &az);
    const float scale = 306.5f;
    float fx = -(ax + 72.5f + 2)  * 0.977671  / scale;
    float fy = -(ay - 179.5f + 10) * 0.977671  / scale;
    float fz = -(az - 510.5f + 12) * 1.08881f  / scale;

    
    /*xmin = min(xmin, fx);
    xmax = max(xmax, fx);
    
    ymin = min(ymin, fy);
    ymax = max(ymax, fy);
    
    zmin = min(zmin, fz);
    zmax = max(zmax, fz);
    
    // display tab-separated accel x/y/z values
    Serial.print("accel:\t");
    Serial.print("X:[ "); Serial.print(xmin);Serial.print(" "); Serial.print(xmax); Serial.print(" : "); Serial.print(fx);Serial.print("]\t");
    Serial.print("Y:[ "); Serial.print(ymin);Serial.print(" "); Serial.print(ymax); Serial.print(" : "); Serial.print(fy);Serial.print("]\t");
    Serial.print("Z:[ "); Serial.print(zmin);Serial.print(" "); Serial.print(zmax); Serial.print(" : "); Serial.print(fz);Serial.println("];");
    */
    
    float pitch = atan2(fy,sqrt(fx*fx + fz*fz));
    float roll = atan2(fz,sqrt(fx*fx + fy*fy));

    Serial.print("[ "); Serial.print(pitch);Serial.print(" "); Serial.print(roll); Serial.println("];");
    
}
