// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class
// 10/7/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added multiple output formats
//                 - added seamless Fastwire support
//      2011-10-07 - initial release

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

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "ws2812_control.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;



// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


//#define LED_PIN 13
bool blinkState = false;

#define aDiv 204.80
#define gDiv 131.0

double frequency = 0.0;
String state = "UPRIGHT";
double prev = 0.0;

// *** NEOPIXEL STUFF ***
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN 32

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 60

// the first LED that should be used
#define START_LED 52
#define START_LED2 30

// the last LED that should be used
#define END_LED 59
#define END_LED2 37

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

// *** LETTERS AND NUMBERS ***
//int _[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//int A[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
//int B[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,1,1, 0,0,1,1,1,1,0,0,1,1,1,1,1,1,0,0};
//int C[] = {0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0, 0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1};
//int D[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0, 0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0};
//int E[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,1,1};
//int F[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0};
//int G[] = {0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,1,1,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,1,1,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,0};
//int H[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,1,1,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
//int I[] = {1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1};
//int J[] = {0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,0, 0,0,0,0,0,0,0,0,1,1,0,0,0,0,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0};
//int K[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0, 0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0, 0,0,1,1,0,0,0,0,0,0,0,0,1,1,0,0, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1};
//int L[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1};
//int M[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0, 0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
//int N[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
//int O[] = {0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0};
//int P[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,0, 0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0};
//int Q[] = {0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,0,0,1,1,0,0,1,1, 0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1};
//int R[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,0,0,0,0,1,1,1,1,0,0,0,0,0,0, 1,1,0,0,0,0,1,1,0,0,1,1,0,0,0,0, 1,1,0,0,0,0,1,1,0,0,0,0,1,1,0,0, 0,0,1,1,1,1,0,0,0,0,0,0,0,0,1,1};
//int S[] = {0,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,1,1,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,1,1,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,1,1,0,0,0,0,1,1, 1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,0};
//int T[] = {1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//int U[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0};
//int V[] = {1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0};
//int W[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0, 0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,1,1,0,0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
//int X[] = {1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1, 0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0, 0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0, 0,0,0,0,1,1,0,0,0,0,1,1,0,0,0,0, 1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1};
//int Y[] = {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1, 0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0, 1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0};
//int Z[] = {1,1,0,0,0,0,0,0,0,0,1,1,1,1,1,1, 1,1,0,0,0,0,0,0,1,1,0,0,0,0,1,1, 1,1,0,0,0,0,1,1,0,0,0,0,0,0,1,1, 1,1,0,0,1,1,0,0,0,0,0,0,0,0,1,1, 1,1,1,1,0,0,0,0,0,0,0,0,0,0,1,1};
//
//int test[] = {0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1, 0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1};

int _[] = {0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0};
 int A[] = {1,1,1,1,1,1,1,1, 1,0,0,1,0,0,0,0, 1,0,0,1,0,0,0,0, 1,0,0,1,0,0,0,0, 1,1,1,1,1,1,1,1};
 int B[] = {1,1,1,1,1,1,1,1, 1,0,0,1,0,0,0,1, 1,0,0,1,0,0,0,1, 1,0,0,1,0,0,0,1, 0,1,1,0,1,1,1,0};
 int C[] = {0,0,1,1,1,1,0,0, 0,1,0,0,0,0,1,0, 1,0,0,0,0,0,0,1, 1,0,0,0,0,0,0,1, 1,0,0,0,0,0,0,1};
 int D[] = {1,1,1,1,1,1,1,1, 1,0,0,0,0,0,0,1, 1,0,0,0,0,0,0,1, 0,1,0,0,0,0,1,0, 0,0,1,1,1,1,0,0};
 int E[] = {1,1,1,1,1,1,1,1, 1,0,0,1,0,0,0,1, 1,0,0,1,0,0,0,1, 1,0,0,1,0,0,0,1, 1,0,0,1,0,0,0,1};
 int F[] = {1,1,1,1,1,1,1,1, 1,0,0,1,0,0,0,0, 1,0,0,1,0,0,0,0, 1,0,0,1,0,0,0,0, 1,0,0,1,0,0,0,0};
 int G[] = {0,1,1,1,1,1,1,1, 1,0,0,0,0,0,0,1, 1,0,0,0,1,0,0,1, 1,0,0,0,1,0,0,1, 1,0,0,0,1,1,1,0};
 int H[] = {1,1,1,1,1,1,1,1, 0,0,0,0,1,0,0,0, 0,0,0,0,1,0,0,0, 0,0,0,0,1,0,0,0, 1,1,1,1,1,1,1,1};
 int I[] = {1,0,0,0,0,0,0,1, 1,0,0,0,0,0,0,1, 1,1,1,1,1,1,1,1, 1,0,0,0,0,0,0,1, 1,0,0,0,0,0,0,1};
 int J[] = {0,0,0,0,0,1,1,0, 0,0,0,0,1,0,0,1, 0,0,0,0,0,0,0,1, 0,0,0,0,0,0,0,1, 1,1,1,1,1,1,1,0};
 int K[] = {1,1,1,1,1,1,1,1, 0,0,0,1,1,0,0,0, 0,0,1,0,0,1,0,0, 0,1,0,0,0,0,1,0, 1,0,0,0,0,0,0,1};
 int L[] = {1,1,1,1,1,1,1,1, 0,0,0,0,0,0,0,1, 0,0,0,0,0,0,0,1, 0,0,0,0,0,0,0,1, 0,0,0,0,0,0,0,1};
 int M[] = {1,1,1,1,1,1,1,1, 0,1,0,0,0,0,0,0, 0,0,1,0,0,0,0,0, 0,1,0,0,0,0,0,0, 1,1,1,1,1,1,1,1};
 int N[] = {1,1,1,1,1,1,1,1, 0,0,1,0,0,0,0,0, 0,0,0,1,1,0,0,0, 0,0,0,0,0,1,0,0, 1,1,1,1,1,1,1,1};
 int O[] = {0,1,1,1,1,1,1,0, 1,0,0,0,0,0,0,1, 1,0,0,0,0,0,0,1, 1,0,0,0,0,0,0,1, 0,1,1,1,1,1,1,0};
 int P[] = {1,1,1,1,1,1,1,1, 1,0,0,1,0,0,0,0, 1,0,0,1,0,0,0,0, 1,0,0,1,0,0,0,0, 0,1,1,0,0,0,0,0};
 int Q[] = {0,1,1,1,1,1,1,0, 1,0,0,0,0,0,0,1, 1,0,0,0,0,1,0,1, 0,1,1,1,1,1,1,0, 0,0,0,0,0,0,0,1};
 int R[] = {1,1,1,1,1,1,1,1, 1,0,0,1,1,0,0,0, 1,0,0,1,0,1,0,0, 1,0,0,1,0,0,1,0, 0,1,1,0,0,0,0,1};
 int S[] = {0,1,1,1,0,0,0,1, 1,0,0,0,1,0,0,1, 1,0,0,0,1,0,0,1, 1,0,0,0,1,0,0,1, 1,0,0,0,1,1,1,0};
 int T[] = {1,0,0,0,0,0,0,0, 1,0,0,0,0,0,0,0, 1,1,1,1,1,1,1,1, 1,0,0,0,0,0,0,0, 1,0,0,0,0,0,0,0};
 int U[] = {1,1,1,1,1,1,1,0, 0,0,0,0,0,0,0,1, 0,0,0,0,0,0,0,1, 0,0,0,0,0,0,0,1, 1,1,1,1,1,1,1,0};
 int V[] = {1,1,1,1,1,1,0,0, 0,0,0,0,0,0,1,0, 0,0,0,0,0,0,0,1, 0,0,0,0,0,0,1,0, 1,1,1,1,1,1,0,0};
 int W[] = {1,1,1,1,1,1,1,1, 0,0,0,0,0,0,1,0, 0,0,0,0,0,1,0,0, 0,0,0,0,0,0,1,0, 1,1,1,1,1,1,1,1};
 int X[] = {1,1,0,0,0,0,1,1, 0,0,1,0,0,1,0,0, 0,0,0,1,1,0,0,0, 0,0,1,0,0,1,0,0, 1,1,0,0,0,0,1,1};
 int Y[] = {1,1,0,0,0,0,0,0, 0,0,1,0,0,0,0,0, 0,0,0,1,1,1,1,1, 0,0,1,0,0,0,0,0, 1,1,0,0,0,0,0,0};
 int Z[] = {1,0,0,0,0,1,1,1, 1,0,0,0,1,0,0,1, 1,0,0,1,0,0,0,1, 1,0,1,0,0,0,0,1, 1,1,0,0,0,0,0,1};
 
int letterSpace;
int delayTime;
int myMap[27][40];

// *** WIFI STUFF ***
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

// Set these to your desired credentials.
const char *ssid = "yourAP";
const char *password = "yourPassword";
String myInput = "";
String totalAy = "";

WiFiServer server(80);

// *** USING NEOPIXEL LIBRARY ****
//#define NUM_LEDS 60
//#define RED   0xFF0000
//#define GREEN 0x00FF00
//#define BLUE  0x0000FF

void setup() {
    //ws2812_control_init();
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    //Serial.begin(38400);
    Serial.begin(115200);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

//    // use the code below to change accel/gyro offset values
//    Serial.println("Updating internal sensor offsets...");
//    // -76  -2359 1688  0 0 0
//    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
//    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
//    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
//    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
//    Serial.print("\n");
//    accelgyro.setXGyroOffset(220);
//    accelgyro.setYGyroOffset(76);
//    accelgyro.setZGyroOffset(-85);
//    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
//    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
//    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
//    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
//    Serial.print("\n");

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);

    // *** NEOPIXEL SETUP ***
    #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
        clock_prescale_set(clock_div_1);
    #endif
    // END of Trinket-specific code.

    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.show();            // Turn OFF all pixels ASAP
    strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

    // *** LETTERS SET UP ***
    letterSpace = 500;

    setMapInitially(_,0);
    setMapInitially(A,1);
    setMapInitially(B,2);
    setMapInitially(C,3);
    setMapInitially(D,4);
    setMapInitially(E,5);
    setMapInitially(F,6);
    setMapInitially(G,7);
    setMapInitially(H,8);
    setMapInitially(I,9);
    setMapInitially(J,10);
    setMapInitially(K,11);
    setMapInitially(L,12);
    setMapInitially(M,13);
    setMapInitially(N,14);
    setMapInitially(O,15);
    setMapInitially(P,16);
    setMapInitially(Q,17);
    setMapInitially(R,18);
    setMapInitially(S,19);
    setMapInitially(T,20);
    setMapInitially(U,21);
    setMapInitially(V,22);
    setMapInitially(W,23);
    setMapInitially(X,24);
    setMapInitially(Y,25);
    setMapInitially(Z,26);

   // **** WIFI SETUP ***
   Serial.println();
   Serial.println("Configuring access point...");

   // You can remove the password parameter if you want the AP to be open.
    WiFi.softAP(ssid, password);
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);
    server.begin();

    Serial.println("Server started");
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
//        Serial.print("a/g:\t");
//        Serial.print(ax/aDiv); Serial.print("\t");
//        Serial.print(ay/aDiv); Serial.print("\t");
//        Serial.print(az/aDiv); Serial.print("\t");
//        Serial.print(gx/gDiv); Serial.print("\t");
//        Serial.print(gy/gDiv); Serial.print("\t");
//        Serial.println(gz/gDiv);
    #endif
    updateFrequency(ay/aDiv, az/aDiv);
    if (delayTime != 0) {
      //Serial.println(state);
      //Serial.println(frequency);

      #ifdef OUTPUT_BINARY_ACCELGYRO
          Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
          Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
          Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
          Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
          Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
          Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
      #endif

      // blink LED to indicate activity
      //blinkState = !blinkState;
      //digitalWrite(LED_PIN, blinkState);

      // obtains input necessary from client
      wifiLoop();
      // ***CALL THIS FUNCTION TO PRINT OUT VARIABLE STRINGS ON THE MACHINE ****
      if (myInput.length() > 0) {
        myInput.toUpperCase();
        changeBasedOnInput(myInput);
      }

      // ***CALL THIS FUNCTION TO PRINT OUT A SINGLE CHARACTER ON THE MACHINE ***
      // used as a test to ensure it is working --> reference test array to see what should be printed
      //changeBasedOnChar(A);
      Serial.println(delayTime);
//      clearAll();
//      strip.show();
//      delay(500);      
    }
}

void setMapInitially(int arr[], int at) {
  for (int i = 0; i < 80; i++) {
    myMap[at][i] = arr[i];
  }
}

void wifiLoop() {
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,e
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Input: <input id=\"input\" type=\"text\" name=\"Input\"><br>");
            client.print("<button type=\"button\" onclick=\"myFunc()\">Submit!</button><br> <script> function myFunc() {window.location.href = '/' + document.getElementById('input').value} </script><br><br>");

            totalAy += (ay/aDiv);
            totalAy += " ";
            client.print(totalAy);

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            //Serial.println("**************" + currentLine);
            if (currentLine.indexOf("GET /") >= 0 && currentLine.indexOf("HTTP/1.1") >= 0) {
              int indexOfHTTP = currentLine.indexOf("HTTP/1.1");
              String weGotIt = currentLine.substring(5, indexOfHTTP - 1);
              if (!weGotIt.equals("favicon.ico")) {
                myInput = weGotIt;
                myInput.replace("%20"," ");
                Serial.println(myInput);
              }
            }
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }
}

void clearAll() {
  for (int i = 0; i < 60; i++) {
    strip.setPixelColor(i, strip.Color(0,0,0));
  }
  strip.show();
}

void setAll() {
  for (int i = 52; i < 60; i++) {
    strip.setPixelColor(i, strip.Color(0,0,255));
  }
  strip.show();
}

void colorChanges() {
  for (int i = 0; i < START_LED - 1; i++) {
    strip.setPixelColor(i, strip.Color(0,0,0));
  }
  for (int i = START_LED; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(255,0,255));
  }
  strip.show();
}

void colorChangeBasedOnStates() {
  if (state.equals("UPRIGHT")) { // magenta
    for (int i = 0; i < START_LED - 1; i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    for (int i = START_LED; i < LED_COUNT; i++) {
      strip.setPixelColor(i, strip.Color(255,0,255));
    }
    strip.show();
  } else if (state.equals("RIGHTDOWN")) { // blue
    for (int i = 0; i < START_LED - 1; i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    for (int i = START_LED; i < LED_COUNT; i++) {
      strip.setPixelColor(i, strip.Color(0,0,255));
    }
    strip.show();
  } else if (state.equals("DOWNLEFT")) { // red
    for (int i = 0; i < START_LED - 1; i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    for (int i = START_LED; i < LED_COUNT; i++) {
      strip.setPixelColor(i, strip.Color(255,0,0));
    }
    strip.show();
  } else if (state.equals("LEFTUP")) { // green
    for (int i = 0; i < START_LED - 1; i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    for (int i = START_LED; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(0,255,0));
    }
    strip.show();
  } else {
    for (int i = 0; i < START_LED - 1; i++) {
      strip.setPixelColor(i, strip.Color(0,0,0));
    }
    for (int i = START_LED; i < LED_COUNT; i++) {
      strip.setPixelColor(i, strip.Color(100,100,100));
    }
    strip.show();
  }
}

void changeBasedOnInput(String str) {
  // for each letter in the input string
  for (int i = 0; i < str.length(); i++) {
    if (str.charAt(i) == ' ') {
      // if it is a space
      changeBasedOnChar(_);
    } else {
      // array below is the array that is going to be changing
//      int letterToChange[40];
//      int at = str.charAt(i - 64);
//      for (int i = 0; i < 40; i++) {
//        letterToChange[i] = myMap[at][i];
//      }
      changeBasedOnChar(myMap[str.charAt(i)-64]);
    }
  }
}

void changeBasedOnChar(int letter[]) {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  updateFrequency(ay,az);
  double fullCircleDelay = lookUpValue(ay) * 1000;
  int i;
  // first corresponds to last --> 59
  struct led_state new_state;
  for (i = 0; i < 8; i++) {
    if (letter[i] == 1) {
      //Serial.println("***HERE***");
      strip.setPixelColor(END_LED-i, strip.Color(0,0,255));
      strip.setPixelColor(START_LED2+i, strip.Color(0,0,255));
      //new_state.leds[END_LED-i] = BLUE;
    } else {
      strip.setPixelColor(END_LED-i, strip.Color(0,0,0));
      strip.setPixelColor(START_LED2+i, strip.Color(0,0,0));
      //new_state.leds[END_LED-i] = 0x000000;
    }
  }
//  for (i = 0; i < START_LED; i++) {
//    new_state.leds[i] = 0x000000;
//  }
  for (i = 0; i < START_LED2; i++) {
    strip.setPixelColor(i, strip.Color(0,0,0));
  }
  strip.show();
  //ws2812_write_leds(new_state);
  delay(delayTime);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  updateFrequency(ay,az);
  fullCircleDelay = lookUpValue(ay) * 1000;
  for (i = 0; i < 8; i++) {
    if (letter[i+8] == 1) {
      strip.setPixelColor(END_LED-i, strip.Color(0,0,255));
      strip.setPixelColor(START_LED2+i, strip.Color(0,0,255));
      //new_state.leds[END_LED-i] = BLUE;
    } else {
      strip.setPixelColor(END_LED-i, strip.Color(0,0,0));
      strip.setPixelColor(START_LED2+i, strip.Color(0,0,0));
      //new_state.leds[END_LED-i] = 0x000000;
    }
    //strip.show();
  }
//  for (i = 0; i < START_LED; i++) {
//    new_state.leds[i] = 0x000000;
//  }
  for (i = 0; i < START_LED2; i++) {
    strip.setPixelColor(i, strip.Color(0,0,0));
  }
  strip.show();
  //ws2812_write_leds(new_state);
  delay(delayTime);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  updateFrequency(ay,az);
  fullCircleDelay = lookUpValue(ay) * 1000;
  for (i = 0; i < 8; i++) {
    if (letter[i+16] == 1) {
      strip.setPixelColor(START_LED2+i, strip.Color(0,0,255));
      strip.setPixelColor(END_LED-i, strip.Color(0,0,255));
      //new_state.leds[END_LED-i] = BLUE;
    } else {
      strip.setPixelColor(START_LED2+i, strip.Color(0,0,0));
      strip.setPixelColor(END_LED-i, strip.Color(0,0,0));
      //new_state.leds[END_LED-i] = 0x000000;
    }
  }
//  for (i = 0; i < START_LED; i++) {
//    new_state.leds[i] = 0x000000;
//  }
  for (i = 0; i < START_LED2; i++) {
    strip.setPixelColor(i, strip.Color(0,0,0));
  }
  strip.show();
  //ws2812_write_leds(new_state);
  delay(delayTime);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  updateFrequency(ay,az);
  fullCircleDelay = lookUpValue(ay) * 1000;
  for (i = 0; i < 8; i++) {
    if (letter[i+24] == 1) {
      strip.setPixelColor(START_LED2+i, strip.Color(0,0,255));
      strip.setPixelColor(END_LED-i, strip.Color(0,0,255));
      //new_state.leds[END_LED-i] = BLUE;
    } else {
      strip.setPixelColor(START_LED2+i, strip.Color(0,0,0));
      strip.setPixelColor(END_LED-i, strip.Color(0,0,0));
      //new_state.leds[END_LED-i] = 0x000000;
    }
  }
//  for (i = 0; i < START_LED; i++) {
//    new_state.leds[i] = 0x000000;
//  }
  for (i = 0; i < START_LED2; i++) {
    strip.setPixelColor(i, strip.Color(0,0,0));
  }
  strip.show();
  //ws2812_write_leds(new_state);
  delay(delayTime);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  updateFrequency(ay,az);
  fullCircleDelay = lookUpValue(ay) * 1000;
  for (i = 0; i < 8; i++) {
    if (letter[i+32] == 1) {
      strip.setPixelColor(START_LED2+i, strip.Color(0,0,255));
      strip.setPixelColor(END_LED-i, strip.Color(0,0,255));
      //new_state.leds[END_LED-i] = BLUE;
    } else {
      strip.setPixelColor(START_LED2+i, strip.Color(0,0,0));
      strip.setPixelColor(END_LED-i, strip.Color(0,0,0));
      //new_state.leds[END_LED-i] = 0x000000;
    }
  }
//  for (i = 0; i < START_LED; i++) {
//    new_state.leds[i] = 0x000000;
//  }
  for (i = 0; i < START_LED2; i++) {
    strip.setPixelColor(i, strip.Color(0,0,0));
  }
  strip.show();
  //ws2812_write_leds(new_state);
  delay(delayTime);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  updateFrequency(ay,az);
  fullCircleDelay = lookUpValue(ay) * 1000;
//  for (i = 0; i < 8; i++) {
//    strip.setPixelColor(END_LED-i, strip.Color(0,0,0));
//    //new_state.leds[END_LED-i] = 0x000000;
//  }
////  for (i = 0; i < START_LED; i++) {
////    new_state.leds[i] = 0x000000;
////  }
//  for (i = 0; i < START_LED; i++) {
//    strip.setPixelColor(i, strip.Color(0,0,0));
//  }
//  //strip.clear();
//  //ws2812_write_leds(new_state);
//  //clearAll();
//  strip.show();
//  delay(letterSpace);
   clearAll();
   strip.show();
   delay(letterSpace); 
}

double lookUpValue(double ay) {
  if (ay < 70) {
    return 0.4;
  } else if (ay >= 70 && ay < 90) {
    return (((ay - 70)/20) * (0.354 - 0.328)) + 0.328;
  } else if (ay >= 90 && ay < 110) {
    return (((ay - 90)/20) * (0.328 - 0.268)) + 0.268;
  } else if (ay >= 110 && ay < 140) {
     return (((ay - 110)/30) * (0.268 - 0.262)) + 0.262;
  } else if (ay >= 140) {
    return 0.25;
  } 
}

void updateFrequency(double ay, double az) {
  unsigned long currentTime = millis();
  double currTime = 1.0 * currentTime;
  double diffTime = currTime - prev;
  //diffTime = diffTime / 1000.0;
  // up-> right (detect y becoming greater than 0) --> transition to right->down
  // right->down (detect when z becomes less than 0) --> transitin to down->left
  // down->left (detect when y becomes less than 0) --> transition left-> up
  // left->up (detect when z becomes greater than 0) --> transition up->right
  if (state.equals("UPRIGHT")) {
    if (ay > 0) {
      state = "RIGHTDOWN";
      prev = currTime;
      frequency = 1.0 / (diffTime * 4.0);
      delayTime = 9; //(diffTime) * 4.0;
      //changeBasedOnChar(A);
    }
  } else if (state.equals("RIGHTDOWN")) {
    if (az < 0) {
      state = "DOWNLEFT";
      prev = currTime;
      frequency = 1.0 / (diffTime * 4.0);
      delayTime = 9; //(diffTime) * 4.0;
      //changeBasedOnChar(B);
    }
    
  } else if (state.equals("DOWNLEFT")) {
    if (ay < 0) {
      state = "LEFTUP";
      prev = currTime;
      frequency = 1.0 / (diffTime * 4.0);
      delayTime = 9;//(diffTime) * 4.0;
      //changeBasedOnChar(C);
    }
    
  } else if (state.equals("LEFTUP")) {
    if (az > 0) {
      state = "UPRIGHT";
      prev = currTime;
      frequency = 1.0 / (diffTime * 4.0);
      delayTime = 9;//(diffTime) * 4.0;
      //changeBasedOnChar(D);
    }
    
  } else {
    Serial.println("ERROR");
  }
  
}
