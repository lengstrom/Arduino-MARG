// I2C device class (I2Cdev) demonstration Arduino sketch for MPU9150
// 1/4/2013 original by Jeff Rowberg <jeff@rowberg.net> at https://github.com/jrowberg/i2cdevlib
//          modified by Aaron Weiss <aaron@sparkfun.com>
//
// Changelog:
//     2011-10-07 - initial release
//     2013-1-4 - added raw magnetometer output

/* ============================================
I2Cdev device library code is placed under the MIT license

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

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

// System constants
#define LED_PIN 13
//fusion
float deltat = 0.001f;                      // sampling period in seconds (shown as 1 ms)
float gyroMeasError = 3.14159265358979f * (5.0f / 180.0f);    // gyroscope measurement error in rad/s (shown as 5 deg/s) // compute beta
float beta = (sqrt(3.0f / 4.0f) * gyroMeasError);         // accelerometer measurements

// Global system variables
// fusion
float a_x, a_y, a_z; // accelerometer
float w_x, w_y, w_z; // gyroscope measurements in rad/s
float SEq_1 = 1.0f, SEq_2 = 0.0f, SEq_3 = 0.0f, SEq_4 = 0.0f; // estimated orientation quaternion elements with initial conditions
// Local system variables
// end fusion
bool blinkState = false;

void setup() {
		// join I2C bus (I2Cdev library doesn't do this automatically)
		Wire.begin();

		// initialize serial communication
		// (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
		// it's really up to you depending on your project)
		Serial.begin(38400);

		// initialize device
		Serial.println("Initializing I2C devices...");
		accelgyro.initialize();

		// verify connection
		Serial.println("Testing device connections...");
		Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

		// configure Arduino LED for
		pinMode(LED_PIN, OUTPUT);
}

float n2R(int n) { // number to degrees
	return n * 0.00013315805450393677; //(1.0/32768.0) * 250 * 0.01745329251994; //250 is the maximum
}

float n2g(int n) { // number to g-force
	return n * 0.00006103515626;///32768.0 * 2; //250 is the maximum
}

void loop() {
	// read raw accel/gyro measurements from device
	if (accelgyro.testConnection()) {
		accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		filterUpdate(n2R(gx), n2R(gy), n2R(gz), n2g(ax), n2g(ay), n2g(az));
		Serial.print(SEq_1); Serial.print("\t");
		Serial.print(SEq_2); Serial.print("\t");
		Serial.print(SEq_3); Serial.print("\t");
		Serial.print(SEq_4); Serial.println("\t");
		// blink LED to indicate activity
		blinkState = !blinkState;
		digitalWrite(LED_PIN, blinkState);
	}
	else {
		Serial.println("MPU6050 connection failed");
	}

	//"MPU6050 connection successful" : "MPU6050 connection failed");
	//these methods (and a few others) are also available
	//accelgyro.getAcceleration(&ax, &ay, &az);
	//accelgyro.getRotation(&gx, &gy, &gz);
}

void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z) {
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements 
	float f_1, f_2, f_3; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error

	// Axulirary variables to avoid repeated calculations
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;
	
	// Normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;
	// Compute the objective function and Jacobian

	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	J_11or24 = twoSEq_3;
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1;
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;
	// Compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

	// Normalise the gradient
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4); 
	SEqHatDot_1 /= norm;
	SEqHatDot_2 /= norm;
	SEqHatDot_3 /= norm;
	SEqHatDot_4 /= norm;
	// Compute the quaternion derrivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z; 
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y; 
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x; 
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	// Compute then integrate the estimated quaternion derrivative 
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
	// Normalise quaternion
	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4); SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;
}
