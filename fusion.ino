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
int16_t mx, my, mz;

// System constants
#define LED_PIN 13

//fusion
float deltat = 0.001f;
float lastt = 0;
float ct = 0;
float l = 0;

float gyroMeasError = 0; //3.14159265358979 * (0 / 180); 
float gyroMeasDrift = 0; 
float beta = sqrt(3.0 / 4.0) * gyroMeasError;
float zeta = sqrt(3.0 / 4.0) * gyroMeasDrift;
int b = 0;
// Global system variables 
float a_x, a_y, a_z;
float w_x, w_y, w_z;

float m_x, m_y, m_z;
float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; 
float b_x = 1, b_z = 0;
float w_bx = 0, w_by = 0, w_bz = 0;

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
        accelgyro.setFullScaleGyroRange(1); //set range to +-500°/s
        accelgyro.setFullScaleAccelRange(0); //set range to +-2g

	// verify connection
	Serial.println("Testing device connections...");
	Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	// configure Arduino LED for
	pinMode(LED_PIN, OUTPUT);
}

float n2R(int n, float intercept) { // number to degrees
        float deg = ((n/32768.0) * 500.0 + intercept);
        if (abs(deg) < .3) {
          return 0;
        }
	return deg * (1/57.29578);
}

float n2g(int n) { // number to g-force // useless because we're looking at magnitude
	return n * 0.00006103515626;// 1/32768.0 * 2; 
}

double getPitch(double q0, double q1, double q2, double q3) {
	double top = 2*(q1*q3 + q0*q2);
	double bottom = sqrt(1-pow((2*q1*q3+2*q0*q2),2));
	return (-atan(top/bottom))*57.29578;
}

double getYaw(double q0, double q1, double q2, double q3) {
	double arg1 = 2*(q2*q3-q0*q1);
	double arg2 = 2*pow(q0,2) - 1 + 2*pow(q3,2);
	return atan2(arg1,arg2)*57.29578;
}

double getRoll(double q0, double q1, double q2, double q3) {
	double arg1 = 2*(q1*q2-q0*q3);
	double arg2 = 2*pow(q0,2) - 1 + 2*pow(q1,2);
	return atan2(arg1,arg2)*57.29578;
}

void loop() {
	// read raw accel/gyro measurements from device
	if (accelgyro.testConnection()) {
		if (b > 0) {
			accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
			ct = millis();
			deltat = (ct - lastt) * 0.001;
			lastt = ct;
			float xR = n2R(gx, -0.926877914);
			float yR = n2R(gy, 0.758192);
			float zR = n2R(gz, 1.94250294);
			if (abs(zR) > abs(l)) {
				l = zR;
			}
			filterUpdate(xR, yR, zR, ax, ay, az, mx, my, mz);
			// if (b % 25 == 0) {
			double pitch = getPitch(SEq_1, SEq_2, SEq_3, SEq_4);
			double yaw = getYaw(SEq_1, SEq_2, SEq_3, SEq_4);
			double roll = getRoll(SEq_1, SEq_2, SEq_3, SEq_4);
			Serial.print("p/y/r"); Serial.print("\t");
			Serial.print(pitch); Serial.print("\t");
			Serial.print(yaw); Serial.print("\t");
			Serial.print(roll); Serial.print("\t");
                        Serial.println(l * 180/3.14159);
			// }
		} else {
			lastt = millis();
		}
		b++;
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

void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z) {
	// local system variables
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements // objective function elements
	float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, 
	J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; // objective function Jacobian elements
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;
	float w_err_x, w_err_y, w_err_z;
	float h_x, h_y, h_z;
	
	// axulirary variables to avoid repeated calculations
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;
	float twoSEq_4 = 2.0f * SEq_4;
	float twob_x = 2.0f * b_x;
	float twob_z = 2.0f * b_z;

	float twob_xSEq_1 = 2.0f * b_x * SEq_1;
	float twob_xSEq_2 = 2.0f * b_x * SEq_2;
	float twob_xSEq_3 = 2.0f * b_x * SEq_3;
	float twob_xSEq_4 = 2.0f * b_x * SEq_4;
	float twob_zSEq_1 = 2.0f * b_z * SEq_1;
	float twob_zSEq_2 = 2.0f * b_z * SEq_2;
	float twob_zSEq_3 = 2.0f * b_z * SEq_3;
	float twob_zSEq_4 = 2.0f * b_z * SEq_4;

	float SEq_1SEq_2;
	float SEq_1SEq_3 = SEq_1 * SEq_3;
	float SEq_1SEq_4;
	float SEq_2SEq_3;
	float SEq_2SEq_4 = SEq_2 * SEq_4;
	float SEq_3SEq_4;
	float twom_x = 2.0f * m_x;
	float twom_y = 2.0f * m_y;
	float twom_z = 2.0f * m_z;

	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;

	// normalise the magnetometer measurement 
	norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
	m_x /= norm;
	m_y /= norm;
	m_z /= norm;

	// compute the objective function and Jacobian
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
	f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
	f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;

	J_11or24 = twoSEq_3;
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1;

	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21;
	J_33 = 2.0f * J_11or24;
	J_41 = twob_zSEq_3;
	J_42 = twob_zSEq_4;
	J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1;
	J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2;
	J_51 = twob_xSEq_4 - twob_zSEq_2;
	J_52 = twob_xSEq_3 + twob_zSEq_1;
	J_53 = twob_xSEq_2 + twob_zSEq_4;
	J_54 = twob_xSEq_1 - twob_zSEq_3;
	J_61 = twob_xSEq_3;
	J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
	J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
	J_64 = twob_xSEq_2;

	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 -J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 -J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

	// normalise the gradient to estimate direction of the gyroscope error
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);

	SEqHatDot_1 = SEqHatDot_1 / norm;
	SEqHatDot_2 = SEqHatDot_2 / norm;
	SEqHatDot_3 = SEqHatDot_3 / norm;
	SEqHatDot_4 = SEqHatDot_4 / norm;

	w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
	w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
	w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;

	w_bx += w_err_x * deltat * zeta;
	w_by += w_err_y * deltat * zeta;
	w_bz += w_err_z * deltat * zeta;
	w_x -= w_bx;
	w_y -= w_by;
	w_z -= w_bz;

	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

	SEq_1 += (SEqDot_omega_1 - (beta *SEqHatDot_1)) * deltat;
	SEq_2 += (SEqDot_omega_2 - (beta *SEqHatDot_2)) * deltat;
	SEq_3 += (SEqDot_omega_3 - (beta *SEqHatDot_3)) * deltat;
	SEq_4 += (SEqDot_omega_4 - (beta *SEqHatDot_4)) * deltat;

	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;

	// compute flux in the earth frame 
	SEq_1SEq_2 = SEq_1 * SEq_2;
	SEq_1SEq_3 = SEq_1 * SEq_3;
	SEq_1SEq_4 = SEq_1 * SEq_4;
	SEq_3SEq_4 = SEq_3 * SEq_4;
	SEq_2SEq_3 = SEq_2 * SEq_3;
	SEq_2SEq_4 = SEq_2 * SEq_4;

	h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
	h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
	h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
	
	// normalise the flux vector to have only components in the x and z 
	b_x = sqrt((h_x * h_x) + (h_y * h_y));
	b_z = h_z;
}