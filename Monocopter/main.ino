#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>
#include <SimpleKalmanFilter.h>

#define SEALEVELPRESSURE_HPA (1013.5)
#define TARGET_ALT 0.5

Adafruit_BMP3XX bmp;
Adafruit_MPU6050 mpu;
Servo servo;
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

float baseALT = 0, alt = 0, h_d = 0;
float Kp = 2.0, Ki = 0.1, Kd = 0.5;
float previous_error = 0, integral = 0;
float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;
float sampleFreq = 50.0;

int state = 0;
int yes = 3;
unsigned long stateStartTime = 0;

void computeQuaternion(float gx, float gy, float gz) {
  float qDot1 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5 * ( q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5 * ( q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5 * ( q0 * gz + q1 * gy - q2 * gx);

  q0 += qDot1 * (1.0 / sampleFreq);
  q1 += qDot2 * (1.0 / sampleFreq);
  q2 += qDot3 * (1.0 / sampleFreq);
  q3 += qDot4 * (1.0 / sampleFreq);

  float norm_q = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 /= norm_q; q1 /= norm_q; q2 /= norm_q; q3 /= norm_q;
}

void setup() {
  //Serial.begin(115200);
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();
  servo.attach(2);

  if (!bmp.begin_I2C()) while (1);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_127);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  delay(1000);
  const int numReadings = 100;
  float totalAlt = 0;
  int successfulReadings = 0;
  for (int i = 0; i < numReadings; i++) {
    if (!bmp.performReading()) continue;
    float reading = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    if (isnan(reading)) continue;
    totalAlt += reading;
    successfulReadings++;
    delay(20);
  }
  baseALT = (successfulReadings > 0) ? (totalAlt / successfulReadings + 0.84) : 0;
  // Serial.print("Base Altitude: ");
  // Serial.println(baseALT);

  if (!mpu.begin()) while (1);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(5000);
}

void loop() {
  if (!bmp.performReading()) return;
  alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  h_d = (alt - baseALT) + 22;
  // Serial.print("Altitude: ");
  // Serial.println(h_d);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  static unsigned long lastUpdate = 0;
  float dt = (millis() - lastUpdate) / 1000.0;
  lastUpdate = millis();

  computeQuaternion(g.gyro.x * (M_PI / 180.0), g.gyro.y * (M_PI / 180.0), g.gyro.z * (M_PI / 180.0));
  float quaternionPitch = -asin(2.0 * (q1*q3 - q0*q2)) * (180.0 / M_PI);
  float filteredPitch = simpleKalmanFilter.updateEstimate(quaternionPitch);

  float error = filteredPitch;
  float proportional = Kp * error;
  integral += error * dt;
  float integral_term = Ki * integral;
  float derivative = Kd * (error - previous_error) / dt;
  float pid_output = proportional + integral_term + derivative;
  previous_error = error;

  switch (state) {
    case 0:
      if (yes > 0) {
        state = 1;
        //Serial.println("Starting ascent...");
      }
      break;

    case 1:
      servo.write(120);
      if (h_d >= TARGET_ALT - 0.3) {
        state = 2;
        stateStartTime = millis();
        //Serial.println("Hovering...");
      }
      break;

    case 2:
      servo.write(130);
      if (millis() - stateStartTime >= 5000) {
        state = 3;
        //Serial.println("Descent...");
      }
      break;

    case 3:
      servo.write(100);
      if (h_d <= 0.3) {
        servo.write(0);
        yes--;
        state = 0;
        //Serial.println("Descent complete.");
      }
      break;
  }

  // Serial.print("Filtered Pitch: ");
  // Serial.print(filteredPitch);
  // Serial.print(" | PID Output: ");
  // Serial.println(pid_output);

  delay(100);
}
