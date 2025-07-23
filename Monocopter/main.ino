#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMI088.h>
#include <Adafruit_BMP3XX.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>

#define SEALEVELPRESSURE_HPA (1013.5)
#define TARGET_ALT 0.5

Adafruit_BMI088 bmi;
Adafruit_BMP3XX bmp;
Servo servo;
SimpleKalmanFilter simpleKalmanFilter(2, 2, 0.01);

float baseALT = 0, alt = 0, h_d = 0;
float pitch = 0;
float sampleFreq = 50.0;

float q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;

float pitch_Kp = 2.0, pitch_Ki = 0.1, pitch_Kd = 0.5;
float alt_Kp = 15.0, alt_Ki = 0.3, alt_Kd = 3.5;

float pitch_integral = 0, pitch_prev_error = 0;
float alt_integral = 0, alt_prev_error = 0;

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
  Wire.setSDA(0);
  Wire.setSCL(1);
  Wire.begin();
  servo.attach(2);

  bmp.begin_I2C();
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_127);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  delay(1000);
  float totalAlt = 0;
  int count = 0;
  for (int i = 0; i < 100; i++) {
    if (!bmp.performReading()) continue;
    float reading = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    if (isnan(reading)) continue;
    totalAlt += reading;
    count++;
    delay(20);
  }
  baseALT = (count > 0) ? (totalAlt / count + 0.84) : 0;

  bmi.begin_BMI055();
  bmi.setAccelerometerRange(BMI088_RANGE_6_G);
  bmi.setGyroRange(BMI088_RANGE_500_DEG);
  bmi.setAccelDataRate(BMI088_DATARATE_100_HZ);
  bmi.setGyroDataRate(BMI088_GYRO_DATARATE_100_HZ);

  delay(5000);
}

void loop() {
  static unsigned long lastUpdate = 0;
  float dt = (millis() - lastUpdate) / 1000.0;
  lastUpdate = millis();

  if (!bmp.performReading()) return;
  alt = bmp.readAltitude(SEALEVELPRESSURE_HPA);
  h_d = (alt - baseALT) + 22;

  sensors_event_t accel, gyro, temp;
  bmi.getEvent(&accel, &gyro, &temp);

  computeQuaternion(
    gyro.gyro.x * (M_PI / 180.0),
    gyro.gyro.y * (M_PI / 180.0),
    gyro.gyro.z * (M_PI / 180.0)
  );

  pitch = -asin(2.0 * (q1*q3 - q0*q2)) * (180.0 / M_PI);
  float filteredPitch = simpleKalmanFilter.updateEstimate(pitch);

  float pitch_error = filteredPitch;
  pitch_integral += pitch_error * dt;
  float pitch_derivative = (pitch_error - pitch_prev_error) / dt;
  float pitch_pid = pitch_Kp * pitch_error + pitch_Ki * pitch_integral + pitch_Kd * pitch_derivative;
  pitch_prev_error = pitch_error;

  float alt_error = TARGET_ALT - h_d;
  alt_integral += alt_error * dt;
  float alt_derivative = (alt_error - alt_prev_error) / dt;
  float alt_pid = alt_Kp * alt_error + alt_Ki * alt_integral + alt_Kd * alt_derivative;
  alt_prev_error = alt_error;

  float output = constrain(120 + alt_pid - pitch_pid, 0, 180);
  servo.write((int)output);

  delay(50);
}
