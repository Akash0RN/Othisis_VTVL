#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>

#define BAUD 115200
#define SDA_PIN 0
#define SCL_PIN 1
#define SERVO_PIN 2
#define SAMPLE_FREQ 50.0f //20ms

Adafruit_MPU6050 mpu;
Servo servo;
SimpleKalmanFilter kalmanFilter(2, 2, 0.01); //(Measured uncertainty ; Estimated Uncertainty ; Process Noise Variance)

float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // {q0, q1, q2, q3}

void setup() {
    Serial.begin(BAUD);
    Wire.setSDA(SDA_PIN);
    Wire.setSCL(SCL_PIN);
    Wire.begin();

    servo.attach(SERVO_PIN);
    servo.write(0);
    
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void updateQuaternion(const sensors_event_t& gEvent, float dt) {
    float gyroNorm = sqrt(gEvent.gyro.x * gEvent.gyro.x + gEvent.gyro.y * gEvent.gyro.y + gEvent.gyro.z * gEvent.gyro.z);
    if (gyroNorm > 0.0f) {
        float norm[3] = {gEvent.gyro.x / gyroNorm, gEvent.gyro.y / gyroNorm, gEvent.gyro.z / gyroNorm};
        float theta = gyroNorm * dt;
        float qRot[4] = {cos(theta / 2.0f), 
                         norm[0] * sin(theta / 2.0f), 
                         norm[1] * sin(theta / 2.0f), 
                         norm[2] * sin(theta / 2.0f)};

        float tempQ[4];
        tempQ[0] = qRot[0] * q[0] - qRot[1] * q[1] - qRot[2] * q[2] - qRot[3] * q[3];
        tempQ[1] = qRot[0] * q[1] + qRot[1] * q[0] + qRot[2] * q[3] - qRot[3] * q[2];
        tempQ[2] = qRot[0] * q[2] - qRot[1] * q[3] + qRot[2] * q[0] + qRot[3] * q[1];
        tempQ[3] = qRot[0] * q[3] + qRot[1] * q[2] - qRot[2] * q[1] + qRot[3] * q[0];

        //FIXED
        float qNorm = sqrt(tempQ[0] * tempQ[0] + tempQ[1] * tempQ[1] + tempQ[2] * tempQ[2] + tempQ[3] * tempQ[3]);
        for (int i = 0; i < 4; ++i) {
            q[i] = tempQ[i] / qNorm;
        }
    }
}

void loop() {
    static unsigned long lastUpdate = 0;
    float dt = (millis() - lastUpdate) / 1000.0f;
    lastUpdate = millis();

    sensors_event_t aEvent, gEvent, tempEvent;
    mpu.getEvent(&aEvent, &gEvent, &tempEvent);
    updateQuaternion(gEvent, dt);

    // quaternion to euler
    float pitch = atan2(2.0f * (q[2] * q[3] + q[0] * q[1]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * (180.0f / M_PI);
    float roll = asin(2.0f * (q[1] * q[3] - q[0] * q[2])) * (180.0f / M_PI);
    float yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * (180.0f / M_PI);

    float filteredPitch = kalmanFilter.updateEstimate(pitch);

    int servoPosition = map(static_cast<int>(filteredPitch), -90, 90, 180, 0);
    servo.write(servoPosition);
    delay(10);

    // debugging
    // Serial.print("Pitch: "); Serial.print(pitch, 2);
    // Serial.print(" | Filtered Pitch: "); Serial.print(filteredPitch, 2);
    // Serial.print(" | Roll: "); Serial.print(roll, 2);
    // Serial.print(" | Yaw: "); Serial.print(yaw, 2);
    // Serial.print(" | Servo: "); Serial.println(servoPosition);
}