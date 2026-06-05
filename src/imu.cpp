#include "imu.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "globals.h"
#include "config.h"

static Adafruit_MPU6050 mpu;

void imuInit()
{
    Wire.begin(21, 22);
    Wire.setClock(100000);  // 100 kHz — robust under ISR interruptions

    if (!mpu.begin()) {
        Serial.println("MPU6050 not found — check wiring");
        while (1) delay(10);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
}

void calibrate()
{
    float gyroSum  = 0.0f;
    float gyroZSum = 0.0f;
    float accelSum = 0.0f;
    const int N = 200;
    for (int i = 0; i < N; i++) {
        sensors_event_t a, g, tmp;
        mpu.getEvent(&a, &g, &tmp);
        gyroSum  += g.gyro.y;
        gyroZSum += g.gyro.z;
        accelSum += atan2f(a.acceleration.z, a.acceleration.x);
        delay(5);
    }

    gyroBias      = gyroSum  / N;
    gyroBiasZ     = gyroZSum / N;
    BALANCE_ANGLE = accelSum / N;
    yaw_rate      = 0.0f;
    yawCorrection = 0.0f;
    yawIntegral   = 0.0f;
    prevYawRate   = 0.0f;
    fallen        = false;
    lastCalibMs   = millis();
    Serial.printf("Calibrated — bias_y=%.4f  bias_z=%.4f  balance=%.4f rad (%.2f deg)\n",
                  gyroBias, gyroBiasZ, BALANCE_ANGLE, BALANCE_ANGLE * 180.0f / PI);
}

bool imuRead(float dt)
{
    sensors_event_t a, g, tmp;
    bool ok = false;
    for (int i = 0; i < 3 && !ok; i++) {
        ok = mpu.getEvent(&a, &g, &tmp);
        if (!ok) { Wire.begin(21, 22); Wire.setClock(100000); delayMicroseconds(100); }
    }
    if (!ok) { imuOk = false; imuErrCount++; return false; }
    imuOk = true;

    accel_angle = atan2f(a.acceleration.z, a.acceleration.x);
    gyro_raw    = g.gyro.y;
    gyro_rate   = g.gyro.y - gyroBias;

    // Yaw rate: bias-correct gyro.z, negate so clockwise = positive, then EMA
    float raw_yaw = -(g.gyro.z - gyroBiasZ);
    yaw_rate = YAW_EMA_ALPHA * raw_yaw + (1.0f - YAW_EMA_ALPHA) * yaw_rate;

    theta = (1.0f - CF_COEFF) * accel_angle
          + CF_COEFF * (theta + gyro_rate * dt);

    return true;
}
