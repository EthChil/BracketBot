/*
Compiled for the Node MCU 1.0 ESP Microcontroller
BNO085 connected to default I2C pins
IMU mounted with PCB parallel to the side plane of the robot, header pins parallel to floor plane
*/

#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
};
euler_t ypr;
euler_t dypr;
euler_t grv;

float pitchPrev = 0;
float yawPrev = 0;
float prevWrap = 0;
int wraps = 0;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setReports() {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
    Serial.println("Could not enable stabilized remote vector");
  }
//  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
//    Serial.println("Could not enable gyroscope");
//  }
}

void setup(void) {
  Serial.begin(115200);
  Serial1.begin(115200);
  while (!Serial) delay(10);
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports();

  Serial.println("Reading events");
  delay(100);
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {
    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));
    ypr->yaw = yawWrap(ypr->yaw);
    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

float yawWrap(float yaw) {
    float norm = atan2(sin(yaw), cos(yaw));
    if(norm - prevWrap > PI) {
      wraps--;
    } else if (prevWrap - norm > PI) {
      wraps++;
    }
    prevWrap = norm;
    return norm + 2*PI*wraps;
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void pitchFromVector(sh2_Accelerometer_t* gravity_vector, euler_t* grv, bool degrees = false) {
    grv->pitch = atan2(gravity_vector->y, gravity_vector->x);
    grv->roll = atan2(gravity_vector->z, sqrt(gravity_vector->x*gravity_vector->x + gravity_vector->y*gravity_vector->y));
    grv->yaw = atan2(-gravity_vector->x*sin(grv->yaw) + gravity_vector->y*cos(grv->yaw), gravity_vector->z*cos(grv->roll) + gravity_vector->x*cos(grv->yaw)*sin(grv->roll) + gravity_vector->y*sin(grv->yaw)*sin(grv->roll));
    
    if (degrees) {
      grv->yaw *= RAD_TO_DEG;
      grv->pitch *= RAD_TO_DEG;
      grv->pitch -= 90;
      grv->roll *= RAD_TO_DEG;
    }
}

void gyroRates(sh2_Gyroscope_t* gyro, euler_t* dypr, bool degrees = false) {
    dypr->pitch = gyro->z;
    dypr->yaw = gyro->y;
    dypr->roll = gyro->x;
    
    if (degrees) {
      dypr->yaw *= RAD_TO_DEG;
      dypr->pitch *= RAD_TO_DEG;
      dypr->roll *= RAD_TO_DEG;
    }
}

void loop() {
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }
  if (bno08x.getSensorEvent(&sensorValue)) {
        switch (sensorValue.sensorId) {
//          case SH2_GYROSCOPE_CALIBRATED: // Best rate reading
//            Serial.println(sensorValue.un.gyroscope.z);
//            gyroRates(&sensorValue.un.gyroscope, &dypr, false);
//            break;
          case SH2_ARVR_STABILIZED_RV: // Best angle reading
            quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, false);
            break;
    }
    static long last = 0;
    long now = micros();
    float deltaT = float((now-last)/1000000.0);
    last = now;
    float pitch_rate = -(ypr.pitch - pitchPrev)/deltaT; // Negative to match gyro orientation
    float yaw_rate = (ypr.yaw - yawPrev)/deltaT;
    String msg = 's'+String(ypr.pitch, 6)+','+String(ypr.yaw, 6)+','+String(pitch_rate, 6)+','+String(yaw_rate, 6) + 'e';
    Serial1.print(msg);
//    Serial.println(msg);
    pitchPrev = ypr.pitch;
    yawPrev = ypr.yaw;
  }
}