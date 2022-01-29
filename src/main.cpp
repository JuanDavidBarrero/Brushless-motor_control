#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

Servo motorRigth;
Servo motorLeft;

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void initMPU()
{
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("Sensor encontrado");
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
}

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  Serial.println("timer");
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup()
{
  Serial.begin(115200);

  initMPU();

  motorRigth.attach(25);
  motorLeft.attach(26);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 30000, true);
  timerAlarmEnable(timer);

}

void loop()
{

  mpu.getEvent(&a, &g, &temp);

  float accel_ang_y = atan(a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * (180.0 / 3.14);

  motorRigth.writeMicroseconds(1000);
  motorLeft.writeMicroseconds(1000);
  Serial.printf("El angulo en y es %4.2f \n", accel_ang_y);

}