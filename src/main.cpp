#include <Arduino.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

Servo motorRigth;
Servo motorLeft;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

float pwmLeft, pwmRight;
float kp, ki, accel_ang_y;
float throttle = 1000;
float setpoint = 0.0;
float error, output, up, ui;

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

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  error = setpoint - accel_ang_y;
  up = kp * error;
  ui = ui + ki * error;
  output = up + ui;
  output=constrain(output, -1000, 1000);
  ui=constrain(ui, -1000, 1000);
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

  pwmLeft = throttle + output;
  pwmRight = throttle - output;

  pwmLeft=constrain(pwmLeft,1000,2000);
  pwmRight=constrain(pwmRight,1000,2000);

  accel_ang_y = atan(a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * (180.0 / 3.14);

  motorRigth.writeMicroseconds(pwmRight);
  motorLeft.writeMicroseconds(pwmLeft);
}