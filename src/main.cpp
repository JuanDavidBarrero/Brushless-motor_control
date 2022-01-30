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
float accel_ang_y;
float kp = 0.5;
float ki = 0.09;
float throttle = 1400;
float setpoint = 0.0;
float output = 0.0;
float error, up, ui;
bool timerup = false;

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
  timerup = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void starMotors()
{
  for (int dutycycle = 500; dutycycle < throttle + 50; dutycycle += 50)
  {
    motorLeft.writeMicroseconds(dutycycle);
    motorRigth.writeMicroseconds(dutycycle);
    delay(500);
  }
}

void setup()
{
  Serial.begin(115200);

  initMPU();

  motorRigth.attach(25);
  motorLeft.attach(26);

  Serial.println("empezando motores");
  delay(6000);

  starMotors();
  Serial.println("Motores iniciados");
  delay(3000);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100000, true);
  timerAlarmEnable(timer);
}

void loop()
{

  if (timerup)
  {

    mpu.getEvent(&a, &g, &temp);
    accel_ang_y = atan(a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * (180.0 / 3.14);

    error = setpoint - accel_ang_y;
    up = kp * error;
    ui = ui + ki * error;
    output = up + ui;
    output = constrain(output, -1000, 1000);
    ui = constrain(ui, -1000, 1000);

    pwmLeft = throttle + output;
    pwmRight = throttle - output;

    pwmLeft = constrain(pwmLeft, 1000, 2000);
    pwmRight = constrain(pwmRight, 1000, 2000);

    motorRigth.writeMicroseconds((int)pwmRight);
    motorLeft.writeMicroseconds((int)pwmLeft);
    // Serial.printf("el valor del setpoint -> %4.2f\n", setpoint);
    Serial.printf("el valor de angulo -> %4.2f\n", output);
    Serial.printf("el valor de pwmRigth -> %i\n", (int)pwmRight);
    Serial.printf("el valor de pwmLeft -> %i\n", (int)pwmLeft);
    timerup = false;
  }
}