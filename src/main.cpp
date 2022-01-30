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
const int motorRigthPin = 25;
const int motorLeftPin = 26;
const int mixSpeed = 1000;
const int maxSpeed = 2000;
float throttle = 1400;

#define RAD_A_DEG  57.295779  // 180/PI
#define Gyro_Factor 131.0 //
float accel_ang_y, ang_y;
float ang_y_prev;

float kp = 0.5;
float ki = 0.09;
float setpoint = 0.0;
float output = 0.0;
float error, up, ui;

#define Ts 0.01
#define timeFactor 1000000
bool timerup = false;

float alpha = 0.98;


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

  motorRigth.attach(motorRigthPin);
  motorLeft.attach(motorLeftPin);

  Serial.println("empezando motores");
  // delay(5000); //TODO:

  // starMotors();
  Serial.println("Motores iniciados");
  // delay(3000); //TODO:

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, Ts*timeFactor, true);
  timerAlarmEnable(timer);
}

void loop()
{

  if (timerup)
  {

    mpu.getEvent(&a, &g, &temp);

    accel_ang_y = atan(a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * RAD_A_DEG;
    ang_y = alpha * (ang_y_prev + (g.gyro.y / Gyro_Factor) * Ts) + (1-alpha) * accel_ang_y;

    error = setpoint - accel_ang_y;
    up = kp * error;
    ui = ui + ki * error;
    output = up + ui;
    output = constrain(output, -mixSpeed, maxSpeed);
    ui = constrain(ui, -mixSpeed, maxSpeed);

    pwmLeft = throttle + output;
    pwmRight = throttle - output;

    pwmLeft = constrain(pwmLeft, mixSpeed, maxSpeed);
    pwmRight = constrain(pwmRight, mixSpeed, maxSpeed);

    motorRigth.writeMicroseconds((int)pwmRight);
    motorLeft.writeMicroseconds((int)pwmLeft);

    ang_y_prev = ang_y;
    Serial.printf("el valor del setpoint -> %4.2f\n", ang_y);
    Serial.printf("el valor de pwmRigth -> %i\n", (int)pwmRight);
    Serial.printf("el valor de pwmLeft -> %i\n", (int)pwmLeft);
    timerup = false;
  }
}