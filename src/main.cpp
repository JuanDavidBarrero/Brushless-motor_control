#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>

Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

Servo motorRigth;
Servo motorLeft;

void setup()
{
  Serial.begin(115200);

 motorRigth.attach(25);
  motorLeft.attach(26);

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

  motorLeft.writeMicroseconds(1200);
  motorRigth.writeMicroseconds(1200);
  delay(5000);
  Serial.println("Iniciando....");


}

void loop()
{

  mpu.getEvent(&a, &g, &temp);

  float accel_ang_y = atan(a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * (180.0 / 3.14);

  motorRigth.writeMicroseconds(600);
  motorLeft.writeMicroseconds(600);
  Serial.printf("El angulo en y es %4.2f \n", accel_ang_y);

  delay(100);
}