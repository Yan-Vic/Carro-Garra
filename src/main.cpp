#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <vector>

const int NUM_KEYS = 3;
const int buttonPins[NUM_KEYS] = {5, 18, 19};
const int BUZZER_PIN = 23;
const int BUZZER_CHANNEL = 0;
const int Servo1 = 25;
const int Servo2 = 26;
const int Garra = 27;

#define Bt_Iniciar 32;

// put function declarations here:
int myFunction(int, int);

void setup()
{
  // put your setup code here, to run once:
}

void loop()
{
  // put your main code here, to run repeatedly:
}