#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <Stepper.h>
#include "utilsfunc.h"

//  setup()
void setup()
{
  Serial.begin(115200);

  Wire.begin();
 
// Semente para random()
  randomSeed(analogRead(A0));

  pinMode(BT_P1, INPUT_PULLUP);
  pinMode(BT_P2, INPUT_PULLUP);
  pinMode(BT_P3, INPUT_PULLUP);
  pinMode(BT_POWER_ERROR,INPUT_PULLUP);

  pinMode(LED_P1, OUTPUT);
  pinMode(LED_P2, OUTPUT);
  pinMode(LED_P3, OUTPUT);
  pinMode(LED_WAIT, OUTPUT);
  pinMode(LED_ERRO, OUTPUT);

  M1 = false;
  M2 = false;

  gripperServo.attach(CLAW_SERVO);
  gripperServo.write(90); // posição fechada (corrigido)

  movementStepper.setSpeed(40); // velocidade adequada

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistema iniciado");
  lcd.setCursor(0, 1);
  lcd.print("Carro em P2");

  digitalWrite(LED_P1, LOW);
  digitalWrite(LED_P2, HIGH);
  digitalWrite(LED_P3, LOW);
  digitalWrite(LED_WAIT, LOW);
  digitalWrite(LED_ERRO, LOW);

  Serial.println("Sistema de transporte iniciado. Estado: REPOUSO");
}

//  loop()
void loop()
{
  switch (state)
  {
  case REST:
    checkButtons();
    break;
  case CLOSE_GRIPPER:
    closeGripper();
    break;
  case MOVING:
    moveToTarget();
    break;
  case OPEN_GRIPPER:
    openGripper();
    break;
  }
}