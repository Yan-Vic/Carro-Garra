#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <Stepper.h>
#include "utilsfunc.h"

//  1) Definição de valores
bool M1 = false, M2 = false;
int currentStation = 2;
int targetStation = 0;
State state = REST;
LiquidCrystal_I2C lcd(0x27, 16, 2);

Stepper movementStepper(
    stepsPerRevolution,
    STEPPER_PIN_1,
    STEPPER_PIN_2,
    STEPPER_PIN_3,
    STEPPER_PIN_4);

Servo gripperServo;

//  2) Leitura dos botões
void checkButtons()
{
  if (digitalRead(BT_P1) == LOW)
  {
    targetStation = 1;
    M1 = true;
  }
  else if (digitalRead(BT_P2) == LOW)
  {
    targetStation = 2;
    M2 = true;
  }
  else if (digitalRead(BT_P3) == LOW)
  {
    targetStation = 3;
  }
  else
    return;

  if (targetStation != currentStation)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Selecionado P");
    lcd.print(targetStation);

    Serial.printf("Selecionado P%d\n\r", targetStation);
    state = CLOSE_GRIPPER;
  }
  else if (targetStation == currentStation && (digitalRead(BT_P1) == LOW || digitalRead(BT_P2) == LOW || digitalRead(BT_P3) == LOW))
  {
    state = MOVING;
  }
}
//  3) Fechar garra
void closeGripper()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fechando Garra...");
  Serial.println("Fechando garra...\n\r");
  gripperServo.write(90);
  delay(1000);
  state = MOVING;
}

//  4) Mover o motor de passo
void moveToTarget()
{
  // Emular erro aleatório (~5% de chance)
  if (random(0, 100) < 5)
  {
    errorCondition();
  }

  int delta = targetStation - currentStation;

  if (delta != 0)
  {
    digitalWrite(LED_WAIT, HIGH);
    digitalWrite(LED_P1, LOW);
    digitalWrite(LED_P2, LOW);
    digitalWrite(LED_P3, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printf("Movendo para P%d", targetStation);
    lcd.setCursor(0, 1);
    lcd.print("Aguarde...");
    Serial.printf("Movendo para P%d\n\r", targetStation);

    movementStepper.step(stepsPerRevolution * delta);
    delay(1000);
    currentStation = targetStation;
    Serial.printf("Chegou em P%d\n\r", currentStation);
    state = OPEN_GRIPPER;
  }
  else
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Erro: Selecione");
    lcd.setCursor(0, 1);
    lcd.print("um Local Valido");
    Serial.println("O carro já se encontra no local requesitado");
    delay(1000);
    state = REST;
  }
  if (M1 == true)
  {
    M1 = false;
    digitalWrite(LED_P1, HIGH);
  }
  else if (M2 == true)
  {
    M2 = false;
    digitalWrite(LED_P2, HIGH);
  }
  else if (M1 == false && M2 == false)
  {
    digitalWrite(LED_P3, HIGH);
  }
  digitalWrite(LED_WAIT, LOW);
}

//  5) Abrir a garra
void openGripper()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Abrindo Garra...");
  Serial.println("Abrindo Garra...\n\r");
  gripperServo.write(180);
  delay(1000);
  state = REST;
}

//  6) Verificação de erro
void errorCondition()
{
  // Acende LED de erro e limpa LCD
  digitalWrite(LED_ERRO, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("!!! ERRO FATAL !!!");
  lcd.setCursor(0, 1);
  lcd.print("Sistema parado");
  Serial.println("ERRO FATAL: sistema travado\n\r");

  // Para a execução em loop infinito
  while (true)
  {
    if (digitalRead(BT_POWER_ERROR) == LOW)
    {
      digitalWrite(LED_ERRO, LOW);
      state = REST;
      break;
    }
    delay(500);
  }
}