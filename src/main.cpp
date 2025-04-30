#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <Stepper.h>

//  1) Definição de pinos
const int BUZZER_PIN = 23;
const int BUZZER_CHANNEL = 0;

#define BT_P1 5
#define BT_P2 18
#define BT_P3 19

#define LED_P1 17
#define LED_P2 16
#define LED_P3 4
#define LED_WAIT 13
#define LED_ERRO 33

#define STEPPER_PIN_1 26
#define STEPPER_PIN_2 27
#define STEPPER_PIN_3 14
#define STEPPER_PIN_4 12

#define CLAW_SERVO 25

#define I2C_ADDR 0x27
#define LCD_COLUMNS 16
#define LCD_LINES 2

//  2) Mapeamento de estações
int currentStation = 2;
int targetStation;

//  3) Estados da máquina
enum State
{
  REST,
  OPEN_GRIPPER,
  MOVING,
  CLOSE_GRIPPER
};
State state = REST;

//  4) Configuração do motor de passo
const int stepsPerRevolution = 200;
Stepper movementStepper(stepsPerRevolution, STEPPER_PIN_1, STEPPER_PIN_2, STEPPER_PIN_3, STEPPER_PIN_4);
Servo gripperServo;

//  5) Display LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

void checkButtons();
void openGripper();
void moveToTarget();
void closeGripper();

//  setup()
void setup()
{
  Serial.begin(115200);

  Wire.begin();

  pinMode(BT_P1, INPUT_PULLUP);
  pinMode(BT_P2, INPUT_PULLUP);
  pinMode(BT_P3, INPUT_PULLUP);

  pinMode(LED_P1, OUTPUT);
  pinMode(LED_P2, OUTPUT);
  pinMode(LED_P3, OUTPUT);
  pinMode(LED_WAIT, OUTPUT);
  pinMode(LED_ERRO, OUTPUT);

  gripperServo.attach(CLAW_SERVO);
  gripperServo.write(90); // posição fechada (corrigido)

  movementStepper.setSpeed(20); // velocidade adequada

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

//  a) Leitura dos botões
void checkButtons()
{
  if (digitalRead(BT_P1) == LOW)
    targetStation = 1;
  else if (digitalRead(BT_P2) == LOW)
    targetStation = 2;
  else if (digitalRead(BT_P3) == LOW)
    targetStation = 3;
  else
    return;

  if (targetStation != currentStation)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Selecionado P");
    lcd.print(targetStation);

    Serial.printf("Selecionado P%d\n", targetStation);
    state = CLOSE_GRIPPER;
  }
  else if (targetStation == currentStation && (digitalRead(BT_P1) == LOW || digitalRead(BT_P2) == LOW || digitalRead(BT_P3) == LOW))
  {
    state = MOVING;
  }
}
//  b) Fechar garra
void closeGripper()
{
  digitalWrite(LED_P1, LOW);
  digitalWrite(LED_P2, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fechando Garra...");
  Serial.println("Fechando garra...");
  gripperServo.write(90);
  delay(500);
  state = MOVING;
}

//  c) Mover o motor de passo
void moveToTarget()
{
  int delta = targetStation - currentStation;

  if (delta != 0)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.printf("Movendo para P%d\n", targetStation);
    lcd.setCursor(0, 1);
    lcd.print("Aguarde...");
    Serial.printf("Movendo para P%d\n", targetStation);
    movementStepper.step(stepsPerRevolution * delta);
    delay(3000);
    currentStation = targetStation;
    Serial.printf("Chegou em P%d\n", currentStation);
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
}

//  d) Abrir a garra
void openGripper()
{
  Serial.println("Abrindo garra...");
  gripperServo.write(180);
  delay(500);
  state = REST;
}