#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>
#include <Stepper.h>

// ——————————————————————————————————————————————
//  1) Definição de pinos
// ——————————————————————————————————————————————
const int BUZZER_PIN = 23;
const int BUZZER_CHANNEL = 0;

#define BT_P1 5
#define BT_P2 18
#define BT_P3 32 // Corrigido: evitar redefinição

#define STEPPER_PIN_1 26
#define STEPPER_PIN_2 27
#define STEPPER_PIN_3 14
#define STEPPER_PIN_4 12

#define CLAW_SERVO 25

#define I2C_ADDR 0x27
#define LCD_COLUMNS 16
#define LCD_LINES 2

//  2) Mapeamento de estações
enum Station
{
  P1 = 1,
  P2 = 2,
  P3 = 3
};
Station currentStation = P2;
Station targetStation;

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

//  Protótipos
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

  gripperServo.attach(CLAW_SERVO);
  gripperServo.write(0); // posição fechada (corrigido)

  movementStepper.setSpeed(20); // velocidade adequada

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sistema iniciado");
  lcd.setCursor(0, 1);
  lcd.print("Estado: REPOUSO");

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
  case OPEN_GRIPPER:
    openGripper();
    break;
  case MOVING:
    moveToTarget();
    break;
  case CLOSE_GRIPPER:
    closeGripper();
    break;
  }
}

//  a) Leitura dos botões
void checkButtons()
{
  if (digitalRead(BT_P1) == LOW)
    targetStation = P1;
  else if (digitalRead(BT_P2) == LOW)
    targetStation = P2;
  else if (digitalRead(BT_P3) == LOW)
    targetStation = P3;
  else
    return;

  if (targetStation != currentStation)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Selecionado P");
    lcd.print(targetStation);

    Serial.printf("Selecionado P%d\n", targetStation);
    state = OPEN_GRIPPER;
  }
}

// ——————————————————————————————————————————————
//  b) Abrir a garra
// ——————————————————————————————————————————————
void openGripper()
{
  Serial.println("Abrindo garra...");
  gripperServo.write(180);
  delay(500);
  state = MOVING;
}

// ——————————————————————————————————————————————
//  c) Mover o motor de passo
// ——————————————————————————————————————————————
void moveToTarget()
{
  int delta = targetStation - currentStation;

  if (delta > 0)
  {
    Serial.println("Movendo para DIREITA");
    movementStepper.step(stepsPerRevolution * delta);
  }
  else if (delta < 0)
  {
    Serial.println("Movendo para ESQUERDA");
    movementStepper.step(stepsPerRevolution * delta); // negativo
  }

  currentStation = targetStation;
  Serial.printf("Chegou em P%d\n", currentStation);
  state = CLOSE_GRIPPER;
}

// ——————————————————————————————————————————————
//  d) Fechar garra
// ——————————————————————————————————————————————
void closeGripper()
{
  Serial.println("Fechando garra...");
  gripperServo.write(180);
  delay(500);
  state = REST;
}
