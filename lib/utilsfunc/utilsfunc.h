#ifndef UTILSFUNC_H
#define UTILSFUNC_H

//  1) Definição de pinos
#define BT_P1 5
#define BT_P2 18
#define BT_P3 19
#define BT_POWER_ERROR 35

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
extern int currentStation;
extern int targetStation;

extern bool M1, M2;

//  3) Estados da máquina
enum State
{
  REST,
  OPEN_GRIPPER,
  MOVING,
  CLOSE_GRIPPER
};
extern State state;

//  4) Configuração do motor de passo
static constexpr int stepsPerRevolution = 200;
extern Stepper movementStepper;//(stepsPerRevolution, STEPPER_PIN_1, STEPPER_PIN_2, STEPPER_PIN_3, STEPPER_PIN_4);
extern Servo gripperServo;

//  5) Display LCD
extern LiquidCrystal_I2C lcd;

//  6) Funções
void checkButtons();
void openGripper();
void moveToTarget();
void closeGripper();
void errorCondition();
#endif