#include <bluefruit.h>

#include <nrf.h>

#define SERVO0_PIN 13
#define SERVO1_PIN 16
#define SERVO2_PIN 14
#define SERVO3_PIN 15

#define SERVO0_DUTY_IDX 0
#define SERVO1_DUTY_IDX 1
#define SERVO2_DUTY_IDX 2
#define SERVO3_DUTY_IDX 3

#define B_IA_PIN 2
#define B_IB_PIN 3
#define A_IA_PIN 4
#define A_IB_PIN 5

#define B_IA_DUTY_IDX 0
#define B_IB_DUTY_IDX 1
#define A_IA_DUTY_IDX 2
#define A_IB_DUTY_IDX 3

#define PWM0_COUNTERTOP 16000
#define PWM1_COUNTERTOP 20000

typedef struct
{
  uint8_t target_angle;
  uint8_t current_angle;
  uint8_t speed;
} servo_ctrl_t;

static uint16_t pwm0_seq[4];
static uint16_t pwm1_seq[4];

static servo_ctrl_t servo_ctrl[] = {{90, 90, 0}, {90, 90, 0}, {90, 90, 0}, {90, 90, 0}};
static SoftwareTimer servo_timer;

static void start_pwm0(void);
static uint32_t calc_pwm0_comp(uint8_t duty);
static void set_pwm0_duty(uint8_t pin, uint8_t duty);

static void start_pwm1(void);
static uint32_t calc_pwm1_comp(uint8_t angle);
static void set_pwm1_duty(uint8_t pin, uint8_t duty);

static void servo_write(uint8_t servo, uint8_t angle);
static void servoEvent(TimerHandle_t xTimerID);
static void servoTimerStart();

static void start_pwm0(void)
{
  pwm0_seq[B_IA_DUTY_IDX] = calc_pwm0_comp(0);
  pwm0_seq[B_IB_DUTY_IDX] = calc_pwm0_comp(0);
  pwm0_seq[A_IA_DUTY_IDX] = calc_pwm0_comp(0);
  pwm0_seq[A_IB_DUTY_IDX] = calc_pwm0_comp(0);

  NRF_PWM0->PSEL.OUT[0] = (B_IA_PIN << PWM_PSEL_OUT_PIN_Pos) |
                          (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  NRF_PWM0->PSEL.OUT[1] = (B_IB_PIN << PWM_PSEL_OUT_PIN_Pos) |
                          (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  NRF_PWM0->PSEL.OUT[2] = (A_IA_PIN << PWM_PSEL_OUT_PIN_Pos) |
                          (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  NRF_PWM0->PSEL.OUT[3] = (A_IB_PIN << PWM_PSEL_OUT_PIN_Pos) |
                          (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

  NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
  NRF_PWM0->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
  NRF_PWM0->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos);
  NRF_PWM0->COUNTERTOP = (PWM0_COUNTERTOP << PWM_COUNTERTOP_COUNTERTOP_Pos); //1 msec
  NRF_PWM0->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
  NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
                      (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
  NRF_PWM0->SEQ[0].PTR = ((uint32_t)(pwm0_seq) << PWM_SEQ_PTR_PTR_Pos);
  NRF_PWM0->SEQ[0].CNT = ((sizeof(pwm0_seq) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
  NRF_PWM0->SEQ[0].REFRESH = 0;
  NRF_PWM0->SEQ[0].ENDDELAY = 0;
  NRF_PWM0->TASKS_SEQSTART[0] = 1;
}

static uint32_t calc_pwm0_comp(uint8_t duty)
{
  uint32_t comp = PWM0_COUNTERTOP - (PWM0_COUNTERTOP * duty) / 100;
  return comp;
}

static void set_pwm0_duty(uint8_t pin, uint8_t duty)
{
  switch (pin)
  {
  case B_IA_PIN:
    pwm0_seq[B_IA_DUTY_IDX] = calc_pwm0_comp(duty);
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
    break;
  case B_IB_PIN:
    pwm0_seq[B_IB_DUTY_IDX] = calc_pwm0_comp(duty);
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
    break;
  case A_IA_PIN:
    pwm0_seq[A_IA_DUTY_IDX] = calc_pwm0_comp(duty);
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
    break;
  case A_IB_PIN:
    pwm0_seq[A_IB_DUTY_IDX] = calc_pwm0_comp(duty);
    NRF_PWM0->TASKS_SEQSTART[0] = 1;
    break;
  }
}

static void start_pwm1(void)
{
  pwm1_seq[SERVO0_DUTY_IDX] = calc_pwm1_comp(90);
  pwm1_seq[SERVO1_DUTY_IDX] = calc_pwm1_comp(90);
  pwm1_seq[SERVO2_DUTY_IDX] = calc_pwm1_comp(90);
  pwm1_seq[SERVO3_DUTY_IDX] = calc_pwm1_comp(90);

  NRF_PWM1->PSEL.OUT[0] = (SERVO0_PIN << PWM_PSEL_OUT_PIN_Pos) |
                          (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  NRF_PWM1->PSEL.OUT[1] = (SERVO1_PIN << PWM_PSEL_OUT_PIN_Pos) |
                          (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  NRF_PWM1->PSEL.OUT[2] = (SERVO2_PIN << PWM_PSEL_OUT_PIN_Pos) |
                          (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);
  NRF_PWM1->PSEL.OUT[3] = (SERVO3_PIN << PWM_PSEL_OUT_PIN_Pos) |
                          (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);

  NRF_PWM1->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
  NRF_PWM1->MODE = (PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos);
  NRF_PWM1->PRESCALER = (PWM_PRESCALER_PRESCALER_DIV_16 << PWM_PRESCALER_PRESCALER_Pos);
  NRF_PWM1->COUNTERTOP = (PWM1_COUNTERTOP << PWM_COUNTERTOP_COUNTERTOP_Pos); //20 msec
  NRF_PWM1->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);
  NRF_PWM1->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
                      (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos);
  NRF_PWM1->SEQ[0].PTR = ((uint32_t)(pwm1_seq) << PWM_SEQ_PTR_PTR_Pos);
  NRF_PWM1->SEQ[0].CNT = ((sizeof(pwm1_seq) / sizeof(uint16_t)) << PWM_SEQ_CNT_CNT_Pos);
  NRF_PWM1->SEQ[0].REFRESH = 0;
  NRF_PWM1->SEQ[0].ENDDELAY = 0;
  NRF_PWM1->TASKS_SEQSTART[0] = 1;
}

static uint32_t calc_pwm1_comp(uint8_t angle)
{
  //0.5ms:0度, 1.45ms:90度, 2.4ms:180度
  uint32_t comp;
  comp = map(angle, 0, 180, 500, 2400);

  return PWM1_COUNTERTOP - comp;
}

static void set_pwm1_duty(uint8_t pin, uint8_t duty)
{
  switch (pin)
  {
  case SERVO0_PIN:
  {
    pwm1_seq[SERVO0_DUTY_IDX] = calc_pwm1_comp(duty);
    NRF_PWM1->TASKS_SEQSTART[0] = 1;
    break;
  }
  case SERVO1_PIN:
  {
    pwm1_seq[SERVO1_DUTY_IDX] = calc_pwm1_comp(duty);
    NRF_PWM1->TASKS_SEQSTART[0] = 1;
    break;
  }
  case SERVO2_PIN:
  {
    pwm1_seq[SERVO2_DUTY_IDX] = calc_pwm1_comp(duty);
    NRF_PWM1->TASKS_SEQSTART[0] = 1;
    break;
  }
  case SERVO3_PIN:
  {
    pwm1_seq[SERVO3_DUTY_IDX] = calc_pwm1_comp(duty);
    NRF_PWM1->TASKS_SEQSTART[0] = 1;
    break;
  }
  }
}

void motor_onboard_init(void)
{
  pinMode(B_IA_PIN, OUTPUT);
  pinMode(B_IB_PIN, OUTPUT);
  pinMode(A_IA_PIN, OUTPUT);
  pinMode(A_IB_PIN, OUTPUT);

  digitalWrite(B_IA_PIN, LOW);
  digitalWrite(B_IB_PIN, LOW);
  digitalWrite(A_IA_PIN, LOW);
  digitalWrite(A_IB_PIN, LOW);

  pinMode(SERVO0_PIN, OUTPUT);
  pinMode(SERVO1_PIN, OUTPUT);
  pinMode(SERVO2_PIN, OUTPUT);
  pinMode(SERVO3_PIN, OUTPUT);

  digitalWrite(SERVO0_PIN, LOW);
  digitalWrite(SERVO1_PIN, LOW);
  digitalWrite(SERVO2_PIN, LOW);
  digitalWrite(SERVO3_PIN, LOW);

  start_pwm0();
  start_pwm1();

  servoTimerStart();
}

void servo_control_onboard(uint8_t servo, uint8_t angle, uint8_t speed)
{
  Serial.print("servo:");
  Serial.print(servo);
  Serial.print("\tangle:");
  Serial.print(angle);
  Serial.print("\tspeed:");
  Serial.print(speed);
  Serial.println("");

  servo_ctrl[servo].target_angle = angle;
  servo_ctrl[servo].speed = speed;
}

void motor_control_onboard(uint8_t motor, uint8_t speed)
{
  int _speed = (int8_t)speed;

  Serial.print("motor:");
  Serial.print(motor);
  Serial.print("\tspeed:");
  Serial.print(_speed);
  Serial.println("");

  if (motor == 0)
  {
    if (_speed > 0)
    {
      set_pwm0_duty(A_IA_PIN, _speed);
      set_pwm0_duty(A_IB_PIN, 0);
    }
    else if (_speed == 0)
    {
      set_pwm0_duty(A_IA_PIN, 0);
      set_pwm0_duty(A_IB_PIN, 0);
    }
    else
    {
      set_pwm0_duty(A_IA_PIN, 0);
      set_pwm0_duty(A_IB_PIN, -_speed);
    }
  }
  else if (motor == 1)
  {
    if (_speed > 0)
    {
      set_pwm0_duty(B_IA_PIN, _speed);
      set_pwm0_duty(B_IB_PIN, 0);
    }
    else if (_speed == 0)
    {
      set_pwm0_duty(B_IA_PIN, 0);
      set_pwm0_duty(B_IB_PIN, 0);
    }
    else
    {
      set_pwm0_duty(B_IA_PIN, 0);
      set_pwm0_duty(B_IB_PIN, -_speed);
    }
  }
}

static void servo_write(uint8_t servo, uint8_t angle)
{
  if (servo == 0)
  {
    set_pwm1_duty(SERVO0_PIN, angle);
  }
  else if (servo == 1)
  {
    set_pwm1_duty(SERVO1_PIN, angle);
  }
  else if (servo == 2)
  {
    set_pwm1_duty(SERVO2_PIN, angle);
  }
  else if (servo == 3)
  {
    set_pwm1_duty(SERVO3_PIN, angle);
  }
};

static void servoEvent(TimerHandle_t xTimerID)
{
  for (int i; i < sizeof(servo_ctrl) / sizeof(servo_ctrl[0]); i++)
  {
    if (servo_ctrl[i].target_angle > servo_ctrl[i].current_angle)
    {
      servo_ctrl[i].current_angle++;
      servo_write(i, servo_ctrl[i].current_angle);
    }
    else if (servo_ctrl[i].target_angle < servo_ctrl[i].current_angle)
    {
      servo_ctrl[i].current_angle--;
      servo_write(i, servo_ctrl[i].current_angle);
    }
  }
}

static void servoTimerStart()
{
  servo_timer.begin(50, servoEvent);
  servo_timer.start();
}