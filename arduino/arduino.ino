#include <bluefruit.h>
#include "motor_control.h"
#include "motor_control_onboard.h"
#include "raspi_com.h"

// Device Name: Maximum 30 bytes
#define DEVICE_NAME "LINE Things Trial dev board"

// User service UUID: Change this to your generated service UUID
#define USER_SERVICE_UUID "3fd1e37b-83a3-4691-8a70-dc42cd486ef7"
// User service characteristics
#define WRITE_CHARACTERISTIC_UUID "E9062E71-9E62-4BC6-B0D3-35CDCD9B027B"
#define NOTIFY_CHARACTERISTIC_UUID "62FBD229-6EDD-4D1A-B554-5C4E1BB29169"

// PSDI Service UUID: Fixed value for Developer Trial
#define PSDI_SERVICE_UUID "E625601E-9E55-4597-A598-76018A0D293D"
#define PSDI_CHARACTERISTIC_UUID "26E2B12B-85F0-4F3F-9FDD-91D114270E6E"

#define BUTTON 29
#define LED1 7

typedef enum
{
  LED_WRITE = 0,
  MOTOR_WRITE = 1,
  SERVO_WRITE = 2,
  MOTOR_CONTROL_MODE = 3,
} write_type_t;

typedef enum
{
  MOTOR_CONTROL_AUTO = 0,
  MOTOR_CONTROL_MANUAL = 1,
} motor_control_mode_t;

static uint8_t userServiceUUID[16];
static uint8_t psdiServiceUUID[16];
static uint8_t psdiCharacteristicUUID[16];
static uint8_t writeCharacteristicUUID[16];
static uint8_t notifyCharacteristicUUID[16];

static BLEService userService;
static BLEService psdiService;
static BLECharacteristic psdiCharacteristic;
static BLECharacteristic notifyCharacteristic;
static BLECharacteristic writeCharacteristic;

static volatile int btnAction = 0;

static uint8_t write_cmd[100];
static motor_control_mode_t motor_control_mode = MOTOR_CONTROL_AUTO;
static void ExecLineThingdCmd(uint8_t *write_cmd);

uint8_t angle_target = 90;
uint8_t angle = 90;

SoftwareTimer servo_timer;

// Callback for buzzer control @1khz
void servoEvent(TimerHandle_t xTimerID) {
  if(angle_target!=angle){
    if(angle_target>angle){
      angle++;
    }else{
      angle--;
    }
  }
}

void servoTimerStart() {
  servo_timer.begin(100, servoEvent);
  servo_timer.start();
}

void setup()
{
  servoTimerStart();
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, 0);
  pinMode(BUTTON, INPUT_PULLUP);
  attachInterrupt(BUTTON, buttonAction, CHANGE);

  Bluefruit.begin();
  Bluefruit.setName(DEVICE_NAME);

  setupServices();
  startAdvertising();

  Serial.begin(115200);
  Serial.println("Start");
  motor_init();
  motor_onboard_init();
}

void loop()
{
  uint8_t btnRead;
  while (btnAction > 0)
  {
    btnRead = !digitalRead(BUTTON);
    btnAction = 0;
    notifyCharacteristic.notify(&btnRead, sizeof(btnRead));
    delay(20);
  }

  ExecUartCmd();
  ExecLineThingdCmd(write_cmd);
}

void ExecUartCmdCallback(uint8_t cmd, uint8_t a, int b)
{
  // Serial.print("motor: ");
  // Serial.print(motor);
  // Serial.print("\tspeed: ");
  // Serial.println(speed);

  if (cmd == 'm')
  {
    if (motor_control_mode == MOTOR_CONTROL_AUTO)
    {
      uint8_t motor = a;
      uint8_t speed = (uint8_t)b;
      cmd_motor(motor, speed);
      motor_control_onboard(motor, speed);
    }
  }

  if (cmd == 's')
  {
    uint8_t servo = a;
    uint8_t angle = (uint8_t)b;
    cmd_servo(servo, angle, 30);
    servo_control_onboard(servo, angle, 30);
  }
}

static void ExecLineThingdCmd(uint8_t *write_cmd)
{
  uint8_t len = write_cmd[0];
  if (len > 0)
  {
    uint8_t *data = &write_cmd[1];
    write_type_t type = (write_type_t)data[0];

    // for (int i = 0; i < len; i++)
    // {
    //   Serial.print(data[i], HEX);
    //   Serial.print(":");
    // }
    // Serial.println("");

    switch (type)
    {
    case LED_WRITE:
    {
      uint8_t state;
      static uint8_t state_old = 0xff;
      if (len != 2)
        break;

      state = (data[1] == 0) ? 0 : 1;

      if (state_old != state)
      {
        Serial.print("LED:");
        Serial.println(state);
        digitalWrite(LED1, state);
      }
      state_old = state;
      break;
    }
    case MOTOR_WRITE:
    {
      if (len != 3)
        break;
      if (motor_control_mode == MOTOR_CONTROL_MANUAL)
      {
        static uint8_t motor_no_old = 0xff;
        static uint8_t speed_old = 0xff;
        uint8_t motor_no = data[1];
        uint8_t speed = data[2];

        if(1)
        // if ((motor_no != motor_no_old) || (speed != speed_old))
        {
          cmd_motor(motor_no, speed);
          motor_control_onboard(motor_no, speed);
        }
        motor_no_old = motor_no;
        speed_old = speed;
      }
      break;
    }
    case SERVO_WRITE:
    {
      if (len != 4)
        break;
      uint8_t servo_no = data[1];
      angle_target = data[2];
      uint8_t speed = data[3];
      static uint8_t servo_no_old = 0xff;
      static uint8_t angle_old = 0xff;
      static uint8_t speed_old = 0xff;

      if(1)
      // if ((servo_no_old != servo_no) || (angle_old != angle) || (speed_old != speed))
      {
        cmd_servo(servo_no, angle, speed);
        servo_control_onboard(servo_no, angle, speed);
      }
      servo_no_old = servo_no;
      angle_old = angle;
      speed_old = speed;
      break;
    }
    case MOTOR_CONTROL_MODE:
    {
      uint8_t state;
      static uint8_t state_old = 0xff;
      if (len != 2)
        break;

      state = (data[1] == 0) ? 0 : 1;
      if (state_old != state)
      {
        if (state == 0)
        {
          motor_control_mode = MOTOR_CONTROL_AUTO;
          Serial.println("MOTOR_CONTROL_AUTO");
        }
        else
        {
          motor_control_mode = MOTOR_CONTROL_MANUAL;
          Serial.println("MOTOR_CONTROL_MANUAL");
        }
      }
      state_old = state;
      break;
    }
    }
  }
}

void setupServices(void)
{
  // Convert String UUID to raw UUID bytes
  strUUID2Bytes(USER_SERVICE_UUID, userServiceUUID);
  strUUID2Bytes(PSDI_SERVICE_UUID, psdiServiceUUID);
  strUUID2Bytes(PSDI_CHARACTERISTIC_UUID, psdiCharacteristicUUID);
  strUUID2Bytes(WRITE_CHARACTERISTIC_UUID, writeCharacteristicUUID);
  strUUID2Bytes(NOTIFY_CHARACTERISTIC_UUID, notifyCharacteristicUUID);

  // Setup User Service
  userService = BLEService(userServiceUUID);
  userService.begin();

  writeCharacteristic = BLECharacteristic(writeCharacteristicUUID);
  writeCharacteristic.setProperties(CHR_PROPS_WRITE);
  writeCharacteristic.setWriteCallback(writeLEDCallback);
  writeCharacteristic.setPermission(SECMODE_ENC_NO_MITM, SECMODE_ENC_NO_MITM);
  // writeCharacteristic.setFixedLen(1);
  writeCharacteristic.begin();

  notifyCharacteristic = BLECharacteristic(notifyCharacteristicUUID);
  notifyCharacteristic.setProperties(CHR_PROPS_NOTIFY);
  notifyCharacteristic.setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
  notifyCharacteristic.setFixedLen(1);
  notifyCharacteristic.begin();

  // Setup PSDI Service
  psdiService = BLEService(psdiServiceUUID);
  psdiService.begin();

  psdiCharacteristic = BLECharacteristic(psdiCharacteristicUUID);
  psdiCharacteristic.setProperties(CHR_PROPS_READ);
  psdiCharacteristic.setPermission(SECMODE_ENC_NO_MITM, SECMODE_NO_ACCESS);
  psdiCharacteristic.setFixedLen(sizeof(uint32_t) * 2);
  psdiCharacteristic.begin();

  // Set PSDI (Product Specific Device ID) value
  uint32_t deviceAddr[] = {NRF_FICR->DEVICEADDR[0], NRF_FICR->DEVICEADDR[1]};
  psdiCharacteristic.write(deviceAddr, sizeof(deviceAddr));
}

void startAdvertising(void)
{
  // Start Advertising
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(userService);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.start(0);
}

void buttonAction()
{
  btnAction++;
}

void writeLEDCallback(uint16_t conn_hdl, BLECharacteristic *chr, uint8_t *data, uint16_t len)
{
  for (int i = 0; i < len; i++)
  {
    write_cmd[i + 1] = data[i];
  }
  write_cmd[0] = len;
}

// UUID Converter
void strUUID2Bytes(String strUUID, uint8_t binUUID[])
{
  String hexString = String(strUUID);
  hexString.replace("-", "");

  for (int i = 16; i != 0; i--)
  {
    binUUID[i - 1] = hex2c(hexString[(16 - i) * 2], hexString[((16 - i) * 2) + 1]);
  }
}

char hex2c(char c1, char c2)
{
  return (nibble2c(c1) << 4) + nibble2c(c2);
}

char nibble2c(char c)
{
  if ((c >= '0') && (c <= '9'))
    return c - '0';
  if ((c >= 'A') && (c <= 'F'))
    return c + 10 - 'A';
  if ((c >= 'a') && (c <= 'f'))
    return c + 10 - 'a';
  return 0;
}