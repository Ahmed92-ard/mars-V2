#include <Arduino.h>

// ================= CONFIG =================
#define BAUDRATE 115200

#define LEFT_PULSE_PIN    2
#define RIGHT_PULSE_PIN   3

#define LEFT_PWM_PIN      9
#define RIGHT_PWM_PIN     10
#define LEFT_DIR_PIN      8
#define RIGHT_DIR_PIN     7
#define LEFT_EN_PIN       6
#define RIGHT_EN_PIN      5

#define MAX_PWM 255

// Deadband (hub motors)
#define MIN_PWM_LEFT   40
#define MIN_PWM_RIGHT  40

#define INVERT_LEFT_MOTOR   false
#define INVERT_RIGHT_MOTOR  true

#define LEFT_MOTOR_SCALE   1.00
#define RIGHT_MOTOR_SCALE  0.94

// ================= ENCODERS =================
volatile long left_ticks  = 0;
volatile long right_ticks = 0;
volatile int left_dir_sign  = 1;
volatile int right_dir_sign = 1;

// ================= SERIAL =================
char rx_buffer[64];
uint8_t rx_index = 0;

// Motor targets (ticks per loop from ROS)
int left_cmd = 0;
int right_cmd = 0;

// ================= ISR ===================
void leftEncoderISR()  { left_ticks  += left_dir_sign; }
void rightEncoderISR() { right_ticks += right_dir_sign; }

// ================= SETUP =================
void setup()
{
  Serial.begin(BAUDRATE);
  delay(1000);

  Serial.println("=================================");
  Serial.println("ARDUINO HUB MOTOR DRIVER (NO PID)");
  Serial.println("=================================");

  pinMode(LEFT_PULSE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_PULSE_PIN, INPUT_PULLUP);

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(LEFT_EN_PIN, OUTPUT);
  pinMode(RIGHT_EN_PIN, OUTPUT);

  digitalWrite(LEFT_EN_PIN, HIGH);
  digitalWrite(RIGHT_EN_PIN, HIGH);

  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);

  attachInterrupt(digitalPinToInterrupt(LEFT_PULSE_PIN),
                  leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_PULSE_PIN),
                  rightEncoderISR, RISING);

  Serial.println("Ready: m L R | e");
}

// ================= LOOP ==================
void loop()
{
  readSerial();
  applyMotor(LEFT_PWM_PIN, LEFT_DIR_PIN,
             left_cmd, &left_dir_sign,
             INVERT_LEFT_MOTOR, MIN_PWM_LEFT, LEFT_MOTOR_SCALE);

  applyMotor(RIGHT_PWM_PIN, RIGHT_DIR_PIN,
             right_cmd, &right_dir_sign,
             INVERT_RIGHT_MOTOR, MIN_PWM_RIGHT, RIGHT_MOTOR_SCALE);
}

// ================= SERIAL =================
void readSerial()
{
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\r' || c == '\n')
    {
      if (rx_index)
      {
        rx_buffer[rx_index] = '\0';
        handleCommand(rx_buffer);
        rx_index = 0;
      }
    }
    else if (rx_index < sizeof(rx_buffer) - 1)
    {
      rx_buffer[rx_index++] = c;
    }
  }
}

void handleCommand(const char *cmd)
{
  if (cmd[0] == 'm')
  {
    sscanf(cmd, "m %d %d", &left_cmd, &right_cmd);
  }
  else if (cmd[0] == 'e')
  {
    sendEncoders();
  }
}

// ================= MOTOR =================
void applyMotor(int pwm_pin, int dir_pin,
                int cmd, volatile int *dir_sign,
                bool invert, int min_pwm, float scale)
{
  if (cmd == 0)
  {
    analogWrite(pwm_pin, 0);
    return;
  }

  bool forward = (cmd > 0);
  int pwm = abs(cmd);

  pwm = (int)(pwm * scale);
  pwm = constrain(pwm, 0, MAX_PWM);
  if (pwm < min_pwm) pwm = min_pwm;

  bool dir = invert ? !forward : forward;
  digitalWrite(dir_pin, dir ? HIGH : LOW);
  analogWrite(pwm_pin, pwm);

  noInterrupts();
  *dir_sign = forward ? 1 : -1;
  interrupts();
}

// ================= TX =====================
void sendEncoders()
{
  long l, r;
  noInterrupts();
  l = left_ticks;
  r = right_ticks;
  interrupts();

  Serial.print(l);
  Serial.print(" ");
  Serial.println(r);
}
