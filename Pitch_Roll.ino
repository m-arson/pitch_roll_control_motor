#include <Wire.h>
#include <MPU6050.h>

#define FORWARD 0
#define REVERSE 1
#define STOP 2

#define MOTOR_FORWARD 8
#define MOTOR_BACKWARD 9

const int inc_val = 3;

MPU6050 mpu;
int prev_val = -9999;
int now_val = 0;
int next_val = 0;

void setup() {
  Serial.begin(115200);

  Serial.println("Initialize MPU6050");
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  pinMode(7, INPUT_PULLUP);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
}

void loop() {
  if (digitalRead(7) == LOW) {
    int *get_pitch_roll = cpr();
    now_val = *get_pitch_roll;

    if (prev_val <= -9999)
      next_val = now_val + inc_val;
    else
      next_val = now_val + inc_val - (now_val - prev_val);

    while (1) {
      int *check_pitch_roll = cpr();
      if (*check_pitch_roll == next_val) {
        Serial.println("OK Doki");
        run_motor(STOP);
        prev_val = next_val;
        break;
      }
      else if (*check_pitch_roll > next_val) {
        run_motor(REVERSE);
      }
      else {
        run_motor(FORWARD);
        Serial.println(*get_pitch_roll);
      }
    }
  }
}
int *cpr() {
  int p = 0, r = 0;
  static int arr[2];
  for (int i = 0; i < 15; ++i) {
    Vector normAccel = mpu.readNormalizeAccel();
    int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis * normAccel.YAxis + normAccel.ZAxis * normAccel.ZAxis)) * 180.0) / M_PI;
    int roll = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;
    p = p + pitch;
    r = r + roll;
  }
  p = p / 15;
  r = r / 15;
  arr[0] = p;
  arr[1] = r;

  return arr;
}
void run_motor(byte status) {
  if (status == REVERSE) {
    digitalWrite(MOTOR_FORWARD, HIGH);
    digitalWrite(MOTOR_BACKWARD, LOW);
  }
  else if (status == STOP) {
    digitalWrite(MOTOR_FORWARD, LOW);
    digitalWrite(MOTOR_BACKWARD, LOW);
  }
  else {
    digitalWrite(MOTOR_FORWARD, LOW);
    digitalWrite(MOTOR_BACKWARD, HIGH);
  }
}
