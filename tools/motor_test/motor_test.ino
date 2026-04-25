// Motor test — Teensy 4.0
// ARM0: Channel A (AIN1=8, AIN2=9, PWMA=5)
// ARM1: Channel B (BIN1=11, BIN2=12, PWMB=6)
// Shared STBY=10

#define AIN1  8
#define AIN2  9
#define PWMA  5
#define BIN1  11
#define BIN2  12
#define PWMB  6
#define STBY  10

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);  // take out of standby
}

// speed: 0–255 | dir: 1=forward, -1=reverse, 0=brake
void driveA(int speed, int dir) {
  if (dir == 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, 0);
    return;
  }
  digitalWrite(AIN1, dir == 1 ? HIGH : LOW);
  digitalWrite(AIN2, dir == 1 ? LOW  : HIGH);
  analogWrite(PWMA, constrain(speed, 0, 255));
}

void driveB(int speed, int dir) {
  if (dir == 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 0);
    return;
  }
  digitalWrite(BIN1, dir == 1 ? HIGH : LOW);
  digitalWrite(BIN2, dir == 1 ? LOW  : HIGH);
  analogWrite(PWMB, constrain(speed, 0, 255));
}

void loop() {
  driveA(128, 1);  driveB(128, 1);   // both forward 50%
  delay(2000);
  driveA(255, 1);  driveB(255, 1);   // both forward 100%
  delay(2000);
  driveA(0,  0);   driveB(0,  0);    // brake
  delay(500);
  driveA(192, -1); driveB(192, -1);  // both reverse 75%
  delay(2000);
  digitalWrite(STBY, LOW);           // coast
  delay(1000);
  digitalWrite(STBY, HIGH);          // wake
}
