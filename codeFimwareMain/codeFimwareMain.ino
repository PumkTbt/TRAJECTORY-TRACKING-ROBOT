#define READ_ENC       'e'
#define MOTOR_SPEEDS   'm'
#define MOTOR_RAW_PWM  'o'
#define RESET_ENC      'r'
#define LEFT            0
#define RIGHT           1

#define BAUDRATE     57600
#define MAX_PWM        255

#define LEFT_ENC_A PD2  
#define LEFT_ENC_B PD3  
#define RIGHT_ENC_A PC4  
#define RIGHT_ENC_B PC5  

#define RIGHT_MOTOR_1   5
#define LEFT_MOTOR_1    6
#define RIGHT_MOTOR_0   9
#define LEFT_MOTOR_0    10
#define RIGHT_MOTOR_EN  12
#define LEFT_MOTOR_EN   13

volatile long left_enc_pos = 0L;    //Lưu vị trí của encoder trái và phải
volatile long right_enc_pos = 0L;
static const int8_t enc_state [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //Bộ mã hóa trạng thái encoder
// ---------------------------------- Ngắt đọc giá trị encoder -------------------------------------------
ISR (PCINT2_vect){
  static uint8_t enc_last=0;      // Biến lưu trạng thái encoder 
  enc_last <<=2;                  // Dịch 2 bit biến lưu trạng thái mới vào
  //Cập nhật giá trị 
  enc_last |= (PIND & (3 << 2)) >> 2;     
  left_enc_pos += enc_state[(enc_last & 0x0f)]; 
}

ISR (PCINT1_vect){
  static uint8_t enc_last=0;
  enc_last <<=2; 
  enc_last |= (PINC & (3 << 4)) >> 4;
  right_enc_pos += enc_state[(enc_last & 0x0f)];
}

// ------------ Đọc giá trị encoder ------------------------
long readEncoder(int i) {
  if (i == LEFT) 
      return left_enc_pos;
  else 
      return right_enc_pos;
}

// ---------- Reset lại giá trị encoder ---------------------
void resetEncoder(int i) {
  if (i == LEFT){
    left_enc_pos=0L;
    return;
  } else { 
    right_enc_pos=0L;
    return;
  }
}

void resetEncoder_LR() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}
// -------------------- Cài đặt tốc độ cho motor ----------------------------------------------------
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;
  
    if (spd < 0)
    {
      spd = -spd;
      reverse = 1;
    }
    if (spd > 255)
      spd = 255;
    
    if (i == LEFT) { 
      if (reverse == 0) { 
          analogWrite(LEFT_MOTOR_0, spd); 
          analogWrite(LEFT_MOTOR_1, 0); 
      }
      else if (reverse == 1) { 
          analogWrite(LEFT_MOTOR_1, spd); 
          analogWrite(LEFT_MOTOR_0, 0); 
      }
    }
    else {
      if (reverse == 0) {
          analogWrite(RIGHT_MOTOR_0, spd); 
          analogWrite(RIGHT_MOTOR_1, 0); 
      }
      else if (reverse == 1) { 
          analogWrite(RIGHT_MOTOR_1, spd); 
          analogWrite(RIGHT_MOTOR_0, 0); 
      }
    }
  }
  
  void setMotorSpeed_LR(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
// ------------------------------------------------------------------------

#include <Servo.h>
#include "servos.h"
#include "turne_PID.h"
#define PID_RATE           30     // Hz
const int PID_INTERVAL = 1000 / PID_RATE;
unsigned long nextPID = PID_INTERVAL;
#define tg_stop 2000          // tg dừng động cơ = 2000 ms
long prev_command = tg_stop;

int arg = 0;
int index = 0;
char chr;
char cmd;
char argv1[16];     //Biến lưu trữ giá trị nhận được từ Raspberry Pi
char argv2[16];
long arg1;
long arg2;

// ----------------- Reset command ------------------------
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}
// --------------- Khởi chạy giao tiếp command -------------------
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1); // Chuyển đổi chuỗi ký tự nhận được sang kiểu dữ liệu int
  arg2 = atoi(argv2);
  
  switch(cmd) {
  // ----------- Lệnh đọc encoder --------------------------------
  case READ_ENC:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
  // -----------  ----------- Lệnh reset giá trị encoder và PID của phiên làm việc trước đó --------------------------------
  case RESET_ENC:
    resetEncoder_LR();
    resetPID();
    Serial.println("OK"); // In ra dòng text "OK" sau khi tiến trình thành công
    break;
  // ----------------------- Lệnh set tốc độ motor ---------------------------------
  case MOTOR_SPEEDS:
    prev_command = millis();
    if (arg1 == 0 && arg2 == 0) {
      setMotorSpeed_LR(0, 0);
      resetPID();
      moving= 0;
    }
    else moving= 1;
    leftPID.target = arg1;
    rightPID.target = arg2;
    Serial.println("OK"); 
    break;

  default:
    Serial.println("Nhap lenh!");
    break;
  }
}
// ------------------ Setup ----------------------------------
void setup() {
  // ------------------ Cấu hình serial ----------------------
    Serial.begin(BAUDRATE);
  // ------------------ Cấu hình chân đọc cảm biến encoder -------------------------
    pinMode(LEFT_ENC_A, INPUT);
    pinMode(LEFT_ENC_B, INPUT);
    pinMode(RIGHT_ENC_A, INPUT);
    pinMode(RIGHT_ENC_B, INPUT);
  
    digitalWrite(LEFT_ENC_A, HIGH);
    digitalWrite(LEFT_ENC_B, HIGH);
    digitalWrite(RIGHT_ENC_A, HIGH);
    digitalWrite(RIGHT_ENC_B, HIGH);

  // --------------- Cấu hình chân kết nối với motor --------------------------------
    digitalWrite(RIGHT_MOTOR_EN, HIGH);
    digitalWrite(LEFT_MOTOR_EN, HIGH);

  // --------------- Cấu hình ngắt đọc giá trị encoder ----------------------------  
    PCMSK2 |= (1 << LEFT_ENC_A)|(1 << LEFT_ENC_B);
    PCMSK1 |= (1 << RIGHT_ENC_A)|(1 << RIGHT_ENC_B);
    PCICR |= (1 << PCIE1) | (1 << PCIE2);
  
  // --------------- Đưa các giá trị PID về không -----------------------------------
    resetPID();
  // --------------- Cấu hình trạng thái motor -------------------------------------
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
}

void loop() {
  while (Serial.available() > 0) {  // Kiểm tra có dữ liệu truyền qua cổng serial
    chr = Serial.read();    // Đọc giá trị truyền qua cổng serial 
    // -------- Kiểm tra chuỗi truyền qua serial ----------------------
    if (chr == 13) {
      if (arg == 1) 
          argv1[index] = NULL;
      else if (arg == 2) 
          argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
  // ------------- Tiến hành phân tách chuỗi nhận được để lấy giá trị điều khiển động cơ ---------------------
    else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        cmd = chr;
      }
      else if (arg == 1) {
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  // ---------------- Kiểm tra thời gian chờ -------------------
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  // ------------------ Kiểm tra thời gian delay nhận lệnh ---------------
  if ((millis() - prev_command) > tg_stop) {
    setMotorSpeed_LR(0, 0);
    moving= 0;
  }
// -------------- Điều khiển servo ------------------------
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }

}

