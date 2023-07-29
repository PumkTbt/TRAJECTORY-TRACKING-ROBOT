SweepServo::SweepServo()
{
  this->currentPositionDegrees = 0;
  this->targetPositionDegrees = 0;
  this->lastSweepCommand = 0;
}
// ------------- Khởi tạo đối tượng quản lí servo -------------------
void SweepServo::initServo(
    int servoPin,
    int stepDelayMs,
    int initPosition)
{
  this->servo.attach(servoPin);
  this->stepDelayMs = stepDelayMs;
  this->currentPositionDegrees = initPosition;
  this->targetPositionDegrees = initPosition;
  this->lastSweepCommand = millis();
}
// --------- Khởi chạy servo ------------------
void SweepServo::doSweep()
{
// ----------- Tính khoảng thời gian sweep servo
  int delta = millis() - this->lastSweepCommand;

  if (delta > this->stepDelayMs) {
    if (this->targetPositionDegrees > this->currentPositionDegrees) {
      this->currentPositionDegrees++;
      this->servo.write(this->currentPositionDegrees);
    }
    else if (this->targetPositionDegrees < this->currentPositionDegrees) {
      this->currentPositionDegrees--;
      this->servo.write(this->currentPositionDegrees);
    }
    // ----------------- Reset thời gian -------------------------
    this->lastSweepCommand = millis();
  }
}

// ------- Đặt lại điểm mục tiêu mới --------------------------
void SweepServo::setTargetPosition(int position)
{
  this->targetPositionDegrees = position;
}

Servo SweepServo::getServo()
{
  return this->servo;
}