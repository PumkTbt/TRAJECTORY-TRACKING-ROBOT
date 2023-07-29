#define N_SERVOS 2
int stepDelay [N_SERVOS] = { 0, 0 }; 
byte servoPins [N_SERVOS] = { 3, 4 };
byte servoInitPosition [N_SERVOS] = { 90, 90 };

class SweepServo
{
  public:
    SweepServo();
    void initServo(
        int servoPin,
        int stepDelayMs,
        int initPosition);
    void doSweep();
    void setTargetPosition(int position);
    Servo getServo();

  private:
    Servo servo;
    int stepDelayMs;
    int currentPositionDegrees;
    int targetPositionDegrees;
    long lastSweepCommand;
};

SweepServo servos [N_SERVOS];

