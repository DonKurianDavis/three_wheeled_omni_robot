#include <EncoderStepCounter.h>

const int LEFT_WHEEL_RIGHTWARD = 7;
const int LEFT_WHEEL_LEFTWARD = 8;
const int LEFT_WHEEL_ENABLE = 9;

const int RIGHT_WHEEL_RIGHTWARD = 4;
const int RIGHT_WHEEL_LEFTWARD = 5;
const int RIGHT_WHEEL_ENABLE = 6;

const int BACK_WHEEL_RIGHTWARD = 10;
const int BACK_WHEEL_LEFTWARD = 11;
const int BACK_WHEEL_ENABLE = 12;

const int LEFT_ENCODER_A = 18;
const int LEFT_ENCODER_B = 19;
#define LEFT_ENCODER_INT1 digitalPinToInterrupt(LEFT_ENCODER_A)
#define LEFT_ENCODER_INT2 digitalPinToInterrupt(LEFT_ENCODER_B)

const int RIGHT_ENCODER_A = 20;
const int RIGHT_ENCODER_B = 21;
#define RIGHT_ENCODER_INT1 digitalPinToInterrupt(RIGHT_ENCODER_A)
#define RIGHT_ENCODER_INT2 digitalPinToInterrupt(RIGHT_ENCODER_B)

const int BACK_ENCODER_A = 2;
const int BACK_ENCODER_B = 3;
#define BACK_ENCODER_INT1 digitalPinToInterrupt(BACK_ENCODER_A)
#define BACK_ENCODER_INT2 digitalPinToInterrupt(BACK_ENCODER_B)

#define MAX_PWM 255
#define PID_RATE 30
#define AUTO_STOP_INTERVAL 2000

const int PID_INTERVAL = 1000 / PID_RATE;
unsigned long nextPID = PID_INTERVAL;

char incomingData[20];
char *token;
char cmd;
int idx = 0;
int values[4];

unsigned char moving=0;

signed long left_pos = 0;
signed long right_pos = 0;
signed long back_pos = 0;

int Kp = 20;
int Ki = 0;
int Kd = 12;
int Ko = 50;

EncoderStepCounter left_encoder(LEFT_ENCODER_A, LEFT_ENCODER_B);
EncoderStepCounter right_encoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B);
EncoderStepCounter back_encoder(BACK_ENCODER_A, BACK_ENCODER_B);

typedef struct
{
  double TargetTicksPerFrame;
  long Encoder;
  long PrevEnc;
  int PrevInput;
  int ITerm;
  long output;
}
SetPointInfo;

SetPointInfo leftPID, rightPID, backPID;

void setup()
{
  Serial.begin(115200);

  pinMode(LEFT_WHEEL_RIGHTWARD, OUTPUT);
  pinMode(LEFT_WHEEL_LEFTWARD, OUTPUT);
  pinMode(LEFT_WHEEL_ENABLE, OUTPUT);

  pinMode(RIGHT_WHEEL_RIGHTWARD, OUTPUT);
  pinMode(RIGHT_WHEEL_LEFTWARD, OUTPUT);
  pinMode(RIGHT_WHEEL_ENABLE, OUTPUT);

  pinMode(BACK_WHEEL_RIGHTWARD, OUTPUT);
  pinMode(BACK_WHEEL_LEFTWARD, OUTPUT);
  pinMode(BACK_WHEEL_ENABLE, OUTPUT);

  left_encoder.begin();
  right_encoder.begin();
  back_encoder.begin();

  attachInterrupt(LEFT_ENCODER_INT1, L_interrupt, CHANGE);
  attachInterrupt(LEFT_ENCODER_INT2, L_interrupt, CHANGE);

  attachInterrupt(RIGHT_ENCODER_INT1, R_interrupt, CHANGE);
  attachInterrupt(RIGHT_ENCODER_INT2, R_interrupt, CHANGE);

  attachInterrupt(BACK_ENCODER_INT1, B_interrupt, CHANGE);
  attachInterrupt(BACK_ENCODER_INT2, B_interrupt, CHANGE);

  stopMotors();

}

void L_interrupt()
{
  left_encoder.tick();
}

void R_interrupt()
{
  right_encoder.tick();
}

void B_interrupt()
{
  back_encoder.tick();
}

void loop() 
{
  signed char l_pos = left_encoder.getPosition();
  signed char r_pos = right_encoder.getPosition();
  signed char b_pos = back_encoder.getPosition();
  
  if (l_pos != 0)
  {
    left_pos += l_pos;
    left_encoder.reset();
  }

  if (r_pos != 0)
  {
    right_pos += r_pos;
    right_encoder.reset();
  }

  if (b_pos != 0)
  {
    back_pos += b_pos;
    back_encoder.reset();
  }

  while(Serial.available())
  {
    char incomingByte = Serial.read();
    if (incomingByte == '\r')
    {
      incomingData[idx] = '\0';
      idx = 0;

      for(int i = 0; i < 4; i++)
        values[i] = 0;

      token = strtok(incomingData, " ");
      if (token != NULL)
        cmd = token[0];
      
      int i = 0;
      while (token != NULL && i < 4)
      {
        token = strtok(NULL," ");
        if (token != NULL)
        {
          values[i++] = atoi(token);
        }   
      }
      runCommand(cmd,values);
      memset(incomingData, 0, sizeof(incomingData));
    }
    else
    {
      incomingData[idx++] = incomingByte;
      if (idx >= 100)
      {
        idx = 99;
      }
    }
  }
  if (millis() > nextPID)
  {
    updatePID();
    nextPID += PID_INTERVAL;
  }
}

void runCommand(char cmd, int val[])
{
  if (cmd == 'e')
  {
    Serial.print(left_pos);
    Serial.print(" ");
    Serial.print(right_pos);
    Serial.print(" ");
    Serial.print(back_pos);
    Serial.println(" ");
  }
  else if (cmd == 'r')
  {
    left_pos = 0;
    right_pos = 0;
    back_pos = 0;
    Serial.println("OK");
  }
  else if(cmd == 'p')
  {
    Kp = val[0];
    Kd = val[1];
    Ki = val[2];
    Ko = val[3];
//    Serial.println("OK");
  }
  else if(cmd == 'm')
  {
    if (val[0] == 0 && val[1] == 0 && val[2] == 0)
    {
//      Serial.println("Motor Stops");
      stopMotors();
      resetPID();
      moving = 0;
    }
    else
      moving = 1;
    leftPID.TargetTicksPerFrame = val[0];
    rightPID.TargetTicksPerFrame = val[1];
    backPID.TargetTicksPerFrame = val[2];
    Serial.println("OK");
  }
}

void motorControl(int spd,int rightward_pin, int leftward_pin, int enable_pin)
{
  unsigned char reverse = 0;
  if(spd < 0)
  {
    spd = -spd;
    reverse = 1;
  }
  if(spd > MAX_PWM)
  {
    spd = MAX_PWM;
  }
  analogWrite(enable_pin, spd);
  if (reverse == 0)
  {
    digitalWrite(rightward_pin,HIGH);
    digitalWrite(leftward_pin,LOW);
  }
  else if (reverse ==1)
  {
    digitalWrite(rightward_pin,LOW);
    digitalWrite(leftward_pin,HIGH);
  }
}

void resetPID()
{
  leftPID.TargetTicksPerFrame = 0.0;
  leftPID.Encoder = left_pos;
  leftPID.PrevEnc = leftPID.Encoder;
  leftPID.output = 0;
  leftPID.PrevInput = 0;
  leftPID.ITerm = 0;

  rightPID.TargetTicksPerFrame = 0.0;
  rightPID.Encoder = right_pos;
  rightPID.PrevEnc = rightPID.Encoder;
  rightPID.output = 0;
  rightPID.PrevInput = 0;
  rightPID.ITerm = 0;
  
  backPID.TargetTicksPerFrame = 0.0;
  backPID.Encoder = back_pos;
  backPID.PrevEnc = backPID.Encoder;
  backPID.output = 0;
  backPID.PrevInput = 0;
  backPID.ITerm = 0;
}

void doPID(SetPointInfo * p) 
{
  long Perror;
  long output;
  int input;
  input = p->Encoder - p->PrevEnc;
  Perror = p->TargetTicksPerFrame - input;
  output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;
  p->PrevEnc = p->Encoder;

  output += p->output;
  if (output >= MAX_PWM)
    output = MAX_PWM;
  else if (output <= -MAX_PWM)
    output = -MAX_PWM;
  else
    p->ITerm += Ki * Perror;

  p->output = output;
  p->PrevInput = input;
}

void updatePID()
{
  leftPID.Encoder = left_pos;
  rightPID.Encoder = right_pos;
  backPID.Encoder = back_pos;
  
  if (!moving)
  {
    if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0 || backPID.PrevInput != 0)
      resetPID();
    return;
  }
  
  doPID(&leftPID);
  doPID(&rightPID);
  doPID(&backPID);
  motorControl(leftPID.output, LEFT_WHEEL_RIGHTWARD, LEFT_WHEEL_LEFTWARD, LEFT_WHEEL_ENABLE);
  motorControl(rightPID.output, RIGHT_WHEEL_RIGHTWARD, RIGHT_WHEEL_LEFTWARD, RIGHT_WHEEL_ENABLE);
  motorControl(backPID.output, BACK_WHEEL_RIGHTWARD, BACK_WHEEL_LEFTWARD, BACK_WHEEL_ENABLE);  
}

void stopMotors()
{
  motorControl(0, LEFT_WHEEL_RIGHTWARD, LEFT_WHEEL_LEFTWARD, LEFT_WHEEL_ENABLE);
  motorControl(0, RIGHT_WHEEL_RIGHTWARD, RIGHT_WHEEL_LEFTWARD, RIGHT_WHEEL_ENABLE);
  motorControl(0, BACK_WHEEL_RIGHTWARD, BACK_WHEEL_LEFTWARD, BACK_WHEEL_ENABLE);
}
