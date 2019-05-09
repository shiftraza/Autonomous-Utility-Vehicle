#include <PID_v1.h>
#include <Servo.h>

volatile int dist_interrupt;

double Setpoint;
double Input;
double Output;

double Kp = 1, Ki = 4, Kd = 1;
double New_Speed;
//parameters for PID

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//pid instance

int pos = 0;
const int Input1 = 7;
const int Input2 = 8;
const int ENA = 6;
const int Input3 = 12;
const int Input4 = 13;
const int ENB = 5;
const int In1 = 9;
const int In2 = 2;
const int In3 = 4;
const int In4 = A1;
const int ENA2 = 11;
const int ENB2 = 3;

const int echoPin = A3;
const int trigPin = A2;

const int echo2 = A5;
const int trig2 = A4;

long duration;
long distance;
long dist;

long duration2;
long distance2;
long dist2;

int SPEED = 100;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  //PID Stuff
  Setpoint = 20;
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd);
  //PID Stuff

  pinMode(Input1, OUTPUT);
  pinMode(Input2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(Input3, OUTPUT);
  pinMode(Input4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(trig2, OUTPUT);

  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(ENA2, OUTPUT);
  pinMode(In3, OUTPUT);
  pinMode(In4, OUTPUT);
  pinMode(ENB2, OUTPUT);
}

void loop()
{

  dist = detect_wall(echoPin, trigPin);
  dist2 = detect_wall(echo2, trig2);
  //  Serial.println("dist1: " + String(dist));
  if (dist < 50)
  {
    Dist_Interrupt();        //Look around for an alternative route
    delayMicroseconds(2000); // delay to prevent measurement issues
    turn(SPEED);
  }
  else if (dist >= 100 && dist2 == 30)
  {
    forward();
  }
  else
  {
    New_Speed = PID_controller(dist2);
    turn(New_Speed);
  }
}

bool forward()
{
  digitalWrite(Input1, HIGH);
  digitalWrite(Input2, LOW);
  digitalWrite(Input3, HIGH);
  digitalWrite(Input4, LOW);
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  analogWrite(ENA2, SPEED);
  analogWrite(ENB2, SPEED);
}

bool back()
{
  digitalWrite(Input1, LOW);
  digitalWrite(Input2, HIGH);
  digitalWrite(Input3, LOW);
  digitalWrite(Input4, HIGH);
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);

  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  digitalWrite(In3, LOW);
  digitalWrite(In4, HIGH);
  analogWrite(ENA2, SPEED);
  analogWrite(ENB2, SPEED);
}

bool turn(double New_Speed)
{
  digitalWrite(Input1, HIGH);
  digitalWrite(Input2, LOW);
  digitalWrite(Input3, HIGH);
  digitalWrite(Input4, LOW);
  analogWrite(ENA, New_Speed);
  analogWrite(ENB, New_Speed);

  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  digitalWrite(In3, HIGH);
  digitalWrite(In4, LOW);
  analogWrite(ENA2, SPEED);
  analogWrite(ENB2, SPEED);
}

int detect_wall(int echo, int trigger)
{

  digitalWrite(trigger, LOW);
  delayMicroseconds(2);

  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);

  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, HIGH);

  distance = duration / 58.2;

  return distance;
}

double PID_controller(int dist2)
{
  if (dist2 > 300)
  {
    dist2 = 300;
  }

  Input = map(dist2, 0, 300, 0, 255);
  myPID.Compute();
  delayMicroseconds(10);
  return Output;
}

void Dist_Interrupt()
{
  int temp1 = 0;
  int temp2;
  for (int i = 0; i <= 180; i++)
  {
    myServo.write(i);
    delayMicroseconds(30);
    temp2 = detect_wall;

    if (temp2 > temp1)
    {
      temp1 = temp2; // this will store the highest value of distance as the sensor rotates
    }
    if (i == 180)
    {
      return i;
    }
  }
}

