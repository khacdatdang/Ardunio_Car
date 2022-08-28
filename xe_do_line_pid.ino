////////// XE DO LINE - IR - PID //////////

#define ss0 A0
#define ss1 A1
#define ss2 A2
#define ss3 A3
#define ss4 A4

#define inA1 2
#define inA2 4
#define inB1 5
#define inB2 7

#define EnA 3
#define EnB 6

#define clp A5  //Cong tac hanh trinh

float P;
float error;
float I, D;
float previous_error = 0;
float PID_value;

int sensor[5];
int initial_motor_speed = 70;
boolean start = false;

long long T1 = 0;
int SoNgach = 0;

void setup()
{
  // put your setup code here, to run once:
  pinMode(ss0, INPUT);
  pinMode(ss1, INPUT);
  pinMode(ss2, INPUT);
  pinMode(ss3, INPUT);
  pinMode(ss4, INPUT);
  pinMode(clp, INPUT);

  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);

  Serial.begin(9600);

  I = 0;  D = 0;  P = 0;  error = 0;
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (digitalRead(clp) == 1)
    start = true;

    read_sensor_values();
    calculate_pid();
  if (start == true)
  {
    motor_control();
  }
}

void read_sensor_values()
{
//  sensor[0] = digitalRead(ss0);
//  sensor[1] = digitalRead(ss1);
//  sensor[2] = digitalRead(ss2);
//  sensor[3] = digitalRead(ss3);
//  sensor[4] = digitalRead(ss4);

  if (digitalRead(ss0) == 0)    sensor[0] = 1;  else    sensor[0] = 0;
  if (digitalRead(ss1) == 0)    sensor[1] = 1;  else    sensor[1] = 0;
  if (digitalRead(ss2) == 0)    sensor[2] = 1;  else    sensor[2] = 0;
  if (digitalRead(ss3) == 0)    sensor[3] = 1;  else    sensor[3] = 0;
  if (digitalRead(ss4) == 0)    sensor[4] = 1;  else    sensor[4] = 0;

//  for(int i=0;i<5;i++)
//  Serial.print(sensor[i]);
//  Serial.println(" ");
// debug code

  // khi ở trên đường
  if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))    error = 0; //0  

  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))    error = 4; //4
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0))    error = 2; //2
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))    error = 1; //1


  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))    error = -1; //-1
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))    error = -2; //-2
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))    error = -4; //-4

  
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))    error = 6; //6
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))    error = -6; //-6

  // khi cham 1 bong den cuoi
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))    error = 0; 
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    error = 0; 
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))    error = 0; //moi them: error=0:

  
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))    error = 4; //4
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))    error = 2; //2
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))    error = 1; //1
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))    error = -1; //-1
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))    error = -2; //-2
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))    error = -4; //-4
  
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 1))    error = 6; //6
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))    error = -6; //-6
  else error = 0 ;
  
}

void calculate_pid()
{
  float Kp = 10.5, Kd = 0.25, Ki = 0;

  float P, I, D;
  float SamplingTime = 0.01;
  error = 0 - error;
  P = error * Kp;
  I += Ki * error * SamplingTime;
  D = (Kd * (error - previous_error)) / SamplingTime;
  PID_value = P + I + D;
  previous_error = error;
//  Serial.print("error = " + String(error)); // debug
//  Serial.print("  ,PID = " + String(PID_value));
//  Serial.println("  ,SoNgach = " + String(SoNgach));
  
}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed  = initial_motor_speed + PID_value;   
  int right_motor_speed = initial_motor_speed - PID_value;   
//  Serial.print("PID= ");  Serial.println(PID_value);
  Serial.print(left_motor_speed);
  Serial.print(" ");
  Serial.println(right_motor_speed);
  
  // The motor speed should not exceed the max PWM value
  //// Mới thêm //////////////////////////
  if (left_motor_speed > 255)
    left_motor_speed = 255;
  else if (left_motor_speed < 0)
    left_motor_speed = 0;

  if (right_motor_speed > 255)
    right_motor_speed = 255;
  else if (right_motor_speed < 0 )
    right_motor_speed = 0;
  //// Kết thúc mới thêm//////////////////

  analogWrite(EnB, left_motor_speed); //Left Motor Speed
  analogWrite(EnA, right_motor_speed); //Right Motor Speed

  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);
}
