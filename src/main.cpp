#include "include.h"

#define LOOPTIME 100

ros::NodeHandle nh;
// rospy_tutorials::Floats debug_msg;
// std_msgs::Header debug_header;
// rovit_navsys::debug debug_message;

// Debug variables
char noVelDataCount = 0;
short cmdVelNoDataMillis;
short cmdVelLastMillis;
short cmdVelTimeout = 0;
bool timeout;

// PID Setting Variables
const float PID_Left_Param[3] = {0.3055939155039545, 9.86003191826279, 0.0};
// opsi lain gain 0.22459699617741438, 7.246654591938676 adjust here https://pidtuner.com/#/DxuRkeRLrY
const float PID_Right_Param[3] = {0.5240459349708211, 9.21523683674757, 0.0};

double leftPIDOut, rightPIDOut, leftSpeed, rightSpeed, leftSetpoint, rightSetpoint;
float speed_array[3];
// float tempLSpeed, tempRSpeed;

PID PIDLeftMotor(&leftSpeed, &leftPIDOut, &leftSetpoint, PID_Left_Param[0], PID_Left_Param[1], PID_Left_Param[2], DIRECT);
PID PIDRightMotor(&rightSpeed, &rightPIDOut, &rightSetpoint, PID_Right_Param[0], PID_Right_Param[1], PID_Right_Param[2], DIRECT);

// void calibCb(const std_msgs::Empty& cal_mode);
void cmdVelCb(const geometry_msgs::Twist &cmd_vel);
// void imu_calibration_routine();
void measureSpeed();
// float SpeedtoPWM(float speed, int motor);
float SpeedtoPWM(float speed, int motor, float correction);

ros::Subscriber<geometry_msgs::Twist> cmdVel("cmd_vel", cmdVelCb);
// ros::Subscriber <std_msgs::Empty> calib_f("imu/calibration", calibCb);

// ros::Publisher imu_pub("imu/data_raw", &imu_msg);
// ros::Publisher debug_pub("debug/data", &debug_message);

geometry_msgs::Vector3Stamped speed_msg;       // create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg); // create a publisher to ROS topic "speed" using the "speed_msg" type

int countLeft = 0;
int countRight = 0;

void left_wheel_tick()
{
  if (digitalRead(MOTOR_PINS[1]) == 1)
  {
    countLeft++;
  }
  else
  {
    countLeft--;
  }
}

void right_wheel_tick()
{
  if (digitalRead(MOTOR_PINS[4]) == 0)
  {
    countRight++;
  }
  else
  {
    countRight--;
  }
}

int getLeftTick()
{
  return countLeft;
}

int getRightTick()
{
  return countRight;
}

int last_millis[2]; // Number of tasks

void motor_setup()
{
  for (int i = 0; i <= 5; i++)
  {
    pinMode(MOTOR_PINS[i], OUTPUT);
  }

  pinMode(ENCODER_PINS[0], INPUT_PULLUP); // Left encoder
  pinMode(ENCODER_PINS[1], INPUT_PULLUP); // Right encoder

  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[0]), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[1]), right_wheel_tick, RISING);

  PIDLeftMotor.SetMode(AUTOMATIC);
  PIDRightMotor.SetMode(AUTOMATIC);
  PIDLeftMotor.SetSampleTime(95);
  PIDRightMotor.SetSampleTime(95);
  PIDLeftMotor.SetOutputLimits(-0.7, 0.7);
  PIDRightMotor.SetOutputLimits(-0.7, 0.7);
}

float calculateSpeed(long count, long time)
{
  float speed = (((count / encoder_cpr) / wheel_gear_ratio) * PI * wheel_dia) * (1000 / time);
  return speed;
}

void safetyCmdVelWD()
{
  cmdVelNoDataMillis++;
  if (cmdVelNoDataMillis > 100)
  {
    cmdVelTimeout += millis() - cmdVelLastMillis;
    cmdVelLastMillis = millis();
  }
  if (cmdVelTimeout > 100)
  {
    timeout = true;
  }
  else
  {
    timeout = false;
  }
}

void cmdVelCb(const geometry_msgs::Twist &cmd_vel)
{
  leftSetpoint = cmd_vel.linear.x - cmd_vel.angular.z * (wheelbase / 2);
  rightSetpoint = cmd_vel.linear.x + cmd_vel.angular.z * (wheelbase / 2);
  cmdVelNoDataMillis = 0;
}

void measureSpeed()
{
  leftSpeed = calculateSpeed(getLeftTick(), 10);
  rightSpeed = calculateSpeed(getRightTick(), 10);
}

void setMotorPWM(short PWM, bool left)
{
  if (PWM > 0) // forward
  {
    if (left)
    {
      digitalWrite(MOTOR_PINS[0], HIGH); // EL_left
      digitalWrite(MOTOR_PINS[1], HIGH); // ZF_left
    }
    else
    {
      digitalWrite(MOTOR_PINS[3], HIGH); // EL_right
      digitalWrite(MOTOR_PINS[4], LOW);  // ZF_right
    }
  }
  else // backward
  {
    if (left)
    {
      digitalWrite(MOTOR_PINS[0], HIGH); // EL_left
      digitalWrite(MOTOR_PINS[1], LOW);  // ZF_left
    }
    else
    {
      digitalWrite(MOTOR_PINS[3], HIGH); // EL_right
      digitalWrite(MOTOR_PINS[4], HIGH); // ZF_right
    }
  }

  if (PWM == 0)
  {
    if (left)
    {
      digitalWrite(MOTOR_PINS[0], LOW); // EL_left
    }
    else
    {
      digitalWrite(MOTOR_PINS[3], LOW); // EL_right
    }
  }
}

void motor_run()
{
  /* Compute speed
   * Compute PID
   * Compute Speed to PWM
   * Execute PWM
   */
  float Left_PWM;
  float Right_PWM;

  PIDLeftMotor.Compute();
  PIDRightMotor.Compute();

  Left_PWM = SpeedtoPWM(leftSetpoint, 0, leftPIDOut);
  Right_PWM = SpeedtoPWM(rightSetpoint, 1, rightPIDOut);

  setMotorPWM(Left_PWM, 1);
  setMotorPWM(Right_PWM, 0);

  if (leftSetpoint == 0)
  {
    setMotorPWM(0, 1);
  }
  else
  {
    setMotorPWM(Left_PWM, 1);
  }

  if (rightSetpoint == 0)
  {
    setMotorPWM(0, 0);
  }
  else
  {
    setMotorPWM(Right_PWM, 0);
  }
}

float SpeedtoPWM(float speed, int motor, float correction)
{
  int PWM;
  if (speed > 0)
  {
    PWM = (speed + correction + min_speed[motor]) / speed_to_pwm[motor];
  }
  else
  {
    PWM = (speed + correction - min_speed[motor + 2]) / speed_to_pwm[motor + 2];
  }
  return PWM;
}

void publishSpeed(double time)
{
  measureSpeed();
  speed_msg.header.stamp = nh.now(); // timestamp for odometry data
  speed_msg.vector.x = leftSpeed;    // left wheel speed (in m/s)
  speed_msg.vector.y = rightSpeed;   // right wheel speed (in m/s)
  speed_msg.vector.z = time / 1000;  // looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
}

void ros_init()
{
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  // nh.advertise(imu_pub);
  // nh.advertise(debug_pub);
  nh.advertise(speed_pub);
  nh.subscribe(cmdVel);
  // nh.subscribe(calib_f);
}

// void task1(){
//   if(millis() - last_millis[0] > 10){
//     last_millis[0] = millis();
//     imu_update();
//   }
// }

void task2()
{
  if (millis() - last_millis[0] > 100)
  {
    last_millis[0] = millis();
    motor_run();
  }
}

void task3()
{
  if (millis() - last_millis[1] >= LOOPTIME)
  {
    last_millis[1] = millis();
    // update_debug();
    publishSpeed(LOOPTIME);
  }
}

void setup()
{
  ros_init();
  // imu_setup();
  motor_setup();
  WiFi.mode(WIFI_OFF);
  btStop();
}

void loop()
{
  // task1();
  task2();
  task3();
}