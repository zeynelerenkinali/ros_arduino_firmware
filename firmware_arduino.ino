#include <SharpIR.h>
#include <Encoder.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Twist.h>
#include <rosserial_arduino/Test.h>
#include <std_srvs/Empty.h>

// Define motor pins
#define M1R_DIR 5   // RIGHT motor direction
#define M1R_PWM 4   // RIGHT motor PWM
#define M2L_DIR 6   // LEFT motor direction
#define M2L_PWM 9   // LEFT motor PWM

// Define range sensor pins (IR sensors)
#define IR_FRONT A2 // Front IR sensor
#define IR_RIGHT A3 // Right IR sensor
#define IR_LEFT  A4 // Left IR sensor

// Define encoder pins
#define ENC_M1R_A 18  // RIGHT encoder channel A 
#define ENC_M1R_B 19  // RIGHT encoder channel B
#define ENC_M2L_A 3   // LEFT encoder channel A
#define ENC_M2L_B 2   // LEFT encoder channel B

#define axis_length 0.097
#define radius 0.016
#define pulses_rev 8400.0
#define rate 0.1
#define maxSpeed 0.083

#define KP 0.001
#define KI 0.02
#define KD 0

#define FORWARD 0
#define BACKWARD 1

// SMART LED Pin
#define SMART_LED_P 10 
#define NUMPIXELS 2 // Number of leds attached to the arduino

// Define the SharpIR model
#define MODEL 1

#define NORMALIZE(z) atan2(sin(z), cos(z))

typedef struct odomMsg{
  float x, y, v, w, theta;
};
typedef struct speed{
  float v, w, wLeft, wRight;
};
typedef struct errorValues{
 float eP[2], eD[2], eI[2];
};

// Global variables
odomMsg currentPose = {0.0, 0.0, 0.0, 0.0, 0.0};
speed desiredRobotSpeed = {0.00, 0.0, 0.0, 0.0};
speed realRobotSpeed = {0.0, 0.0, 0.0, 0.0};
errorValues errValues = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
errorValues prevErrValues = {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
float G[2] = {0.0, 0.0};
float deltaT = 0.1;
float linear_x = 0.0;
float angular_z = 0.0;
bool led_cond = true;

// Setup SharpIR sensors
SharpIR leftSensor(MODEL, IR_LEFT);
SharpIR frontSensor(MODEL, IR_FRONT);
SharpIR rightSensor(MODEL, IR_RIGHT);

// Setup encoders
Encoder encL(ENC_M1R_A, ENC_M1R_B);
Encoder encR(ENC_M2L_A, ENC_M2L_B);

// Setup RGB
Adafruit_NeoPixel strip(NUMPIXELS, SMART_LED_P, NEO_GRB + NEO_KHZ800);
uint8_t ledRGB[3] = {0,0,0};

// Create an instance for node handler
ros::NodeHandle nh;

std_msgs::Float32 front_distance_msg;
std_msgs::Float32 right_distance_msg;
std_msgs::Float32 left_distance_msg;
geometry_msgs::Pose2D pose_msg;

ros::Publisher front_distance_pub("front_distance", &front_distance_msg);
ros::Publisher right_distance_pub("right_distance", &right_distance_msg);
ros::Publisher left_distance_pub("left_distance", &left_distance_msg);
ros::Publisher pose_pub("pose", &pose_msg);

void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel)
{
  nh.loginfo("cmd_vel received!");
  linear_x = cmd_vel.linear.x;
  angular_z = cmd_vel.angular.z;
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_callback);

void set_pose_callback(const geometry_msgs::Pose2D& set_pose)
{
  currentPose.x = set_pose.x;
  currentPose.w = set_pose.theta;
}
ros::Subscriber<geometry_msgs::Pose2D> set_pose_sub("set_pose", set_pose_callback);


void ledRGB_callback(const std_msgs::ColorRGBA& ledRGBmsg)
{
  nh.loginfo("RGB color received!");
  ledRGB[0] = ledRGBmsg.r;
  ledRGB[1] = ledRGBmsg.g;
  ledRGB[2] = ledRGBmsg.b;
  set_leds_color(ledRGB[0], ledRGB[1], ledRGB[2], 0);
}
ros::Subscriber<std_msgs::ColorRGBA> color_sub("ledRGB", ledRGB_callback);

// Initialize the buzzer service in order to start and stop buzzer
// We are used char approach because request does not accept using with String
bool equalsIgnoreCase(const char* a, const char* b) 
{
  while (*a && *b) 
  {
    if (tolower(*a) != tolower(*b)) 
    {
      return false;
    }
    ++a;
    ++b;
  }
  return *a == *b;
}

bool buzzerServiceCallback(rosserial_arduino::Test::Request &req, rosserial_arduino::Test::Response &res) 
{
  if (equalsIgnoreCase(req.input, "start")) 
  {
    res.output = "Buzzer has started";
  } 
  else if (equalsIgnoreCase(req.input, "stop")) 
  {
    res.output = "Buzzer has stopped";
  } 
  else 
  {
    res.output = "Error on request";
    return false;
  }
  return true;
}
ros::ServiceServer<rosserial_arduino::Test::Request, rosserial_arduino::Test::Response> buzzer_srv("buzzer_service", &buzzerServiceCallback);


bool setRGBCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) 
{
  nh.loginfo("RGB color service request received!");

  // Update the LED colors with the received values
  if(led_cond == true)
  {
    ledRGB[0] = 0;
    ledRGB[1] = 0;
    ledRGB[2] = 0;
    led_cond = false;
  }
  else
  {
    ledRGB[0] = 255;
    ledRGB[1] = 255;
    ledRGB[2] = 255;
    led_cond = true;
  }
  // Set the LEDs to the new color
  set_leds_color(ledRGB[0], ledRGB[1], ledRGB[2], 0);

  return true;
}
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> set_rgb_srv("set_rgb_service", &setRGBCallback);


void setup() 
{
  // Define pin modes for sensors
  pinMode(IR_FRONT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_LEFT, INPUT);

  // Define pin modes for motors
  pinMode(M1R_DIR, OUTPUT);
  pinMode(M1R_PWM, OUTPUT);
  pinMode(M2L_DIR, OUTPUT);
  pinMode(M2L_PWM, OUTPUT);

  // Define pin modes for encoders
  pinMode(ENC_M1R_A, INPUT);
  pinMode(ENC_M1R_B, INPUT);
  pinMode(ENC_M2L_A, INPUT);
  pinMode(ENC_M2L_B, INPUT);
  
  // Define Smart RGB
  strip.begin();           // Initialize NeoPixel object
  strip.setBrightness(10); // Set BRIGHTNESS to about 4% (max = 255)
  strip.show();            // Initialize all pixels to 'off'
  
  // Initialize the node
  nh.getHardware()->setBaud(57600);  
  nh.initNode();
 
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(set_pose_sub);
  nh.subscribe(color_sub);

  nh.advertise(front_distance_pub);
  nh.advertise(right_distance_pub);
  nh.advertise(left_distance_pub);
  nh.advertise(pose_pub);
  
  nh.advertiseService(buzzer_srv);
  nh.advertiseService(set_rgb_srv);
}

void loop() 
{
  // Getting sensor values with library
  unsigned long curMillis = millis();
  unsigned long inMillis = curMillis;
  while(1)
  {
    if(millis() - curMillis >= rate*1000) // means if one second pass kind of delay one seconds
    {    
      curMillis = millis();
      
      // Give the robot desired linear and angular velocities
      // float* desiredSpeed = cmdVel(linear_x, angular_z);
      desiredRobotSpeed.v = linear_x;
      desiredRobotSpeed.w = angular_z;
      
      // Calculate the desired wL and wR for the robot wheelsö
      float* vel2Wheels = cmdVel2Wheels(desiredRobotSpeed.v, desiredRobotSpeed.w); 
      desiredRobotSpeed.wLeft = vel2Wheels[0];
      desiredRobotSpeed.wRight = vel2Wheels[1];
      
      float* encVal = encUpdate();
      // Calculate Pose 2 
      currentPose = poseUpdate(encVal[0], encVal[1]);
      
      // Calculate real angular velocities of the wheels wL, wR
      vel2Wheels = cmdVel2Wheels(currentPose.v, currentPose.w); // calculate the desired wL and wR for the robot wheels
      realRobotSpeed.wLeft = vel2Wheels[0];
      realRobotSpeed.wRight = vel2Wheels[1];
      
      // Run the PID Control function
      pidControl(deltaT);

      front_distance_msg.data = sensF();
      right_distance_msg.data = sensR();
      left_distance_msg.data = sensL();

      // 
      pose_msg.x = currentPose.x;
      pose_msg.y = currentPose.y;
      pose_msg.theta = currentPose.theta;
      
      front_distance_pub.publish(&front_distance_msg);
      right_distance_pub.publish(&right_distance_msg);
      left_distance_pub.publish(&left_distance_msg);
      pose_pub.publish(&pose_msg);
    }
    nh.spinOnce();
  }  
}

struct odomMsg poseUpdate(float encLeft, float encRight)
{
  static odomMsg odomMsgIns;

  float displacement[2] = {((2.0 * M_PI * radius) / pulses_rev) * encLeft,((2.0 * M_PI * radius) / pulses_rev) * encRight};
  float linearDisplacement = (displacement[0] + displacement[1]) / 2.0; // D
  odomMsgIns.v = ((2.0 * PI * radius) / pulses_rev) * ((encRight + encLeft) / 2.0) * (1.0 / deltaT);
  odomMsgIns.w = ((2.0 * PI * radius) / pulses_rev) * ((encRight - encLeft) / axis_length) * (1.0 / deltaT);
  odomMsgIns.theta = atan2(sin(currentPose.theta + odomMsgIns.w * deltaT), cos(currentPose.theta + odomMsgIns.w * deltaT));
  odomMsgIns.x = currentPose.x + odomMsgIns.v * cos(odomMsgIns.theta) * deltaT;
  odomMsgIns.y = currentPose.y + odomMsgIns.v * sin(odomMsgIns.theta) * deltaT;
  
  return odomMsgIns;
}

float* encUpdate()
{
  static float encDiff[2], encPrevVal[2] = {encL.read(), encR.read()};
  
  encDiff[0] = encL.read() - encPrevVal[0];
  encDiff[1] = encR.read() - encPrevVal[1];
  
  encPrevVal[0] = encL.read();
  encPrevVal[1] = encR.read();
  
  return encDiff;
}

float* cmdVel(float linear_x, float angular_z)
{
  static float vel[2] = {linear_x, angular_z}; // velocity
  return vel;
}

float* cmdVel2Wheels(float v, float w) 
{
  static float vel2Wheels[2];
  vel2Wheels[0] = ((2.0 * v) - (axis_length * w)) / (2.0 * radius); // left angular value
  vel2Wheels[1] = ((2.0 * v) + (axis_length * w)) / (2.0 * radius); // right angular value
  
  return vel2Wheels;
}

void pidControl(float time)
{
  errValues.eP[0] = desiredRobotSpeed.wLeft - realRobotSpeed.wLeft; // Left Wheel 
  errValues.eP[1] = desiredRobotSpeed.wRight - realRobotSpeed.wRight; // Right Wheel
  errValues.eI[0] = errValues.eI[0] + errValues.eP[0] * time; // Left Wheel
  errValues.eI[1] = errValues.eI[1] + errValues.eP[1] * time; // Right Wheel
  errValues.eD[0] = (errValues.eP[0] - prevErrValues.eP[0]) / time; // Left Wheel
  errValues.eD[1] = (errValues.eP[1] - prevErrValues.eP[1]) / time; // Right Wheel
  
  // PID controller
  G[0] = KP * errValues.eP[0] + KI * errValues.eI[0] + KD * errValues.eD[0]; // Left Wheel
  G[1] = KP * errValues.eP[1] + KI * errValues.eI[1] + KD * errValues.eD[1]; // Right Wheel
  
  // Save previous values for the next iteration
  prevErrValues = errValues;
  
  applySpeed(G[0], G[1], FORWARD, FORWARD);
}

void applySpeed(float speedLeft, float speedRight, int rightDirection, int leftDirection) 
{
  // constrain function will stead the wheels speed between 255 and 0
  int pwmLeft = constrain(255 * speedLeft / maxSpeed, 0, 255);
  int pwmRight = constrain(255 * speedRight / maxSpeed, 0, 255);

  analogWrite(M2L_PWM, pwmLeft);
  digitalWrite(M2L_DIR, leftDirection);
  analogWrite(M1R_PWM, pwmRight);
  digitalWrite(M1R_DIR, rightDirection);
}

int sensL() 
{ 
  int sum = 0, repeatAverage = 3; // sum will be the summation of three values and repeatAverage is the value that we will be filtering.
  for(int i = 0; i < repeatAverage; i++) // Will add three sensor values instantaneously and give an average of three of them sort of filtering the values for reaching exact value
  {
    sum += leftSensor.getDistance() * 10; // Left Sensor value obtained by library function multiplied by 10 to get mm type values
  }
  return sum / repeatAverage;  
}
int sensR() 
{
  int sum = 0, repeatAverage = 3;  // sum will be the summation of three values and repeatAverage is the value that we will be filtering.
  for(int i = 0; i < repeatAverage; i++) // Will add three sensor values instantaneously and give an average of three of them sort of filtering the values for reaching exact value
  {
    sum += rightSensor.getDistance() * 10; // Right Sensor value obtained by library function multiplied by 10 to get mm type values
  }
  return sum / repeatAverage;  // Right Sensor value obtained by library function multiplied by 10 to get mm type values
}
int sensF() 
{
  int sum = 0, repeatAverage = 3; // sum will be the summation of three values and repeatAverage is the value that we will be filtering.
  for(int i = 0; i < repeatAverage; i++) // Will add three sensor values instantaneously and give an average of three of them sort of filtering the values for reaching exact value
  {
    sum += frontSensor.getDistance() * 10; // Front Sensor value obtained by library function multiplied by 10 to get mm type values
  }
  return sum / repeatAverage;  // Front Sensor value obtained by library function multiplied by 10 to get mm type values
}

void set_leds_color(uint8_t R, uint8_t G, uint8_t B, unsigned int t_delay)
{
  for (int i = 0; i<NUMPIXELS; i++){
    strip.setPixelColor(i, strip.Color(R,G,B));
    strip.show();
    delay(t_delay);
  }
}
