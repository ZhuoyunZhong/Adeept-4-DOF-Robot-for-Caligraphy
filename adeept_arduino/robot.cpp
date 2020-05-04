#include <ros.h>
#include <std_msgs/String.h>
#include <Servo.h>

// Define servo
int servopin1 = 9;    //Define servo interface digital interface 9
int servopin2 = 6;    //Define servo interface digital interface 6
int servopin3 = 5;    //Define servo interface digital interface 5
int servopin4 = 3;    //Define servo interface digital interface 3
int servopin5 = 11;   //Define servo interface digital interface 11

int moveServoData;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

// Command callback
ros::NodeHandle nh;

void commandCb1(const std_msgs::Float64& msg){
    joint_pos = msg.data /3.1416*180 + 90;
    servo1.attach(servopin1);
    servo1.write(joint_pos);
}

void commandCb2(const std_msgs::Float64& msg){
    joint_pos = msg.data /3.1416*180 + 155;
    servo2.attach(servopin2);
    servo2.write(joint_pos);
}

void commandCb3(const std_msgs::Float64& msg){
    joint_pos = msg.data /3.1416*180 + 155;
    servo3.attach(servopin3);
    servo3.write(joint_pos);
}

void commandCb4(const std_msgs::Float64& msg){
    joint_pos = msg.data /3.1416*180 + 85;
    servo4.attach(servopin4);
    servo4.write(joint_pos);
}

// Command subscriber initialization
ros::Publisher joints_pub = nh.advertise<std_msgs::String>("chatter", 1);
ros::Subscriber<std_msgs::Float64> command_sub1('/adeept/joint1_position_controller/command', &commandCb1);
ros::Subscriber<std_msgs::Float64> command_sub2('/adeept/joint2_position_controller/command', &commandCb2);
ros::Subscriber<std_msgs::Float64> command_sub3('/adeept/joint3_position_controller/command', &commandCb3);
ros::Subscriber<std_msgs::Float64> command_sub4('/adeept/joint4_position_controller/command', &commandCb4);


void setup(){
    //Set the servo interface as the output interface
    pinMode(servopin1, OUTPUT); 
    pinMode(servopin2, OUTPUT);
    pinMode(servopin3, OUTPUT);
    pinMode(servopin4, OUTPUT);
    pinMode(servopin5, OUTPUT);

    // Receive command
    nh.initNode();
    nh.subscribe(command_sub1)
    nh.subscribe(command_sub2)
    nh.subscribe(command_sub3)
    nh.subscribe(command_sub4)

    Serial.begin(9600);
}


void loop(){
    nh.spinOnce();
    delay(20);
}
