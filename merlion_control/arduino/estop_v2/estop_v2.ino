#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;
std_msgs::Bool bool_msg;
ros::Publisher switch_pub("/merlion/disarm", &bool_msg);

unsigned long last_pub_time = 0;
unsigned long last_read_time = 0;
bool state = false;

void setup() {
  pinMode(8, INPUT_PULLUP);
  pinMode(13, OUTPUT);
//  Serial.begin(57600);

  nh.initNode();
  nh.advertise(switch_pub);

  bool_msg.data = false;
  
  nh.loginfo("E-STOP: Waiting for connection...");
  while(!nh.connected()) 
    nh.spinOnce();
  nh.loginfo("E-STOP: Startup complete");
}

void loop() {
  // Read digital signal
  if (millis() - last_read_time > 20){
    state = digitalRead(8); // GND connect = false, opened = true
    digitalWrite(13, state);
    last_read_time = millis();
  }
  // If estop is closed (GND connected) -> send disarm, else arm
  if(!state)
  {
    bool_msg.data = true;
    // nh.loginfo("Disarming");
  }
  else
  {
    bool_msg.data = false;
    // nh.loginfo("Arming");
  }
  if (millis() - last_pub_time > 50){
    switch_pub.publish(&bool_msg);
    last_pub_time = millis();
    nh.spinOnce();
  }
//  delay(50);
}
