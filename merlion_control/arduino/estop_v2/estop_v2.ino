#include <ros.h>
#include <std_msgs/Bool.h>

ros::NodeHandle nh;
std_msgs::Bool bool_msg;
ros::Publisher switch_pub("/merlion/disarm", &bool_msg);

void setup() {
  pinMode(8, INPUT_PULLUP);
  Serial.begin(9600);

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
  int state = digitalRead(8); // GND connect = false, opened = true

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
  switch_pub.publish(&bool_msg);
  nh.spinOnce();
  delay(100);
}
