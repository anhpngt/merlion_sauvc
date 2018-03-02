#include <ros.h>
#include <sensor_msgs/Joy.h>

ros::NodeHandle nh;
sensor_msgs::Joy joy_msg;
ros::Publisher joy_pub("/joy", &joy_msg);

float axes[] = {0., 0., 0., 0., 0., 0.};
int32_t buttons_arm[]    = {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
int32_t buttons_disarm[] = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0};
bool isarmed = false;
 
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(115200);

  nh.initNode();
  nh.getHardware()->setBaud(115200);
  nh.advertise(joy_pub);

  joy_msg.header.seq = 1;
  joy_msg.header.frame_id = "";
  joy_msg.axes_length = 6;
  joy_msg.axes = axes;
  joy_msg.buttons_length = 15;
  joy_msg.buttons = buttons_disarm;
  isarmed = false;
  
  nh.loginfo("E-STOP: Waiting for connection...");
  while(!nh.connected()) 
    nh.spinOnce();
  nh.loginfo("E-STOP: Startup complete");
}

void loop() {
  // Update msg header
  joy_msg.header.stamp = nh.now();
  
  // Read voltage from resistor
  float voltage = analogRead(A0) * (5.0 / 1023.0);

  // If estop is closed -> send stop cmd
  Serial.println(voltage);
  if(voltage > 4.5)
  {
    if(isarmed)
    {
      nh.loginfo("Disarming");
      digitalWrite(LED_BUILTIN, HIGH);
      joy_msg.buttons = buttons_disarm;
      isarmed = false;
    }
  }
  else
  {
    if(!isarmed)
    {
      nh.loginfo("Arming");
      digitalWrite(LED_BUILTIN, LOW);
      joy_msg.buttons = buttons_arm;
      isarmed = true;
    }
  }
  joy_pub.publish(&joy_msg);
  nh.spinOnce();
  delay(100);
}
