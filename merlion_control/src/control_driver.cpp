#include <ros/ros.h>
#include <string.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/Joy.h>

using namespace std;

string node_name = "merlion_control_driver";

std::string topic_sub_joy = "/joy";
string topic_sub_cmd_vel = "/merlion/control/cmd_vel";
string topic_pub_control = "/mavros/rc/override";

ros::ServiceClient cmd_client;
ros::ServiceClient set_mode_client;
enum {COMPONENT_ARM_DISARM=400}; // https://pixhawk.ethz.ch/mavlink/

double lin_max_vel  = 2.0; // m/s
double lin_min_vel  = -2.0; // m/s
double rot_max_vel  = -2.0; // m/s
double rot_min_vel  = 2.0; // m/s

double lin_x_scaling = 1.0;
double lin_y_scaling = 1.0;
double lin_z_scaling = 1.0;
double rot_x_scaling = 1.0;
double rot_y_scaling = 1.0;
double rot_z_scaling = 1.0;

double directions[7] = {
    1.0,    // pitch
    1.0,    // roll
    1.0,    // throttle
    1.0,    // yaw
    1.0,    // mode
    1.0,    // longitudinal
    1.0,    // lateral
};
double dir_y = 1.0;   // x
double dir_x = 1.0;   // y
double dir_z = 1.0;   // z
double dir_wx = 1.0;   // wy
double dir_wy = 1.0;   // wx
double dir_wz = 1.0;   // wz

double cmd_vel_timeout = 1.0; // sec
ros::Time last_stamp_cmd_vel;
geometry_msgs::Twist curr_cmd_vel;

ros::Publisher pub_control;

int chan_btn_arm = 7;
int chan_btn_dis = 6;
int chan_btn_man = 0;
int chan_btn_sta = 1;
int chan_btn_alt = 3;
sensor_msgs::Joy joy_signal_last;
sensor_msgs::Joy joy_signal_curr;
int joy_count = 0;

int mode = 1000;
int MODE_MANUAL = 0;
int MODE_STABILIZE = 1000; // ppm in uS; from ArduSub/radio.cpp
int MODE_ALT_HOLD = 2000; // ppm in uS; from ArduSub/radio.cpp

void cb_cmd_vel(geometry_msgs::Twist _cmd_vel);
void cb_joy_signal (const sensor_msgs::Joy joy_signal);

void send_control_cmd(bool in_plane);
uint16_t mapToPpm(double _in, double _max, double _min);
double normalize(double _in, double _max, double _min);
bool check_rising_edge(int channel);
void setArming(bool arm);
void setMode(string mode);

int main(int argc, char** argv){
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle nh_param("~");

    nh_param.param<std::string>("topic_sub_cmd_vel", topic_sub_cmd_vel, topic_sub_cmd_vel);
    nh_param.param<std::string>("topic_pub_control", topic_pub_control, topic_pub_control);
    nh_param.param<double>("cmd_vel_timeout", cmd_vel_timeout, cmd_vel_timeout);
	nh_param.param<std::string>("topic_sub_joy", topic_sub_joy, topic_sub_joy);

    nh_param.param<double>("dir_y", directions[6], directions[6]);
    nh_param.param<double>("dir_x", directions[5], directions[5]);
    nh_param.param<double>("dir_z", directions[2], directions[2]);
    nh_param.param<double>("dir_wx", directions[1], directions[1]);
    nh_param.param<double>("dir_wy", directions[0], directions[0]);
    nh_param.param<double>("dir_wz", directions[3], directions[3]);

    ros::Subscriber sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>(topic_sub_cmd_vel, 10, cb_cmd_vel);
	ros::Subscriber sub_joy = nh.subscribe<sensor_msgs::Joy>(topic_sub_joy, 10, cb_joy_signal);

    pub_control = nh.advertise<mavros_msgs::OverrideRCIn>(topic_pub_control, 10);

    cmd_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Rate loop_rate(50);

    while (ros::ok()){
        send_control_cmd(true);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void cb_cmd_vel(geometry_msgs::Twist _cmd_vel){
    last_stamp_cmd_vel = ros::Time::now();
    curr_cmd_vel = _cmd_vel;
}

void send_control_cmd(bool in_plane){
    if (ros::Time::now().toSec() - last_stamp_cmd_vel.toSec() < cmd_vel_timeout){
        mavros_msgs::OverrideRCIn msg;

        msg.channels[5] = mapToPpm(directions[5] * lin_x_scaling * curr_cmd_vel.linear.x, lin_max_vel, lin_min_vel);     // forward  (x)
        msg.channels[6] = mapToPpm(directions[6] * lin_y_scaling * curr_cmd_vel.linear.y, lin_max_vel, lin_min_vel);     // strafe   (y)
        msg.channels[2] = mapToPpm(directions[2] * lin_z_scaling * curr_cmd_vel.linear.z, lin_max_vel, lin_min_vel);     // throttle (z)
        msg.channels[3] = mapToPpm(directions[3] * rot_z_scaling * curr_cmd_vel.angular.z, rot_max_vel, rot_min_vel);    // yaw      (wz)

        if (in_plane) {
            msg.channels[1] = 1500; // roll     (wx)
            msg.channels[0] = 1500; // pitch    (wy)
        
        } else {
            msg.channels[1] = mapToPpm(directions[1] * rot_x_scaling * curr_cmd_vel.angular.x, rot_max_vel, rot_min_vel); // roll     (wx)
            msg.channels[0] = mapToPpm(directions[0] * rot_y_scaling * curr_cmd_vel.angular.y, rot_max_vel, rot_min_vel); // pitch    (wy)
        }
        
        msg.channels[4] = 1500; // mode         - not used (change to service)
        msg.channels[7] = 1500; // camera-tilt  - not used

        pub_control.publish(msg);

    }
}

void cb_joy_signal (const sensor_msgs::Joy joy_signal){		
    if (joy_count < 1){
        joy_signal_curr = joy_signal;
        joy_count++;
    } else {
        joy_signal_last = joy_signal_curr;
        joy_signal_curr = joy_signal;

        if (check_rising_edge(chan_btn_arm)){
            ROS_WARN("Attempt: ARMING");  
            setArming(true);
        }

        if (check_rising_edge(chan_btn_dis)){
            ROS_WARN("Attemp: DISARMING");
            setArming(false);
        }

        if (check_rising_edge(chan_btn_man)){
            mode = MODE_MANUAL;
            ROS_INFO("Attemp changing mode: MANUAL");
            setMode("MANUAL");
        }

        if (check_rising_edge(chan_btn_sta)){
            mode = MODE_STABILIZE;
            ROS_INFO("Attemp changing mode: STABILIZE");   
            setMode("STABILIZE");
        }

        if (check_rising_edge(chan_btn_alt)){
            mode = MODE_ALT_HOLD;
            ROS_INFO("Attemp changing mode: ALT_HOLD");   
            setMode("ALT_HOLD");
        }
    }
}

bool check_rising_edge(int channel){
    if (channel < 0 || channel >= 11) {
        ROS_ERROR("Invalid channel: %d!", channel);
        return false;
    } else {
        if (joy_signal_last.buttons[channel] == 0 && joy_signal_curr.buttons[channel] == 1) return true;
        else return false;
    }
}


void setArming(bool arm) {
  // Arm/disarm method following:
  // https://github.com/mavlink/qgroundcontrol/issues/590
  // https://pixhawk.ethz.ch/mavlink/#MAV_CMD_COMPONENT_ARM_DISARM

  // generate request
  mavros_msgs::CommandLong srv;
  srv.request.command = COMPONENT_ARM_DISARM;
  srv.request.param1 = (arm ? 1 : 0);
  srv.request.param2 = 21196; // force disarm (see GCS_Mavlink.cpp)

  // send request
  if(cmd_client.call(srv)) {
    ROS_INFO(arm ? "Armed" : "Disarmed");
  }
  else {
    ROS_ERROR("Failed to update arming");
  }
}

void setMode(string mode){
    mavros_msgs::SetMode set_mode_cmd;
    set_mode_cmd.request.custom_mode = mode;

    if( set_mode_client.call(set_mode_cmd) && set_mode_cmd.response.mode_sent){
        ROS_WARN("Mode set: %s", mode.c_str());
    } else {
        ROS_ERROR("Failed to set mode: %s", mode.c_str());
    }
}

uint16_t mapToPpm(double _in, double _max, double _min) {
    // in should be min to max
    // converting to -1 to 1
    // out should be 1000 to 2000 (microseconds)
    double in = normalize(_in, _max, _min);

    uint16_t out = 1000 + (in + 1.0) * 500;

    if(out > 2000) return 2000;
    else if(out < 1000) return 1000;
    else return out;
}

double normalize(double _in, double _max, double _min){
    double in = _in;
    double max, min;
    
    // sanity check
    if (_max >= _min) {
        max = _max;
        min = _min;
    } else {
        max = _min;
        min = _max;
    }
    
    if (in >= max) in = max;
    if (in <= min) in = min;

    double range = max - min;

    return (-1.0 + 2.0 * (in - min) / range); 
}