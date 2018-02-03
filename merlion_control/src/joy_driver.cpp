#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt16MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

std::string node_name = "merlion_joy_driver";

void joy_signal_cb (const sensor_msgs::Joy::ConstPtr &joy_signal);

std::string topic_sub_joy = "/joy";
std::string topic_pub_cmd = "/merlion/control/cmd_vel";


double cmd_scale_lin = 1.0;
double cmd_scale_ang = 1.0;

double curr_cmd_lin_x = 0.0;
double curr_cmd_lin_y = 0.0;
double curr_cmd_lin_z = 0.0;
double curr_cmd_ang_x = 0.0;
double curr_cmd_ang_y = 0.0;
double curr_cmd_ang_z = 0.0;

int chan_mode_switch = 8;

int chan_cmd_lin_x = 4;
int chan_cmd_lin_y = 3;
int chan_cmd_lin_z = 1;
// int chan_cmd_ang_x = 3;
// int chan_cmd_ang_y = 3;
int chan_cmd_ang_z = 0;

int chan_cmd_delta_lin = 7;
int chan_cmd_delta_ang = 6;

bool use_gamepad = true;
bool use_joystick = false;

int bound_value (int input, int upper_bound, int lower_bound);
double bound_value_double (double input, double upper_bound, double lower_bound);

int main(int argc, char** argv){
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh;
	ros::NodeHandle nh_param("~");

	nh_param.param<bool>("use_gamepad", use_gamepad, use_gamepad);
	nh_param.param<bool>("use_joystick", use_joystick, use_joystick);
	if (use_joystick) use_gamepad = false;
	if (use_gamepad) use_joystick = false;

	nh_param.param<std::string>("topic_sub_joy", topic_sub_joy, topic_sub_joy);
	nh_param.param<std::string>("topic_pub_cmd", topic_pub_cmd, topic_pub_cmd);

	nh_param.param<double>("cmd_scale_lin", cmd_scale_lin, cmd_scale_lin);
	nh_param.param<double>("cmd_scale_ang", cmd_scale_ang, cmd_scale_ang);

	ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>(topic_sub_joy, 10, joy_signal_cb);

	ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>(topic_pub_cmd, 10);
	
	ros::Rate control_rate (10);

	geometry_msgs::Twist vel_cmd_msg;

	if (use_gamepad){
		chan_mode_switch = 8;

		chan_cmd_lin_x = 4;
		chan_cmd_lin_y = 3;
		chan_cmd_lin_z = 1;
		chan_cmd_ang_z = 0;
		chan_cmd_delta_lin = 7;
		chan_cmd_delta_ang = 6;
		ROS_WARN("[JOY CTRL] Use Gamepad");
	} 
	if (use_joystick){
		chan_mode_switch = 6;

		chan_cmd_lin_x = 1;
		chan_cmd_lin_y = 0;
		chan_cmd_ang_z = 2;
		chan_cmd_delta_lin = 5;
		chan_cmd_delta_ang = 4;
		ROS_WARN("[JOY CTRL] Use Joystick");
	}

	while (ros::ok()){
        vel_cmd_msg.linear.x = curr_cmd_lin_x;
        vel_cmd_msg.linear.y = curr_cmd_lin_y;
        vel_cmd_msg.linear.z = curr_cmd_lin_z;
        
        vel_cmd_msg.angular.x = curr_cmd_ang_x;
        vel_cmd_msg.angular.y = curr_cmd_ang_y;
        vel_cmd_msg.angular.z = curr_cmd_ang_z;

        cmd_pub.publish(vel_cmd_msg);


		ros::spinOnce();
		control_rate.sleep();
	}

	return 0;
}

void joy_signal_cb (const sensor_msgs::Joy::ConstPtr &joy_signal){
    curr_cmd_lin_x = joy_signal->axes[chan_cmd_lin_x] * cmd_scale_lin;
    curr_cmd_lin_y = joy_signal->axes[chan_cmd_lin_y] * cmd_scale_lin;
    curr_cmd_lin_z = joy_signal->axes[chan_cmd_lin_z] * cmd_scale_lin;
    // curr_cmd_ang_x = joy_signal->axes[chan_cmd_ang_x] * cmd_scale_ang;
    // curr_cmd_ang_y = joy_signal->axes[chan_cmd_ang_y] * cmd_scale_ang;
    curr_cmd_ang_z = joy_signal->axes[chan_cmd_ang_z] * cmd_scale_ang;

    double delta_scale_lin = 0.02 * joy_signal->axes[chan_cmd_delta_lin];
    double delta_scale_ang = 0.02 * joy_signal->axes[chan_cmd_delta_ang];
    cmd_scale_lin += delta_scale_lin;
    cmd_scale_ang += delta_scale_ang;
    cmd_scale_lin = bound_value_double(cmd_scale_lin, 2.0, 0.0);
    cmd_scale_ang = bound_value_double(cmd_scale_ang, 2.0, 0.0);

    if (fabs(delta_scale_lin) >= 0.01) ROS_INFO("[JOY CTRL] Scale linear: %.3f", cmd_scale_lin);
    if (fabs(delta_scale_ang) >= 0.01) ROS_INFO("[JOY CTRL] Scale angular: %.3f", cmd_scale_ang);		
}

int bound_value (int input, int upper_bound, int lower_bound){
	if (input >= upper_bound) return upper_bound;
	if (input <= lower_bound) return lower_bound;
	return input;
}

double bound_value_double (double input, double upper_bound, double lower_bound){
	if (input >= upper_bound) return upper_bound;
	if (input <= lower_bound) return lower_bound;
	return input;
}