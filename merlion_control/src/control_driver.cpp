#include <ros/ros.h>
#include <string.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <sensor_msgs/Joy.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <math.h>

using namespace std;

string node_name = "merlion_control_driver";

std::string topic_sub_joy = "/joy";
string topic_sub_cmd_vel = "/merlion/control/cmd_vel";
string topic_sub_cmd_vel_joy = "/merlion/control/cmd_vel_joy";

string topic_sub_pose_update = "/mavros/global_position/local";
string topic_sub_vis_odom = "/visual_odom";

string topic_pub_target_pose = "/merlion/control/target_pose";
string topic_pub_control = "/mavros/rc/override";

bool use_vis_z = false;
bool only_depth_control = true;

double target_alt = 0.0;
double alt_ctrl_signal = 0.0;

double curr_yaw = 1.57;
double last_yaw = 1.57;
double target_yaw = 1.57;
double yaw_ctrl_signal = 0.0;

geometry_msgs::Pose curr_pose;
geometry_msgs::Pose target_pose;
geometry_msgs::Pose rel_pose;
geometry_msgs::Twist target_vel;
tf::Transform target_tf;
ros::Time last_stamp_pose_update;
double pose_update_timeout = 1.0; //sec
ros::Time last_stamp_warning_pose_timemout;
double pose_timemout_warning_interval = 3.0; //sec

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

double cmd_vel_timeout = 0.2; // sec
ros::Time last_stamp_cmd_vel;
geometry_msgs::Twist curr_cmd_vel;

ros::Publisher pub_control;
ros::Publisher pub_target_pose;
bool has_target_pose = false;

bool autonomous = false;
bool use_cmd_vel_joy = true;

int chan_btn_arm = 7;
int chan_btn_dis = 6;
int chan_btn_man = 0;
int chan_btn_sta = 1;
int chan_btn_alt = 3;
int chan_btn_auto = 2;
int chan_btn_joy = 5;
int chan_btn_servo_on = 10; 
int chan_btn_servo_off = 9; 

bool servo_closed = true;

sensor_msgs::Joy joy_signal_last;
sensor_msgs::Joy joy_signal_curr;
int joy_count = 0;

int mode = 1000;
int MODE_MANUAL = 0;
int MODE_STABILIZE = 1000; // ppm in uS; from ArduSub/radio.cpp
int MODE_ALT_HOLD = 2000; // ppm in uS; from ArduSub/radio.cpp

// PID controller params
double kp_x = 0.2, ki_x = 0.0, kd_x = 0.0;
double kp_y = 0.2, ki_y = 0.0, kd_y = 0.0;
double kp_z = 0.2, ki_z = 0.0, kd_z = 0.0;
double kp_wz = 0.2, ki_wz = 0.0, kd_wz = 0.0;

geometry_msgs::Twist curr_err;
geometry_msgs::Twist last_err;
geometry_msgs::Twist accu_err;
geometry_msgs::Twist rate_err;

void cb_cmd_vel(geometry_msgs::Twist _cmd_vel);
void cb_cmd_vel_joy(geometry_msgs::Twist _cmd_vel);
void cb_joy_signal (const sensor_msgs::Joy joy_signal);
void cb_odom(nav_msgs::Odometry _odom);
void cb_pose_update(nav_msgs::Odometry _pose);

void send_control_cmd(bool in_plane, geometry_msgs::Twist target_vel);
uint16_t mapToPpm(double _in, double _max, double _min);
double normalize(double _in, double _max, double _min);
bool check_rising_edge(int channel);
void setArming(bool arm);
void setMode(string mode);

void process_cmd_vel();
void publish_target_pose();
geometry_msgs::Twist gen_twist(double data, int index);
geometry_msgs::Twist calculate_target_vel();

geometry_msgs::Pose twist_to_rel_pose(geometry_msgs::Twist _twist, double dt);

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

    nh_param.param<double>("kp_x", kp_x, kp_x); nh_param.param<double>("ki_x", ki_x, ki_x); nh_param.param<double>("kd_x", kd_x, kd_x);
    nh_param.param<double>("kp_y", kp_y, kp_y); nh_param.param<double>("ki_y", ki_y, ki_y); nh_param.param<double>("kd_y", kd_y, kd_y);
    nh_param.param<double>("kp_z", kp_z, kp_z); nh_param.param<double>("ki_z", ki_z, ki_z); nh_param.param<double>("kd_z", kd_z, kd_z);
    nh_param.param<double>("kp_wz", kp_wz, kp_wz); nh_param.param<double>("ki_wz", ki_wz, ki_wz); nh_param.param<double>("kd_wz", kd_wz, kd_wz);


    ros::Subscriber sub_cmd_vel = nh.subscribe<geometry_msgs::Twist>(topic_sub_cmd_vel, 10, cb_cmd_vel);
    ros::Subscriber sub_cmd_vel_joy = nh.subscribe<geometry_msgs::Twist>(topic_sub_cmd_vel_joy, 10, cb_cmd_vel_joy);
	ros::Subscriber sub_joy = nh.subscribe<sensor_msgs::Joy>(topic_sub_joy, 10, cb_joy_signal);

    ros::Subscriber sub_odom = nh.subscribe<nav_msgs::Odometry>(topic_sub_vis_odom, 10, cb_odom);
    ros::Subscriber sub_pose_update = nh.subscribe<nav_msgs::Odometry>(topic_sub_pose_update, 10, cb_pose_update);

    pub_control = nh.advertise<mavros_msgs::OverrideRCIn>(topic_pub_control, 10);
    pub_target_pose = nh.advertise<geometry_msgs::PoseStamped>(topic_pub_target_pose, 10);

    cmd_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ros::Rate loop_rate(50);

    while (ros::ok()){
        process_cmd_vel();
        // publish_target_pose();

        send_control_cmd(true, target_vel);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void cb_odom(nav_msgs::Odometry _odom){
    curr_pose.position.x = _odom.pose.pose.position.x;
    curr_pose.position.y = _odom.pose.pose.position.y;
    if (use_vis_z) curr_pose.position.z = _odom.pose.pose.position.z;

    curr_pose.orientation = _odom.pose.pose.orientation;
    last_stamp_pose_update = ros::Time::now();

    tf::Quaternion q(
    curr_pose.orientation.x,
    curr_pose.orientation.y,
    curr_pose.orientation.z,
    curr_pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    last_yaw = curr_yaw;
    curr_yaw = yaw;
}

void cb_pose_update(nav_msgs::Odometry _pose){
    if (!use_vis_z) curr_pose.position.z = _pose.pose.pose.position.z;
}

void cb_cmd_vel(geometry_msgs::Twist _cmd_vel){
    if (!use_cmd_vel_joy) {
        curr_cmd_vel = _cmd_vel;
        last_stamp_cmd_vel = ros::Time::now();
    }
}

void cb_cmd_vel_joy(geometry_msgs::Twist _cmd_vel){
    if (use_cmd_vel_joy) {
        curr_cmd_vel = _cmd_vel;
        last_stamp_cmd_vel = ros::Time::now();
    }
}

void process_cmd_vel(){
    // if (ros::Time::now().toSec() - last_stamp_cmd_vel.toSec() < cmd_vel_timeout){
    double dt = 0.2;
    geometry_msgs::Pose rel_pose = twist_to_rel_pose(curr_cmd_vel, dt);
    tf::Transform curr_tf, rel_tf;
    tf::poseMsgToTF(curr_pose, curr_tf);
    tf::poseMsgToTF(rel_pose, rel_tf);
    if (mode == MODE_ALT_HOLD){
        double temp_err = target_alt - curr_pose.position.z;
        alt_ctrl_signal = - temp_err * 800.0;
        if (alt_ctrl_signal >= 100.0) alt_ctrl_signal = 100.0;
        if (alt_ctrl_signal <= -100.0) alt_ctrl_signal = -100.0;

        ROS_INFO("Curr yaw vis: %.3f - Last yaw vis: %.3f", curr_yaw, last_yaw);

        if(!isnan(curr_yaw)) {
            double temp_err_yaw = target_yaw - curr_yaw;
            yaw_ctrl_signal = - temp_err_yaw * 200.0;
            if (yaw_ctrl_signal >= 100.0) yaw_ctrl_signal = 100.0;
            if (yaw_ctrl_signal <= -100.0) yaw_ctrl_signal = -100.0;

            ROS_INFO("yaw control");
        } else {
            yaw_ctrl_signal = 0.0;
        }

        last_err = curr_err;

        target_tf = curr_tf * rel_tf;
        target_vel = curr_cmd_vel;

        tf::poseTFToMsg(target_tf, target_pose);
    } else {
        alt_ctrl_signal = 0.0;
        yaw_ctrl_signal = 0.0;

        if (autonomous){
            if (ros::Time::now().toSec() - last_stamp_pose_update.toSec() > pose_update_timeout){
                if (ros::Time::now().toSec() - last_stamp_warning_pose_timemout.toSec() > pose_timemout_warning_interval){
                    ROS_ERROR("Attempt AUTONOMOUS mode without pose update. Reset velocity!");
                    last_stamp_warning_pose_timemout = ros::Time::now();
                }
                // reset target_vel
                geometry_msgs::Twist vel_reset;
                target_vel = vel_reset;

            } else {
                tf::Transform target_tf_last = target_tf;
                geometry_msgs::Pose target_pose_last;
                tf::poseTFToMsg(target_tf_last, target_pose_last);
                // target_tf = target_tf_last * rel_tf;
                if (fabs(curr_cmd_vel.linear.x) < 0.02 && fabs(curr_cmd_vel.linear.y) < 0.02 && fabs(curr_cmd_vel.linear.z) < 0.02 && 
                    fabs(curr_cmd_vel.angular.x) < 0.02 && fabs(curr_cmd_vel.angular.y) < 0.02 && fabs(curr_cmd_vel.angular.z) < 0.02)
                {
                    // Hold position
                } else {
                    
                    target_tf = curr_tf * rel_tf;

                    tf::poseTFToMsg(target_tf, target_pose);
                    if (fabs(curr_cmd_vel.linear.z) <= 0.02){
                        target_pose.position.z = target_pose_last.position.z;
                    }

                    if (fabs(curr_cmd_vel.angular.z) <= 0.02){
                        target_pose.orientation = target_pose_last.orientation;
                    }
                    tf::poseMsgToTF(target_pose, target_tf);
                    
                }

                target_vel = calculate_target_vel();
            }
        
        } else {
            target_tf = curr_tf * rel_tf;
            target_vel = curr_cmd_vel;

            tf::poseTFToMsg(target_tf, target_pose);
        }
    }
    has_target_pose = true;
    // }
}

geometry_msgs::Twist calculate_target_vel(){
    tf::Transform curr_tf;
    tf::poseMsgToTF(curr_pose, curr_tf);
    tf::Transform tf_curr_to_target = curr_tf.inverseTimes(target_tf); 
    double d_x = tf_curr_to_target.getOrigin().getX();
    double d_y = tf_curr_to_target.getOrigin().getY();
    double d_z = tf_curr_to_target.getOrigin().getZ();

    tf::Quaternion q = tf_curr_to_target.getRotation();

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    last_err = curr_err;

    curr_err.linear.x = d_x; 
    curr_err.linear.y = d_y; 
    curr_err.linear.z = d_z;
    curr_err.angular.x = roll; 
    curr_err.angular.y = pitch; 
    curr_err.angular.z = yaw;

    accu_err.linear.x = accu_err.linear.x + d_x;
    accu_err.linear.y = accu_err.linear.y + d_y;
    accu_err.linear.z = accu_err.linear.z + d_z;
    accu_err.angular.x = accu_err.angular.x + roll;
    accu_err.angular.y = accu_err.angular.y + pitch;
    accu_err.angular.z = accu_err.angular.z + yaw;
    
    rate_err.linear.x = curr_err.linear.x - last_err.linear.x;
    rate_err.linear.y = curr_err.linear.y - last_err.linear.y;
    rate_err.linear.z = curr_err.linear.z - last_err.linear.z;
    rate_err.angular.x = curr_err.angular.x - last_err.angular.x;
    rate_err.angular.y = curr_err.angular.y - last_err.angular.y;
    rate_err.angular.z = curr_err.angular.z - last_err.angular.z;
    
    geometry_msgs::Twist m_twist;
    m_twist.linear.x = curr_err.linear.x * kp_x + accu_err.linear.x * ki_x + rate_err.linear.x * kd_x;
    m_twist.linear.y = curr_err.linear.y * kp_y + accu_err.linear.y * ki_y + rate_err.linear.y * kd_y;
    m_twist.linear.z = curr_err.linear.z * kp_z + accu_err.linear.z * ki_z + rate_err.linear.z * kd_z;
    m_twist.angular.z = curr_err.angular.z * kp_wz + accu_err.angular.z * ki_wz + rate_err.angular.z * kd_wz;

    ROS_INFO("%.3f - %.3f - %.3f - %.3f", d_x, d_y, d_z, yaw);

    return m_twist;
}

geometry_msgs::Twist gen_twist(double data, int index){
    geometry_msgs::Twist m_twist;
    switch (index) {
        case 0: m_twist.linear.x = data; break;
        case 1: m_twist.linear.y = data; break;
        case 2: m_twist.linear.z = data; break;
        case 3: m_twist.angular.x = data; break;
        case 4: m_twist.angular.y = data; break;
        case 5: m_twist.angular.z = data; break;
    }
    return m_twist;
}

void publish_target_pose(){
    // geometry_msgs::PoseStamped msg;
    // msg.header.frame_id = "base_link";
    // msg.header.stamp = ros::Time::now();
    // msg.pose = rel_pose;
    // pub_target_pose.publish(msg);

    if (!has_target_pose) return;
    static tf::TransformBroadcaster br;
    try {
        br.sendTransform(tf::StampedTransform(target_tf, ros::Time::now(), "map", "target_pose"));
    } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        // ros::Duration(1.0).sleep();    
    }
}

geometry_msgs::Pose twist_to_rel_pose(geometry_msgs::Twist _twist, double dt){
    tf::Quaternion quat;
    quat.setRPY(_twist.angular.x * dt, _twist.angular.y * dt, _twist.angular.z * dt);

    geometry_msgs::Pose m_pose;
    m_pose.position.x = _twist.linear.x * dt;
    m_pose.position.y = _twist.linear.y * dt;
    m_pose.position.z = _twist.linear.z * dt;

    m_pose.orientation.x = quat.getX();
    m_pose.orientation.y = quat.getY();
    m_pose.orientation.z = quat.getZ();
    m_pose.orientation.w = quat.getW();

    return m_pose;
}

void send_control_cmd(bool in_plane, geometry_msgs::Twist target_vel){
    if (ros::Time::now().toSec() - last_stamp_cmd_vel.toSec() < cmd_vel_timeout){
        mavros_msgs::OverrideRCIn msg;

        msg.channels[4] = mapToPpm(directions[5] * lin_x_scaling * target_vel.linear.x, lin_max_vel, lin_min_vel);     // forward  (x)
        msg.channels[5] = mapToPpm(directions[6] * lin_y_scaling * target_vel.linear.y, lin_max_vel, lin_min_vel);     // strafe   (y)
        msg.channels[2] = mapToPpm(directions[2] * lin_z_scaling * target_vel.linear.z, lin_max_vel, lin_min_vel) + (int)alt_ctrl_signal;     // throttle (z)
        msg.channels[3] = mapToPpm(directions[3] * rot_z_scaling * target_vel.angular.z, rot_max_vel, rot_min_vel) + (int)yaw_ctrl_signal;    // yaw      (wz)

        if (in_plane) {
            msg.channels[1] = 1500; // roll     (wx)
            msg.channels[0] = 1500; // pitch    (wy)
        
        } else {
            msg.channels[1] = mapToPpm(directions[1] * rot_x_scaling * target_vel.angular.x, rot_max_vel, rot_min_vel); // roll     (wx)
            msg.channels[0] = mapToPpm(directions[0] * rot_y_scaling * target_vel.angular.y, rot_max_vel, rot_min_vel); // pitch    (wy)
        }
        
        msg.channels[6] = 1500; // mode         - not used (change to service)
        msg.channels[7] = (servo_closed ? 1100 : 1900); // camera-tilt  - not used

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
            target_alt = curr_pose.position.z;
            target_yaw = target_yaw;
            ROS_INFO("Attemp changing mode: ALT_HOLD");   
            // setMode("ALT_HOLD");
        }

        if (check_rising_edge(chan_btn_auto)){
            autonomous = !autonomous;
            if (autonomous) {
                ROS_ERROR("AUTONOMOUS MODE");
            } else {
                ROS_WARN("Exit autonomous");
            }   
        }

        if (check_rising_edge(chan_btn_joy)){
            use_cmd_vel_joy = !use_cmd_vel_joy;
            if (use_cmd_vel_joy) {
                ROS_WARN("Use cmd_vel from joystick");
            } else {
                ROS_ERROR("Use cmd_vel from nodes");
            }   
        }

        if (check_rising_edge(chan_btn_servo_on)){
            servo_closed = true;
            ROS_INFO("Servo closed");
        }
        
        if (check_rising_edge(chan_btn_servo_off)){
            servo_closed = false;
            ROS_INFO("Servo opened");
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