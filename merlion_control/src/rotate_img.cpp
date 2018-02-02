#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
// #include "opencv2/xfeatures2d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"

#include "string.h"
#include <algorithm>

using namespace cv;
using namespace std;
// using namespace cv::xfeatures2d;

static const std::string OPENCV_WINDOW = "Image window";

Mat curr_img;
Mat curr_rot;
Mat curr_out;
double curr_yaw = 0.0;
double initial_offset = 0.0;
// double initial_offset = 0.0;
double visual_correction = 0.0;

void imageCb(const sensor_msgs::ImageConstPtr& msg);
void imuCb(const sensor_msgs::Imu::ConstPtr& _imu);
void localPoseCb(const nav_msgs::Odometry _pose);
double radToDeg(double rad);
double wrapDeg(double deg);
Mat rotate(Mat src, double angle);
Mat localize(Mat src);
Mat cropCircle(Mat img);
Mat drawCrossHair(Mat src);
int getBinIndex(double val, double min, double max, int n_int);
void publish_pose();
double bound(double in);

double threshold_x = 5.0;
double last_min_dist_x_3 = 100.0;
double last_min_dist_x_2 = 100.0;
double last_min_dist_x_1 = 100.0;
double curr_min_dist_x = 100.0;

double threshold_y = 5.0;
double last_min_dist_y_3 = 100.0;
double last_min_dist_y_2 = 100.0;
double last_min_dist_y_1 = 100.0;
double curr_min_dist_y = 100.0;

double tile_size_x = 0.050; //m
double tile_size_y = 0.150; //m

int tile_count_x = 0;
int tile_count_y = 0;

double curr_x = 0.0;
double curr_y = 0.0;

ros::Publisher pose_pub;
geometry_msgs::Quaternion curr_quat;

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_img = it.subscribe("/logi_c310/usb_cam_node/image_raw", 10, imageCb);
    image_transport::Publisher pub_img = it.advertise("/image_rotated", 1);

    ros::Subscriber sub_imu = nh.subscribe("/mavros/imu/data", 10, imuCb);
    ros::Subscriber sub_pose = nh.subscribe("/mavros/global_position/local", 10, localPoseCb);

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/merlion/pose_update", 10);

    ros::Rate loop_rate(20);
    while (nh.ok()) {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", curr_out).toImageMsg();
        pub_img.publish(msg);

        publish_pose();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat curr_cropped = cropCircle(cv_ptr->image);
        curr_rot = rotate(curr_cropped, curr_yaw); 
        Mat temp = localize(cv_ptr->image);
        curr_out = rotate(curr_cropped, visual_correction);
        curr_out = drawCrossHair(curr_out);

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void localPoseCb(const nav_msgs::Odometry _pose){
    // tf::Quaternion q(
    //     _pose.pose.pose.orientation.x,
    //     _pose.pose.pose.orientation.y,
    //     _pose.pose.pose.orientation.z,
    //     _pose.pose.pose.orientation.w);
    // tf::Matrix3x3 m(q);
    // double roll, pitch, yaw;
    // m.getRPY(roll, pitch, yaw);

    // curr_yaw = wrapDeg(radToDeg(yaw) - initial_offset);

    // ROS_INFO("Current hdg: %.3f deg", curr_yaw);
}

void imuCb(const sensor_msgs::Imu::ConstPtr& _imu){
    curr_quat = _imu->orientation;
    tf::Quaternion q(
        _imu->orientation.x,
        _imu->orientation.y,
        _imu->orientation.z,
        _imu->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    curr_yaw = wrapDeg(radToDeg(yaw) + initial_offset);

    // ROS_INFO("Current hdg: %.3f deg", curr_yaw);
}

void publish_pose(){
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time::now();

    msg.pose.position.x = curr_x;
    msg.pose.position.y = curr_y;
    msg.pose.position.z = 0.0;
    msg.pose.orientation = curr_quat;

    pose_pub.publish(msg);

    static tf::TransformBroadcaster br;
    tf::Transform transform = tf::Transform(tf::Quaternion(msg.pose.orientation.x,
                            msg.pose.orientation.y,
                            msg.pose.orientation.z,
                            msg.pose.orientation.w),
                    tf::Vector3(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z));

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "merlion"));
}

double radToDeg(double rad){
    return rad/3.14*180.0;
}

double wrapDeg(double deg){
    if (deg < -180.0) return (360.0 + deg);
    else if (deg > 180.0) return (deg - 360);
    else return deg;
}

Mat rotate(Mat src, double angle){
    Mat dst;
    Point2f pt(src.cols/2., src.rows/2.);    
    Mat r = getRotationMatrix2D(pt, angle, 1.0);
    warpAffine(src, dst, r, Size(src.cols, src.rows));
    return dst;
}

Mat cropCircle(Mat src){
    Mat dst;
    int radius = src.rows / 2.;
    Point2f center(src.cols / 2., src.rows / 2.);   
    Mat mask = Mat::zeros( src.rows, src.cols, CV_8UC1 );
    circle( mask, center, radius, Scalar(255,255,255), -1, 8, 0 ); //-1 means filled
    src.copyTo( dst, mask ); // copy values of img to dst if mask is > 0.
    return dst;
}

Mat drawCrossHair(Mat src){
    Mat dst;
    src.copyTo(dst);
    Point2f pw1(src.rows/2.,    0.);
    Point2f pw2(src.rows/2.,    src.cols);
    Point2f pv1(0.,             src.cols/2.);
    Point2f pv2(src.rows,       src.cols/2.);
    line(dst, pw1, pw2, Scalar(0, 0, 255), 3, 8);
    line(dst, pv1, pv2, Scalar(0, 0, 255), 3, 8);
    return dst;
}

Mat localize(Mat src){

    Mat dst, cdst;
    Canny(src, dst, 50, 200, 3);
    cvtColor(dst, cdst, CV_GRAY2BGR);

    Mat canny_output = dst;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Find contours
    Rect best_rect;
    double min_area = 1000.0;
    double min_dist_from_center = 300;
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    double sum_width = 0.0;
    double sum_height = 0.0;
    int n_rect = 0;
    for (int i = 0; i < contours.size(); i++){
        Rect rect = boundingRect(contours[i]);
        double temp_x = ((float) rect.x + (float) rect.width) / 2.0;
        double temp_y = ((float) rect.y + (float) rect.height) / 2.0;
        double dist_from_center = sqrt(temp_x * temp_x + temp_y * temp_y);
        double area = (float)rect.width * (float)rect.height;
        if (area > min_area){
            sum_width += (float)(rect.width);
            sum_height += (float)(rect.height);
            n_rect ++;
        }
    }
    double tile_width = sum_width / (float)n_rect;
    double tile_height = sum_height / (float)n_rect;

    vector<Vec2f> lines;
    HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

    int n_bins = 18;
    vector<int> count(n_bins);
    vector< vector<double> > grads(n_bins);
    vector< vector<double> > dists(n_bins);

    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0];
        float theta = lines[i][1];

        int bin_ind = getBinIndex(theta, 0.0, 3.14, n_bins);
        count[bin_ind]++;
        grads[bin_ind].push_back(theta);
        dists[bin_ind].push_back(rho);

    }
    
    int bin_1st = 0; 
    int bin_2nd = 0;
    int max_count = 0;
    for (int i = 0; i < n_bins; i++){
        if (count[i] > max_count){
            max_count = count[i];
            bin_2nd = bin_1st;
            bin_1st = i;
        }
    }

    double temp_sum_grad_x = 0.0;
    double min_dist_x = 100.0;
    for (int i = 0; i < grads[bin_1st].size(); i++){
        temp_sum_grad_x += grads[bin_1st][i];
        if (dists[bin_1st][i] < min_dist_x && dists[bin_1st][i] > 0.0) 
            min_dist_x = dists[bin_1st][i];
    }
    double bin_1st_grad = temp_sum_grad_x / (float)(grads[bin_1st].size());
    double vis_grad_x_deg = radToDeg(bin_1st_grad);
    visual_correction = vis_grad_x_deg - 90.0;

    last_min_dist_x_3 = last_min_dist_x_2;
    last_min_dist_x_2 = last_min_dist_x_1;
    last_min_dist_x_1 = curr_min_dist_x;
    curr_min_dist_x = min_dist_x;

    // if (curr_min_dist_x > last_min_dist_x_2 + threshold_x && 
    //     last_min_dist_x_1 > last_min_dist_x_2 + threshold_x &&
    //     curr_min_dist_x > last_min_dist_x_3 + threshold_x && 
    //     last_min_dist_x_1 > last_min_dist_x_3 + threshold_x) tile_count_x++;

    // if (curr_min_dist_x < last_min_dist_x_2 - threshold_x &&
    //     last_min_dist_x_1 < last_min_dist_x_2 - threshold_x &&
    //     curr_min_dist_x < last_min_dist_x_3 - threshold_x &&
    //     last_min_dist_x_1 < last_min_dist_x_3 - threshold_x) tile_count_x--;

    if (curr_min_dist_x > last_min_dist_x_1 + threshold_x && 
        curr_min_dist_x > last_min_dist_x_2 + threshold_x && 
        curr_min_dist_x > last_min_dist_x_3 + threshold_x ) tile_count_x++;
    if (curr_min_dist_x < last_min_dist_x_1 - threshold_x && 
        curr_min_dist_x < last_min_dist_x_2 - threshold_x && 
        curr_min_dist_x < last_min_dist_x_3 - threshold_x) tile_count_x--;
    
    double temp_sum_grad_y = 0.0;
    double min_dist_y = 100.0;
    for (int i = 0; i < grads[bin_2nd].size(); i++){
        temp_sum_grad_y += grads[bin_2nd][i];
        if (dists[bin_2nd][i] < min_dist_y && dists[bin_2nd][i] > 0.0) 
            min_dist_y = dists[bin_2nd][i];
    }
    double bin_2nd_grad = temp_sum_grad_y / (float)(grads[bin_2nd].size());
    double vis_grad_y_deg = radToDeg(bin_2nd_grad);

    last_min_dist_y_3 = last_min_dist_y_2;
    last_min_dist_y_2 = last_min_dist_y_1;
    last_min_dist_y_1 = curr_min_dist_y;
    curr_min_dist_y = min_dist_y;

    // if (curr_min_dist_y > last_min_dist_y_2 + threshold_y && 
    //     last_min_dist_y_1 > last_min_dist_y_2 + threshold_y &&
    //     curr_min_dist_y > last_min_dist_y_3 + threshold_y && 
    //     last_min_dist_y_1 > last_min_dist_y_3 + threshold_y) tile_count_y++;

    // if (curr_min_dist_y < last_min_dist_y_2 - threshold_y &&
    //     last_min_dist_y_1 < last_min_dist_y_2 - threshold_y &&
    //     curr_min_dist_y < last_min_dist_y_3 - threshold_y &&
    //     last_min_dist_y_1 < last_min_dist_y_3 - threshold_y) tile_count_y--;

    if (curr_min_dist_y > last_min_dist_y_1 + threshold_y && 
        curr_min_dist_y > last_min_dist_y_2 + threshold_y && 
        curr_min_dist_y > last_min_dist_y_3 + threshold_y ) tile_count_y++;
    if (curr_min_dist_y < last_min_dist_y_1 - threshold_y &&
        curr_min_dist_y < last_min_dist_y_2 - threshold_y && 
        curr_min_dist_y < last_min_dist_y_3 - threshold_y ) tile_count_y--;

    // std::cout << "Grad X: " << vis_grad_x_deg << "\t Grad Y: " << vis_grad_y_deg << endl;
    
    // std::cout << "Visual correction: " << vis_grad_deg << "\t Count: " << grads[bin_1st].size() << endl;
    // std::cout << "Min pos: " << min_dist_x << "\t Min neg: " << min_dist_neg << " \t Box size: " << min_dist_x + fabs(min_dist_neg) << endl;
    // std::cout << "Width: " << tile_width << "\t Height: " << tile_height <<  endl;

    curr_x = ((float)tile_count_x + bound(curr_min_dist_x / tile_height))* tile_size_x; 
    curr_y = ((float)tile_count_y + bound(curr_min_dist_y / tile_width))* tile_size_y; 

    // std::cout << "Tile x: " << tile_count_x << " \t Tile y: " << tile_count_y << endl;
    std::cout << "X: " << curr_x << " \t Y: " << curr_y << endl;
    
    return src;
}

int getBinIndex(double val, double min, double max, int n_int){
     double interval = ( max - min ) / float(n_int);
     int bin_index =  (int)(( val - min ) / interval );
}

double bound(double in){
    if (in > 1.0) return 1.0;
    else return in;
}