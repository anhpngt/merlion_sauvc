#include <chrono>
#include <iostream>
#include <stdlib.h>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Twist.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static image_transport::Publisher image_pub;
static ros::Publisher vel_pub;
static cv::Mat frame_src;
static cv::Mat str_el;
static double min_area = 300;

// Params
static std::string camera_image_topic = "/front/image_rect_color";
static std::string out_image_topic = "/flare/image";
static std::string out_vel_topic = "/merlion/control/cmd_vel";
static double adT_maxValue = 255;
static double adT_C = 3;
static int adT_method = 1;
static int adT_type = 1;
static int adT_blockSize = 51;
static int select_channel = 2;
static int blur_ksize = 5;
static int morph_ksize = 3;
static double vel_scaling = 0.5;

static void imageCb(const sensor_msgs::ImageConstPtr& img_msg)
{
  try
  {
    frame_src = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8)->image;
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  //Get the image in OpenCV format
  if (frame_src.empty())
  {
    ROS_WARN("Empty image");
    return;
  }

  // Find lines
  cv::Mat frame_hsv, frame_binary;
  std::vector<cv::Mat> channels;
  cv::cvtColor(frame_src, frame_hsv, cv::COLOR_BGR2HSV);
  cv::split(frame_hsv, channels);
  cv::GaussianBlur(channels[select_channel], channels[select_channel], cv::Size(blur_ksize, blur_ksize), 0);
  cv::adaptiveThreshold(channels[select_channel], frame_binary, adT_maxValue, adT_method, adT_type, adT_blockSize, adT_C);
  cv::morphologyEx(frame_binary, frame_binary, cv::MORPH_CLOSE, str_el);
  cv::morphologyEx(frame_binary, frame_binary, cv::MORPH_OPEN, str_el);

  std::vector<std::vector<cv::Point> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(frame_binary.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

  // Process each found contour
  cv::Rect best_detection;
  double max_area = 0;
  for (int i = 0; i < contours.size(); i++)
  {
    // Skip small objects 
    if (std::fabs(cv::contourArea(contours[i])) < min_area) continue;

    std::vector<cv::Point> hull;
    cv::convexHull(contours[i], hull, 0, 1);
    cv::Rect rect = cv::boundingRect(contours[i]);
    cv::RotatedRect mr = cv::minAreaRect(contours[i]);

    double area = cv::contourArea(contours[i]);
    double mr_area = (mr.size).height * (mr.size).width;
    double hull_area = cv::contourArea(hull);


    if(std::fabs(area / mr_area) > 0.2
       && std::fabs(area / hull_area) > 0.2
       && (3 * rect.height > frame_src.rows))
    {
      if(area > max_area)
      {
        best_detection = rect;
        max_area = area;
      }
      cv::rectangle(frame_src, rect.tl(), rect.br() - cv::Point(1,1), cv::Scalar(150,150,150), 2, 8, 0);
    }
    if(max_area > 0)
    { 
      cv::rectangle(frame_src, best_detection.tl(), best_detection.br() - cv::Point(1,1), cv::Scalar(0,255,0), 2, 8, 0);

      // Give cmd_vel msg
      geometry_msgs::Twist cmd_vel_msg;
      cv::Point rect_center = (best_detection.tl() + best_detection.br()) / 2;
      double location = ((double)rect_center.x - frame_src.cols / 2) / frame_src.cols;
      if(location > 0.15)
      {
        cmd_vel_msg.linear.y = vel_scaling;
      }
      else if(location < -0.15)
      {
        cmd_vel_msg.linear.y = -vel_scaling;
      }
      else
      {
        cmd_vel_msg.linear.x = vel_scaling;
      }
      vel_pub.publish(cmd_vel_msg);
    }
  }

  // Publish
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, frame_src).toImageMsg();
  image_pub.publish(msg);
}

int main(int argc, char **argv)
{
  // Initialize
  ros::init(argc, argv, "flare_finder");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("camera_image_topic", camera_image_topic);
  pnh.getParam("out_image_topic",out_image_topic);
  pnh.getParam("out_vel_topic", out_vel_topic);
  pnh.getParam("adT_maxValue", adT_maxValue);
  pnh.getParam("adT_C", adT_C);
  pnh.getParam("adT_method", adT_method);
  pnh.getParam("adT_type", adT_type);
  pnh.getParam("adT_blockSize", adT_blockSize);
  pnh.getParam("select_channel", select_channel);
  pnh.getParam("blur_ksize", blur_ksize);
  pnh.getParam("morph_ksize", morph_ksize);
  pnh.getParam("vel_scaling", vel_scaling);

  vel_pub = nh.advertise<geometry_msgs::Twist>(out_vel_topic, 1);
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber img_sub = it.subscribe(camera_image_topic, 1, imageCb);
  image_pub = it.advertise(out_image_topic, 1);

  // Process
  str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_ksize, morph_ksize));
  
  ros::spin();
  return 0;
}