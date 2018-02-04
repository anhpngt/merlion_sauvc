#include <chrono>
#include <iostream>
#include <stdlib.h>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

image_transport::Publisher image_pub;
cv::Mat frame_src;
cv::Mat str_el;

// Params
std::string camera_image_topic = "/front/image_rect_color";
std::string result_image_topic = "/gate/image";
double adT_maxValue = 255;
double adT_C = 3;
int adT_method = 1;
int adT_type = 1;
int adT_blockSize = 51;
int select_channel = 2;
int blur_ksize = 5;
int morph_ksize = 3;

void imageCb(const sensor_msgs::ImageConstPtr& img_msg)
{
  std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
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

  // Publish
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, frame_binary).toImageMsg();
  image_pub.publish(msg);

  ROS_INFO("Frame took %.4lf ms.", (double)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - t1).count() / 1000.0);
}

int main(int argc, char **argv)
{
  // Initialize
  ros::init(argc, argv, "gate_thresholder");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.getParam("camera_image_topic", camera_image_topic);
  pnh.getParam("result_image_topic",result_image_topic);
  pnh.getParam("adT_maxValue", adT_maxValue);
  pnh.getParam("adT_C", adT_C);
  pnh.getParam("adT_method", adT_method);
  pnh.getParam("adT_type", adT_type);
  pnh.getParam("adT_blockSize", adT_blockSize);
  pnh.getParam("select_channel", select_channel);
  pnh.getParam("blur_ksize", blur_ksize);
  pnh.getParam("morph_ksize", morph_ksize);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber img_sub = it.subscribe(camera_image_topic, 1, imageCb);
  image_pub = it.advertise(result_image_topic, 1);

  // Process
  str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_ksize, morph_ksize));
  
  ros::spin();
  return 0;
}