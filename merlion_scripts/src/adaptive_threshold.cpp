#include <chrono>
#include <iostream>
#include <stdlib.h>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
  if(argc != 2)
  {
    std::cout << "Error: invalid/missing input video." << std::endl;
    return -1;
  }

  cv::VideoCapture capture(argv[1]);
  cv::Mat frame_src;

  if(!capture.isOpened())
  {
    std::cout << "Error: Unable to read video." << std::endl;
    return -1;
  }

  // Get params
  cv::FileStorage fs(ros::package::getPath("merlion_scripts") + "/configs/adaptive_threshold.yaml", cv::FileStorage::READ);
  double adT_maxValue = (double)fs["AdaptiveThreshold.maxValue"];
  int adT_method = (int)fs["AdaptiveThreshold.method"];
  int adT_type = (int)fs["AdaptiveThreshold.type"];
  int adT_blockSize = (int)fs["AdaptiveThreshold.blockSize"];
  double adT_C = (double)fs["AdaptiveThreshold.C"];
  int select_channel = (int)fs["AdaptiveThreshold.channel"];
  int blur_ksize = (int)fs["GaussianBlur.ksize"];
  int morph_ksize = (int)fs["MorphEx.ksize"];
  fs.release();  

  // Process
  cv::namedWindow("image", 1);
  cv::startWindowThread();
  cv::Mat str_el = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_ksize, morph_ksize));
  for(;;)
  {
    std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
    capture >> frame_src;
    if (frame_src.empty())
    {
      std::cout << "\nEnd of video." << std::endl;
      break;
    }

    // Find lines
    cv::Mat frame_hsv, frame_binary, s_frame, frame_selectedchannel, frame_threshold;
    std::vector<cv::Mat> channels;
    cv::cvtColor(frame_src, frame_hsv, cv::COLOR_BGR2HSV);
    cv::split(frame_hsv, channels);
    cv::GaussianBlur(channels[select_channel], channels[select_channel], cv::Size(blur_ksize, blur_ksize), 0);
    cv::adaptiveThreshold(channels[select_channel], frame_binary, adT_maxValue, adT_method, adT_type, adT_blockSize, adT_C);
    cv::cvtColor(frame_binary, frame_threshold, cv::COLOR_GRAY2BGR);
    cv::morphologyEx(frame_binary, frame_binary, cv::MORPH_CLOSE, str_el);
    cv::morphologyEx(frame_binary, frame_binary, cv::MORPH_OPEN, str_el);

    // Visualize
    cv::cvtColor(channels[select_channel], frame_selectedchannel, cv::COLOR_GRAY2BGR);
    cv::cvtColor(frame_binary, frame_binary, cv::COLOR_GRAY2BGR);
    cv::hconcat(frame_src, frame_selectedchannel, frame_src);
    cv::hconcat(frame_threshold, frame_binary, frame_binary);
    cv::vconcat(frame_src, frame_binary, frame_src);
    cv::imshow("image", frame_src);
    std::cout << "\rFrame took " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - t1).count() / 1000.0 << "ms" << std::flush;
    cv::waitKey(5); // waits to display frame
  }
  cv::waitKey(0);
  return 0;
}