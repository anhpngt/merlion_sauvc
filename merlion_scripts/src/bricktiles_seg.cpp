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
  fs = cv::FileStorage(ros::package::getPath("merlion_scripts") + "/configs/line_detection.yaml", cv::FileStorage::READ);
  // Canny
  double canny_threshold1 = (double)fs["CannyEdge.threshold1"];
  double canny_threshold2 = (double)fs["CannyEdge.threshold2"];
  int canny_aperture_size = (int)fs["CannyEdge.apertureSize"];
  bool canny_L2gradient = (int)fs["CannyEdge.L2gradient"];
  // HoughLinesP
  double houghlp_rho = (double)fs["HoughLinesP.rho"];
  double houghlp_theta = (double)fs["HoughLinesP.theta"];
  int houghlp_threshold = (int)fs["HoughLinesP.threshold"];
  double houghlp_minlinelength = (double)fs["HoughLinesP.minLineLength"];
  double houghlp_maxlinegap = (double)fs["HoughLinesP.maxLineGap"];
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
    cv::equalizeHist(channels[select_channel], channels[select_channel]);
    cv::adaptiveThreshold(channels[select_channel], frame_binary, adT_maxValue, adT_method, adT_type, adT_blockSize, adT_C);
    cv::cvtColor(frame_binary, frame_threshold, cv::COLOR_GRAY2BGR);
    // cv::morphologyEx(frame_binary, frame_binary, cv::MORPH_CLOSE, str_el);
    cv::morphologyEx(frame_binary, frame_binary, cv::MORPH_OPEN, str_el);
    cv::Canny(frame_binary, frame_binary, canny_threshold1, canny_threshold2, canny_aperture_size, canny_L2gradient);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(frame_binary, lines, houghlp_rho, houghlp_theta, houghlp_threshold, houghlp_minlinelength, houghlp_maxlinegap);

    // Visualize
    for (size_t i = 0, i_end = lines.size(); i < i_end; i++)
    {
      int c1 = std::rand() % 256;
      int c2 = std::rand() % 256;
      int c3 = std::rand() % 256;
      cv::line(frame_src, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(c1, c2, c3), 2, 8);
    }

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