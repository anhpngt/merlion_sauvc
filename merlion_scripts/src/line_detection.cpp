#include <chrono>
#include <iostream>
#include <stdlib.h>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// void lineDetection(cv::Mat input_image)
// {

// }

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
  cv::FileStorage fs(ros::package::getPath("merlion_scripts") + "/configs/line_detection.yaml", cv::FileStorage::READ);

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
    cv::Mat frame_bw, frame_color;
    cv::Canny(frame_src, frame_bw, canny_threshold1, canny_threshold2, canny_aperture_size, canny_L2gradient);
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(frame_bw, lines, houghlp_rho, houghlp_theta, houghlp_threshold, houghlp_minlinelength, houghlp_maxlinegap);

    // Visualize
    cv::cvtColor(frame_bw, frame_color, cv::COLOR_GRAY2BGR);
    for (size_t i = 0, i_end = lines.size(); i < i_end; i++)
    {
      int c1 = std::rand() % 256;
      int c2 = std::rand() % 256;
      int c3 = std::rand() % 256;
      cv::line(frame_src, cv::Point(lines[i][0], lines[i][1]), cv::Point(lines[i][2], lines[i][3]), cv::Scalar(c1, c2, c3), 2, 8);
    }

    cv::hconcat(frame_src, frame_color, frame_src);
    cv::imshow("image", frame_src);
    std::cout << "\rFrame took " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - t1).count() / 1000.0 << "ms" << std::flush;
    cv::waitKey(5); // waits to display frame
  }
  cv::waitKey(0);
  return 0;
}