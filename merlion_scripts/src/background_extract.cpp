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
  if (argc != 2)
  {
    std::cout << "Error: invalid/missing input video." << std::endl;
    return -1;
  }

  cv::VideoCapture capture(argv[1]);
  cv::Mat frame_src;

  if (!capture.isOpened())
  {
    std::cout << "Error: Unable to read video." << std::endl;
    return -1;
  }

  // Get params
  cv::FileStorage fs(ros::package::getPath("merlion_scripts") + "/configs/background_extract.yaml", cv::FileStorage::READ);
  int extract_channel = (int)fs["ExtractChannel"];
  int blur_ksize = (int)fs["Blur.ksize"];
  fs.release();

  // Process
  cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  cv::startWindowThread();

  int hist_size = 256;
  float range[] = {0, 256};
  const float *hist_range = {range};
  for (;;)
  {
    std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
    capture >> frame_src;
    if (frame_src.empty())
    {
      std::cout << "\nEnd of video." << std::endl;
      break;
    }

    std::vector<cv::Mat> channels;
    cv::split(frame_src, channels);

    // Compute the histograms
    cv::Mat frame_equalized, frame_hsv, hist;
    cv::cvtColor(frame_src, frame_hsv, cv::COLOR_BGR2HSV);
    cv::split(frame_hsv, channels);
    cv::calcHist(&channels[extract_channel], 1, 0, cv::Mat(), hist, 1, &hist_size, &hist_range, true, false);
    cv::equalizeHist(channels[extract_channel], frame_equalized);

    // Draw the histograms for B, G, R
    cv::Mat hist_image(frame_src.rows, frame_src.cols, CV_8UC3, cv::Scalar(0, 0, 0));

    /// Normalize the result to [0, histImage.rows]
    cv::normalize(hist, hist, 0, hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat());
    
    // Smoothing
    cv::blur(hist, hist, cv::Size(1, blur_ksize));

    // Draw for each channel
    int bin_w = cvRound((double)frame_src.cols / hist_size);
    for (int i = 1; i < hist_size; i++)
    {
      cv::line(hist_image, cv::Point(bin_w * (i - 1), frame_src.rows - cvRound(hist.at<float>(i - 1))),
               cv::Point(bin_w * (i), frame_src.rows - cvRound(hist.at<float>(i))),
               cv::Scalar(255, 0, 0), 2, 8, 0);
    }

    // Visualize
    cv::hconcat(frame_src, hist_image, frame_src);
    cv::hconcat(channels[extract_channel], frame_equalized, frame_equalized);
    cv::cvtColor(frame_equalized, frame_equalized, cv::COLOR_GRAY2BGR);
    cv::vconcat(frame_src, frame_equalized, frame_src);
    cv::imshow("image", frame_src);
    std::cout << "\rFrame took " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - t1).count() / 1000.0 << "ms" << std::flush;
    cv::waitKey(5); // waits to display frame
  }
  cv::waitKey(0);
  return 0;
}