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
  cv::FileStorage fs(ros::package::getPath("merlion_scripts") + "/configs/histogram.yaml", cv::FileStorage::READ);
  int hist_w = (int)fs["HistWindow.Width"];
  int hist_h = (int)fs["HistWindow.Height"];
  fs.release();

  // Process
  cv::namedWindow("image", cv::WINDOW_NORMAL);
  cv::startWindowThread();

  int hist_size = 256;
  float range[] = {0, 256};
  const float *hist_range = {range};
  int bin_w = cvRound((double)hist_w / hist_size);
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
    cv::Mat b_hist, g_hist, r_hist, frame_gr, gray_hist, frame_hsv, h_hist, s_hist, v_hist;
    cv::calcHist(&channels[0], 1, 0, cv::Mat(), b_hist, 1, &hist_size, &hist_range, true, false);
    cv::calcHist(&channels[1], 1, 0, cv::Mat(), g_hist, 1, &hist_size, &hist_range, true, false);
    cv::calcHist(&channels[2], 1, 0, cv::Mat(), r_hist, 1, &hist_size, &hist_range, true, false);

    cv::cvtColor(frame_src, frame_gr, cv::COLOR_BGR2GRAY);
    cv::calcHist(&frame_gr, 1, 0, cv::Mat(), gray_hist, 1, &hist_size, &hist_range, true, false);

    cv::cvtColor(frame_src, frame_hsv, cv::COLOR_BGR2HSV);
    cv::split(frame_hsv, channels);
    cv::calcHist(&channels[0], 1, 0, cv::Mat(), h_hist, 1, &hist_size, &hist_range, true, false);
    cv::calcHist(&channels[1], 1, 0, cv::Mat(), s_hist, 1, &hist_size, &hist_range, true, false);
    cv::calcHist(&channels[2], 1, 0, cv::Mat(), v_hist, 1, &hist_size, &hist_range, true, false);

    // Draw the histograms for B, G, R
    cv::Mat bgr_hist_image(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat gray_hist_image(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat hsv_hist_image(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

    /// Normalize the result to [0, histImage.rows]
    cv::normalize(b_hist, b_hist, 0, bgr_hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(g_hist, g_hist, 0, bgr_hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(r_hist, r_hist, 0, bgr_hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(gray_hist, gray_hist, 0, gray_hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(h_hist, h_hist, 0, hsv_hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(s_hist, s_hist, 0, hsv_hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat());
    cv::normalize(v_hist, v_hist, 0, hsv_hist_image.rows, cv::NORM_MINMAX, -1, cv::Mat());

    // Draw for each channel
    for (int i = 1; i < hist_size; i++)
    {
      cv::line(bgr_hist_image, cv::Point(bin_w * (i - 1), hist_h - cvRound(b_hist.at<float>(i - 1))),
               cv::Point(bin_w * (i), hist_h - cvRound(b_hist.at<float>(i))),
               cv::Scalar(255, 0, 0), 2, 8, 0);
      cv::line(bgr_hist_image, cv::Point(bin_w * (i - 1), hist_h - cvRound(g_hist.at<float>(i - 1))),
               cv::Point(bin_w * (i), hist_h - cvRound(g_hist.at<float>(i))),
               cv::Scalar(0, 255, 0), 2, 8, 0);
      cv::line(bgr_hist_image, cv::Point(bin_w * (i - 1), hist_h - cvRound(r_hist.at<float>(i - 1))),
               cv::Point(bin_w * (i), hist_h - cvRound(r_hist.at<float>(i))),
               cv::Scalar(0, 0, 255), 2, 8, 0);
      cv::line(gray_hist_image, cv::Point(bin_w * (i - 1), hist_h - cvRound(gray_hist.at<float>(i - 1))),
               cv::Point(bin_w * (i), hist_h - cvRound(gray_hist.at<float>(i))),
               cv::Scalar(255, 255, 255), 2, 8, 0);
      cv::line(hsv_hist_image, cv::Point(bin_w * (i - 1), hist_h - cvRound(h_hist.at<float>(i - 1))),
               cv::Point(bin_w * (i), hist_h - cvRound(h_hist.at<float>(i))),
               cv::Scalar(255, 0, 0), 2, 8, 0);
      cv::line(hsv_hist_image, cv::Point(bin_w * (i - 1), hist_h - cvRound(s_hist.at<float>(i - 1))),
               cv::Point(bin_w * (i), hist_h - cvRound(s_hist.at<float>(i))),
               cv::Scalar(0, 255, 0), 2, 8, 0);
      cv::line(hsv_hist_image, cv::Point(bin_w * (i - 1), hist_h - cvRound(v_hist.at<float>(i - 1))),
               cv::Point(bin_w * (i), hist_h - cvRound(v_hist.at<float>(i))),
               cv::Scalar(0, 0, 255), 2, 8, 0);
    }

    // Visualize
    cv::cvtColor(frame_gr, frame_gr, cv::COLOR_GRAY2BGR);
    cv::hconcat(frame_src, frame_gr, frame_src);
    cv::hconcat(frame_src, frame_hsv, frame_src);
    cv::hconcat(bgr_hist_image, gray_hist_image, bgr_hist_image);
    cv::hconcat(bgr_hist_image, hsv_hist_image, bgr_hist_image);
    cv::vconcat(frame_src, bgr_hist_image, frame_src);
    cv::imshow("image", frame_src);
    std::cout << "\rFrame took " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - t1).count() / 1000.0 << "ms" << std::flush;
    cv::waitKey(5); // waits to display frame
  }
  cv::waitKey(0);
  return 0;
}