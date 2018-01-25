#include <chrono>
#include <iostream>
#include <stdlib.h>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

struct distributionModel
{
  cv::Vec3b mean;
  cv::Vec3b std;
  uint sample_size;
  std::vector<cv::Vec3b> data;
};

cv::Mat frame_src, frame_hsv;
distributionModel background;

static void onMouseCb(int event, int x, int y, int flag, void*)
{
  if (event != cv::EVENT_LBUTTONDOWN && event != cv::EVENT_RBUTTONDOWN)
  {
    return;
  }

  if(event == cv::EVENT_LBUTTONDOWN)
  {
    background.data.push_back(frame_hsv.at<cv::Vec3b>(cv::Point(x, y)));
  }

  if(event == cv::EVENT_RBUTTONDOWN)
  {
    if (background.data.size() > 0)
    {
      background.data.pop_back();
    }
  }

  // Since there is a change in the model, re-calculate
  if (background.sample_size != background.data.size())
  {
    float sample_mean[3] = {0, 0, 0};
    float sample_std[3] = {0, 0, 0};
    for (int i = 0, i_end = background.data.size(); i < i_end; i++)
    {
      sample_mean[0] = sample_mean[0] + background.data[i][0];
      sample_mean[1] = sample_mean[1] + background.data[i][1];
      sample_mean[2] = sample_mean[2] + background.data[i][2];
    }

    sample_mean[0] = sample_mean[0] / (float)background.data.size();
    sample_mean[1] = sample_mean[1] / (float)background.data.size();
    sample_mean[2] = sample_mean[2] / (float)background.data.size();

    for(int i = 0, i_end = background.data.size(); i < i_end; i++)
    {
      sample_std[0] += (background.data[i][0] - sample_mean[0]) * (background.data[i][0] - sample_mean[0]);
      sample_std[1] += (background.data[i][1] - sample_mean[1]) * (background.data[i][1] - sample_mean[1]);
      sample_std[2] += (background.data[i][2] - sample_mean[2]) * (background.data[i][2] - sample_mean[2]);
    }

    sample_std[0] = sqrt(sample_std[0] / (float)background.data.size());
    sample_std[1] = sqrt(sample_std[1] / (float)background.data.size());
    sample_std[2] = sqrt(sample_std[2] / (float)background.data.size());

    background.mean = cv::Vec3b(sample_mean[0], sample_mean[1], sample_mean[2]);
    background.std = cv::Vec3b(sample_std[0], sample_std[1], sample_std[2]);
    background.sample_size = background.data.size();
  }
  std::cout << "Mean: " << background.mean << " std: " << background.std << std::endl;
  return;
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::cout << "Error: invalid/missing input video." << std::endl;
    return -1;
  }

  cv::VideoCapture capture(argv[1]);

  if (!capture.isOpened())
  {
    std::cout << "Error: Unable to read video." << std::endl;
    return -1;
  }

  // Get params
  cv::FileStorage fs(ros::package::getPath("merlion_scripts") + "/configs/background_extract.yaml", cv::FileStorage::READ);
  // int extract_channel = (int)fs["ExtractChannel"];
  int blur_ksize = (int)fs["Blur.ksize"];
  // fs.release();

  // Process
  cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  cv::startWindowThread();

  // Background selection
  cv::setMouseCallback("image", onMouseCb);

  for (;;)
  {
    std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
    capture >> frame_src;
    if (frame_src.empty())
    {
      std::cout << "\nEnd of video." << std::endl;
      break;
    }

    // Process image
    std::vector<cv::Mat> channels;
    cv::GaussianBlur(frame_src, frame_src, cv::Size(blur_ksize, blur_ksize), 0);
    cv::cvtColor(frame_src, frame_hsv, cv::COLOR_BGR2HSV);
    cv::split(frame_hsv, channels);
    cv::equalizeHist(channels[0], channels[0]);
    cv::equalizeHist(channels[1], channels[1]);
    cv::equalizeHist(channels[2], channels[2]);
    cv::merge(channels, frame_hsv);

    cv::Mat frame_filtered;
    if(background.data.size() > 0)
    {
      cv::inRange(frame_hsv, background.mean - background.std*2, background.mean + background.std*2, frame_filtered);
      cv::cvtColor(frame_filtered, frame_filtered, cv::COLOR_GRAY2BGR);
    }
    else
    {
      frame_filtered = frame_src;
    }

    // Visualize
    cv::hconcat(frame_src, frame_filtered, frame_src);
    cv::imshow("image", frame_src);
    std::cout << "\rFrame took " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - t1).count() / 1000.0 << "ms. " << std::flush;
    if(cv::waitKey(30) == 32) // waits to display frame
    {
      // SPACE is pressed
      std::cout << "\rPaused.                " << std::flush;
      cv::waitKey(-1);
      std::cout << "\rUnpaused.              " << std::endl;
    }
  }
  cv::waitKey(0);
  return 0;
}