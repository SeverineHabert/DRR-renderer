#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <ctime>
#include <opencv2/core/core.hpp>
#include <omp.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#pragma once

using namespace Eigen;

struct CameraDataGPU
{
  cv::Mat intrinsics_video;
  cv::Mat extrinsics_video;
  cv::Mat inv_extrinsics_video;
  cv::Mat color, depth;
  CameraDataGPU(){}
  CameraDataGPU(double virtual_distance)
  {
    extrinsics_video=cv::Mat::eye(4,4,CV_64F);
    extrinsics_video.at<double>(2,3)=virtual_distance;
  }

  cv::Mat masktool;
};
