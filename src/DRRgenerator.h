#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <omp.h>
#include <math.h>

#pragma once

#define _USE_MATH_DEFINES

using namespace Eigen;

struct CameraDataGPU
{
    cv::Mat intrinsics_video;
    cv::Mat extrinsics_video;
    CameraDataGPU(){}
};

template <typename T>
class CTvolume
{
public:
  int size_x, size_y, size_z;
  float voxel_size_x,voxel_size_y,voxel_size_z;
  T default_value;
  T* data ;
  float * offset;
  int typevalue;
  
  CTvolume(){}

  CTvolume(int sx, int sy, int sz, double voxel_sx,double voxel_sy,double voxel_sz, T const default_v) :
    size_x(sx), size_y(sy), size_z(sz), voxel_size_x(voxel_sx), voxel_size_y(voxel_sy),voxel_size_z(voxel_sz),default_value(default_v) {
    data= new T [sx * sy * sz];
    std::fill_n(data, sx * sy * sz, default_v);
    offset= new float [3];
    std::fill_n(offset, 3, 0);
    typevalue=1;
  }

  T& at(int x, int y, int z)
  {
    return data[x + size_x * y + size_x * size_y * z];
  }

  T at(int x, int y, int z) const
  {
    return data[x + size_x * y + size_x * size_y * z];
  }
};

class DRRgenerator
{
public:
  DRRgenerator();

  void load_CT(const std::string& filename_raw, const std::string& info);

  Eigen::Isometry3f cv2eigeniso(cv::Mat transfo);

  void  raytracegpu(cv::Mat& color);

  float trilinear_interpolation(short* a, cv::Point3f  pt);
  float attenuation_lookup_hu(float pix_density);
  float attenuation_lookup (float pix_density);

  void findentryandexitpoint(Vector3f startpoint, Vector3f ray, double& texitpoint, double& tentrypoint);


private:
  void split(const std::string& s, std::vector<std::string>& elems);
  
private:
  CTvolume<short> CTvol;
  CameraDataGPU cam;
};
