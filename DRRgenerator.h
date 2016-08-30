#ifndef DRRGENERATOR_H
#define DRRGENERATOR_H
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <ctime>
#include <opencv2/core/core.hpp>
#include <omp.h>
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#define _USE_MATH_DEFINES

using namespace Eigen;
using namespace std;

struct CameraDataGPU
{
    cv::Mat intrinsics_video;
    cv::Mat extrinsics_video;
    CameraDataGPU(){}
};

template <typename T>
class CTvolume {

public:

    int size_x, size_y, size_z;
    float voxel_size_x,voxel_size_y,voxel_size_z;
    T default_value;
    T* data ;
    float * offset;

    CTvolume(){}

    CTvolume(int sx, int sy, int sz, double voxel_sx,double voxel_sy,double voxel_sz, T const default_v) :
        size_x(sx), size_y(sy), size_z(sz), voxel_size_x(voxel_sx), voxel_size_y(voxel_sy),voxel_size_z(voxel_sz),default_value(default_v) {
        data= new T [sx * sy * sz];
        std::fill_n(data, sx * sy * sz, default_v);
        offset= new float [3];
        std::fill_n(offset, 3, 0);
    }

    T& at(int x, int y, int z) {return data[x + size_x * y + size_x * size_y * z];}
    T at(int x, int y, int z) const {return data[x + size_x * y + size_x * size_y * z];}

};


class DRRgenerator
{
 private:
    CTvolume<double> CTvol;
    CameraDataGPU cam;
    void split(const string &s,  vector<string> &elems);

public:
    DRRgenerator();
    void load_CT(std::string filename_raw,std::string info);
    Eigen::Isometry3f  cv2eigeniso(cv::Mat transfo);
    void  raytracegpu( cv::Mat &color);

    float attenuation_lookup_hu (float pix_density);
    float attenuation_lookup (float pix_density);
};

#endif // DRRGENERATOR_H
