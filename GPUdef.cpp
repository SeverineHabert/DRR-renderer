#include "GPUdef.h"

CameraDataGPU::CameraDataGPU(){

}
CameraDataGPU::CameraDataGPU(double virtual_distance){
    extrinsics_video=cv::Mat::eye(4,4,CV_64F);
    extrinsics_video.at<double>(2,3)=virtual_distance;
}
