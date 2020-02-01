#include <iostream>
#include "DRRgenerator.h"
#include <chrono>
using namespace std;

int main( int argc, char** argv )
{
    char* inputMhdFile = argv[1];
    char* inputRawFile = argv[2];
    char* outputCVFile = argv[3];
    char* translation_x = argv[4]; //0
    char* translation_y = argv[5]; //0
    char* translation_z = argv[6]; //0
    char* roll = argv[7]; //
    char* pitch = argv[8]; //
    char* yaw = argv[9]; //

    char* rows = argv[10]; //512
    char* cols = argv[11]; //512
    char* min_hu = argv[12]; //-1000
    char* camera_pos = argv[13]; //-1300
    char* video00 = argv[14]; //2000
    char* video11 = argv[15]; //2000
    char* video02 = argv[16]; //256
    char* video12 = argv[17]; //256
    
    DRRgenerator drrgene;

    // we only load mhd file
    
    std::string filenameCT=inputRawFile;
    std::string info_filename=inputMhdFile;

    drrgene.translation_x = atof(translation_x);
    drrgene.translation_y = atof(translation_y);
    drrgene.translation_z = atof(translation_z);
    drrgene.roll = atof(roll);
    drrgene.pitch = atof(pitch);
    drrgene.yaw = atof(yaw);
    drrgene.rows = atoi(rows);
    drrgene.cols = atoi(cols);
    drrgene.min_hu = atoi(min_hu);
    drrgene.camera_pos = atoi(camera_pos);
    drrgene.video00 = atoi(video00);
    drrgene.video11 = atoi(video11);
    drrgene.video02 = atoi(video02);
    drrgene.video12 = atoi(video12);

    const auto startct = chrono::system_clock::now();
    drrgene.init();
    drrgene.load_CT(filenameCT,info_filename);
    const auto stopct = chrono::system_clock::now();
    const auto duratct =  chrono::duration_cast<chrono::milliseconds>(stopct - startct).count();
    std::cout<<"CT loaded in "<< duratct <<" ms"<<std::endl;

    cv::Mat color;
    const auto start = chrono::system_clock::now();
    drrgene.raytracegpu(color);
    const auto stop = chrono::system_clock::now();
    const auto durat =  chrono::duration_cast<chrono::milliseconds>(stop - start).count();
    std::cout<<"DRR created in "<< durat <<" ms"<<std::endl;

    cv::imwrite(outputCVFile,color);

    return 0;
}

