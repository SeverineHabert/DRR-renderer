#include <iostream>
#include "DRRgenerator.h"
#include <chrono>
using namespace std;

int main()
{
    DRRgenerator drrgene;

    std::string CT_folder="/home/roberto/CampVis/LWS_Phantom/29595_003_1_25_20140221/";
    std::string filenameCT=CT_folder+"LWS_Phantom.raw";
    std::string info_filename=CT_folder+"LWS_Phantom.mhd";
    const auto startct = chrono::system_clock::now();
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
    cv::imshow("DRR", color);
    cv::waitKey(0);
    return 0;
}

