#include <iostream>
#include "DRRgenerator.h"
#include <chrono>

int main()
{
  DRRgenerator drrgene;
  // we only load mhd file
  std::string CT_folder="_folder_name_here_";
  std::string filenameCT=CT_folder+"_filenamehere_.raw";
  std::string info_filename=CT_folder+"_filenamehere_.mhd";

  const auto startct = chrono::system_clock::now();
  drrgene.load_CT(filenameCT,info_filename);
  const auto stopct = chrono::system_clock::now();
  const auto duratct =  chrono::duration_cast<chrono::milliseconds>(stopct - startct).count();

  std::cout<<"CT loaded in "<< duratct <<" ms"<<std::endl;

  cv::Mat color;

  const auto start = chrono::system_clock::now();
  drrgene.raytracegpu(color);
  const auto stop = chrono::system_clock::now();
  const auto durat = chrono::duration_cast<chrono::milliseconds>(stop - start).count();
  std::cout<<"DRR created in "<< durat <<" ms"<<std::endl;
  cv::imshow("DRR", color);
  cv::waitKey(0);

  return 0;
}

