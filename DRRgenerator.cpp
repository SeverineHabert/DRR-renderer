#include "DRRgenerator.h"
#include <algorithm>
#include <iterator>
#include <sstream>

DRRgenerator::DRRgenerator()
{
    cam=CameraDataGPU();

    cv::Mat translation=cv::Mat::zeros(3,1,CV_64F);
    translation.at<double>(0)=0;
    translation.at<double>(1)=100;
    translation.at<double>(2)=0;

    cv::Mat transfotranslation=cv::Mat::eye(4,4,CV_64F);
    for(int i=0;i<3;i++)
        transfotranslation.at<double>(i,3)=translation.at<double>(i);

    //cv::Mat transfo=cv::Mat::eye(4,4,CV_64F);
    //    cv::Mat transfo=cv::Mat::zeros(4,4,CV_64F);
    //    transfo.at<double>(0,1)=-1;
    //    transfo.at<double>(1,2)=1;
    //    transfo.at<double>(2,0)=-1;
    //    transfo.at<double>(3,3)=1;

    float roll=-M_PI/2;
    float pitch=M_PI/4;
    float yaw=M_PI/8;

    cv::Mat rotz=cv::Mat::eye(4,4,CV_64F);
    cv::Mat roty=cv::Mat::eye(4,4,CV_64F);
    cv::Mat rotx=cv::Mat::eye(4,4,CV_64F);

    rotz.at<double>(0,0)=cos(yaw);
    rotz.at<double>(0,1)=-sin(yaw);
    rotz.at<double>(1,0)=sin(yaw);
    rotz.at<double>(1,1)=cos(yaw);

    roty.at<double>(0,0)=cos(pitch);
    roty.at<double>(0,2)=sin(pitch);
    roty.at<double>(2,0)=-sin(pitch);
    roty.at<double>(2,2)=cos(pitch);

    rotx.at<double>(1,1)=cos(roll);
    rotx.at<double>(1,2)=-sin(roll);
    rotx.at<double>(2,1)=sin(roll);
    rotx.at<double>(2,2)=cos(roll);

    cv::Mat rot=rotz*roty*rotx;

    //    cv::Mat transfo=cv::Mat::zeros(4,4,CV_64F); // new coordinate of the base
    //    transfo.at<double>(0,2)=-1;
    //    transfo.at<double>(1,1)=1;
    //    transfo.at<double>(2,0)=1;
    //    transfo.at<double>(3,3)=1;

    cv::Mat transfo2=cv::Mat::eye(4,4,CV_64F);
    transfo2.at<double>(2,3)=-900;

    cam.extrinsics_video=transfotranslation*rot*transfo2;

    cam.intrinsics_video=cv::Mat::eye(3,3,CV_64F);
    cam.intrinsics_video.at<double>(0,0)=2500;
    cam.intrinsics_video.at<double>(1,1)=2500;
    cam.intrinsics_video.at<double>(0,2)=320;
    cam.intrinsics_video.at<double>(1,2)=240;
}

float DRRgenerator::attenuation_lookup_hu (float pix_density)
{
    const double min_hu = -800.0;
    const double mu_h2o = 0.022;
    if (pix_density <= min_hu) {
        return 0.0;
    } else {
        return (pix_density/1000.0) * mu_h2o + mu_h2o;
    }
}

float DRRgenerator::attenuation_lookup (float pix_density)
{
    return attenuation_lookup_hu (pix_density);
}


void DRRgenerator::load_CT(std::string filename_raw,std::string info){
    double vsx=0;
    double vsy=0;
    double vsz=0;
    int imgx=0;
    int imgy=0;
    int imgz=0; // change the info from mhd file
    std::ifstream  fin(info);
    std::string    file_line;
    int c=0;
    float * offset= new float [3];
    while(std::getline(fin, file_line))
    {
        c++;

        if (file_line.find("DimSize =")!= std::string::npos){

            std::size_t pos = file_line.find("=");
            string dims=file_line.substr (pos+1);
            vector<string> elems;
            split(dims, elems);
            imgx=atoi(elems[0].c_str());
            imgy=atoi(elems[1].c_str());
            imgz=atoi(elems[2].c_str());

        }
        if (file_line.find("ElementSpacing =")!= std::string::npos){
            std::size_t pos = file_line.find("=");
            string spacings=file_line.substr (pos+1);
            vector<string> elems;
            split(spacings, elems);
            vsx=atof(elems[0].c_str());
            vsy=atof(elems[1].c_str());
            vsz=atof(elems[2].c_str());
        }

        if (file_line.find("Position =")!= std::string::npos){
            std::size_t pos = file_line.find("=");
            string spacings=file_line.substr (pos+1);
            vector<string> elems;
            split(spacings, elems);
            offset[0]=atof(elems[0].c_str());
            offset[1]=atof(elems[1].c_str());
            offset[2]=atof(elems[2].c_str());
        }

    }
    std::cout<<c<<" "<<vsx<< " "<<vsy<< " "<<vsz<< " "<<imgx<< " "<<imgy<< " "<<imgz<< " "<<offset[0]<< " "<<offset[1]<< " "<<offset[2]<<std::endl;

    CTvolume<short> CTvol_or=CTvolume<short>(imgx, imgy, imgz, vsx,vsy,vsz, 1);
    CTvol_or.offset=offset;
    auto file = fopen(filename_raw.data(), "rb");
    size_t res=fread(CTvol_or.data, sizeof(CTvol_or.data[0]),imgx*imgy*imgz, file);

    CTvol=CTvolume<double>(imgx, imgy, imgz, vsx,vsy,vsz, 1);
    for (int k=0;k<imgz;++k)
        for (int i=0;i<imgx;++i)
            for (int j=0;j<imgy;++j){
                CTvol.at(i,j,k)=attenuation_lookup(CTvol_or.at(i,j,k)-1024)   ;
                //std::cout<<CTvol.at(i,j,k)<<std::endl;

            }
    fclose (file);

}

void DRRgenerator::split(const string &s, vector<string> &elems) {
    std::vector<std::string> vec;

    istringstream iss(s);
    copy(istream_iterator<string>(iss),
         istream_iterator<string>(),
         back_inserter(vec));
    elems=vec;
}

void  DRRgenerator::raytracegpu(cv::Mat &color)
{

    // size of your final DRR (transposed)
    int rows=640;
    int cols=480;

    color =cv::Mat::zeros(rows, cols,CV_8UC1);
    cv::Mat color_raw =cv::Mat::zeros(rows, cols,CV_64F);

    // size of the global CT/2
    float sx = CTvol.size_x*CTvol.voxel_size_x/2;
    float sy = CTvol.size_y*CTvol.voxel_size_y/2;
    float sz = CTvol.size_z*CTvol.voxel_size_z/2;
    float vs_inv_x = 1.0f / CTvol.voxel_size_x;
    float vs_inv_y = 1.0f / CTvol.voxel_size_y;
    float vs_inv_z = 1.0f / CTvol.voxel_size_z;

    // import the pose of the camera and get the rotation and translation
    Isometry3f transfo=cv2eigeniso(cam.extrinsics_video);
    Matrix3f rot = transfo.linear();
    Vector3f startPos = transfo.translation();

    // step for the raytracing ( we move of the smallest size of voxel here)
    float pas=1*CTvol.voxel_size_x;

    // find when we start to enter the CT ( to avoid useless iterations)
    int min_it=(int)((abs(startPos.norm())-sz)/pas);
    // find when we exit the CT ( to avoid useless iterations)
    int max_dist=(int)((sqrt(3)*2*sz)/pas);
    cout<<max_dist<<" "<<min_it<<endl;

    //import the intrinsics
    cv::Mat  intrin_xray=cam.intrinsics_video;
    double fx=intrin_xray.at<double>(0,0);
    double fy=intrin_xray.at<double>(1,1);
    double ux=intrin_xray.at<double>(0,2);
    double uy=intrin_xray.at<double>(1,2);
    //cout<<"intri"<<intrin_xray<<endl;


#pragma omp parallel for
    for(int r=0; r < color.rows;++r)
        for(int c=0; c < color.cols;++c)
        {
            // we create a ray from the pixel where we want to create the DRR
            Vector3f ray((r-ux)/fx,(c- uy)/ fy,1.0f);
            //we bring it to the CT coordinate system
            ray = (rot*ray)*pas;
            // we calculate the starting position
            Vector3f curr = startPos+min_it*ray;
            //cout<<"ray"<<ray<< " start point"<<startPos<<" cur"<<curr<<endl;
            float u_curr=0;
            float sum=0;
            int count=0;
            for(int i=0; i < max_dist;++i)
            {

                // Travel along ray and compute voxel space coordinates
                curr += ray;
                // Check to see if we are out of the CT
                if(curr[0] <= -sx || curr[0] >= sx) continue;
                if(curr[1] <= -sy || curr[1] >= sy) continue;
                if(curr[2] <= -sz || curr[2] >= sz) continue;
                // If inside the CT, we convert our coordinates to the CT coordinates
                int x = (curr[0] + sx) * vs_inv_x;
                int y = (curr[1] + sy) * vs_inv_y;
                int z = (curr[2] + sz) * vs_inv_z;
                 // we get the value of the CT and sum it
                u_curr= (float)CTvol.at(x,y,z);
                sum+=u_curr;
                count++; // we count also the number of voxel crossed for avergaing later

            }
            if (count>0){
                double val=sum/count; // we avergae here
                color_raw.at<double>(r,c) = val;
            }
        }
    // the value inside color_raw are actually very small, so we scale it in between 0 and 255
    double min;
    double max;
    cv::minMaxIdx(color_raw, &min, &max);
    cout<<"max"<<max<<endl;
    float scale = 255 / (max-min);
    color_raw.convertTo(color,CV_8UC1, scale, -min*scale);
    // image inversion to get dark values for dense structures
    bitwise_not ( color, color );

    // Rotate images by 90 degrees because our y/z in 3D is flipped
    cv::transpose(color,color);

}


Eigen::Isometry3f   DRRgenerator::cv2eigeniso(cv::Mat transfo){
    Eigen::Isometry3f  mat;
    Eigen::Matrix4f  mat4f;
    for (size_t i=0;i<4;i++)
        for (size_t j=0;j<4;j++)
            mat4f(i,j)=transfo.at<double>(i,j);
    mat.matrix()=mat4f;
    return mat;
}

