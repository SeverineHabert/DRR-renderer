#include "DRRgenerator.h"
#include <algorithm>
#include <iterator>
#include <sstream>


/* in the constructor, we define the camera parameter
 * it should be changed according to your needs
 *it supposed that the CT coordinate system is centered on the CT center
 */

DRRgenerator::DRRgenerator()
{
    cam=CameraDataGPU();

    // translation compared to the center of CT
    cv::Mat translation=cv::Mat::zeros(3,1,CV_64F);
    //    translation.at<double>(0)=0;
    //    translation.at<double>(1)=0;
    //    translation.at<double>(2)=0;

    cv::Mat transfotranslation=cv::Mat::eye(4,4,CV_64F);
    for(int i=0;i<3;i++)
        transfotranslation.at<double>(i,3)=translation.at<double>(i);

    float roll=-M_PI/2; // rotation around x-axis
    float pitch=M_PI/2; // rotation around y-axis
    float yaw=-M_PI/2; // rotation around z-axis

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

    // distance of the camera from the center of the CT coordinate system (after translation)
    cv::Mat transfo2=cv::Mat::eye(4,4,CV_64F);
    transfo2.at<double>(2,3)=-1000;

    cam.extrinsics_video=transfotranslation*rot*transfo2;

    // intrinsic parameters of the X-ray source
    cam.intrinsics_video=cv::Mat::eye(3,3,CV_64F);
    cam.intrinsics_video.at<double>(0,0)=2200;
    cam.intrinsics_video.at<double>(1,1)=2200;
    cam.intrinsics_video.at<double>(0,2)=320;
    cam.intrinsics_video.at<double>(1,2)=240;
}

/* normalization of the Hounsfield values ( taken from Plastimatch project of Harvard University)
 *which says:
 * According to NIST, the mass attenuation coefficient of H2O at 50 keV
is 0.22 cm^2 per gram.  Thus, we scale by 0.022 per mm
http://physics.nist.gov/PhysRefData/XrayMassCoef/ComTab/water.html
 */
float DRRgenerator::attenuation_lookup_hu (float pix_density)
{

    double min_hu = -800.0; // this is a threshold on the density, if you want to consider less dense matter in the DRR, decrease this value to -1000.
    if (CTvol.typevalue==0)
        min_hu = 100;
    double mu_h2o = 0.022;
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

/* this function load a mhd + raw file
 * we load the information of translation, but does not use it in the rest of code
 */

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

    int type=2;

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
        if ((file_line.find("ElementSpacing =")!= std::string::npos)||(file_line.find("ElementSize =")!= std::string::npos)){
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


        if (file_line.find("ElementType =")!= std::string::npos){
            std::size_t pos = file_line.find("=");
            std::string typestring=file_line.substr (pos+1);
            std::cout<<typestring<<std::endl;
            if (typestring.compare(5,5,"UCHAR")==0) type=0;
            if (typestring.compare(5,5,"SHORT")==0) type=1;
        }

    }
    std::cout<<c<<" "<<vsx<< " "<<vsy<< " "<<vsz<< " "<<imgx<< " "<<imgy<< " "<<imgz<<" "<<type<<std::endl;

    CTvolume<short> CTvol_or=CTvolume<short>(imgx, imgy, imgz, vsx,vsy,vsz, 1);
    CTvol_or.offset=offset;
    CTvol_or.typevalue=type;
    auto file = fopen(filename_raw.data(), "rb");
    if (type==0){
        CTvolume<uchar> CTvol_un=CTvolume<uchar>(imgx, imgy, imgz, vsx,vsy,vsz, 0);
        size_t res=fread(CTvol_un.data, sizeof(uchar),imgx*imgy*imgz, file);
        for (int i=0;i<imgx;i++)
            for (int j=0;j<imgy;j++)
                for (int k=0;k<imgz;k++)
                    CTvol_or.at(i,j,k)=(short)CTvol_un.at(i,j,k);
    }
    else if (type==1)
        size_t res=fread(CTvol_or.data, sizeof(short),imgx*imgy*imgz, file);
    std::cout<<"ct"<<CTvol_or.data[0]<<std::endl;

    std::cout<< "max"<<*std::max_element(CTvol_or.data,CTvol_or.data+imgx*imgy*imgz-1)<<std::endl;
    CTvol=CTvol_or;
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

/* we  average the CT values along the ray created from the pixel of our final DRR
 *
 */

void  DRRgenerator::raytracegpu(cv::Mat &color)
{

    // size of the final DRR (transposed)
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

    //float max_size=max(sy,sz);
    float min_voxelsize=min(CTvol.voxel_size_x,CTvol.voxel_size_z);

    // import the pose of the camera and get the rotation and translation
    Isometry3f transfo=cv2eigeniso(cam.extrinsics_video);
    Matrix3f rot = transfo.linear();
    Vector3f startPos = transfo.translation();

    // step for the raytracing ( we move of the smallest size of voxel here)
    float pas=1*min_voxelsize;

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

            float u_curr=0;
            float sum=0;
            int count=0;
            double texitpoint;
            double tentrypoint;
            // we search for the entry point and exit point to avoid useless iterations
            findentryandexitpoint(startPos, ray, texitpoint, tentrypoint);
            int min_it=tentrypoint;
            int max_dist=ceil(texitpoint)-min_it;

            //cout<<"it"<<max_dist<<" "<<min_it<<endl;
            // we calculate the starting position
            Vector3f curr = startPos+min_it*ray;
            for(int i=0; i < max_dist;++i)
            {

                // Travel along ray and compute voxel space coordinates
                curr += ray;

                // Check to see if we are out of the CT
                if(curr[0] <= -sx || curr[0] >= sx-2) continue;
                if(curr[1] <= -sy || curr[1] >= sy-2) continue;
                if(curr[2] <= -sz || curr[2] >= sz-2) continue;

                // If inside the CT, we convert our coordinates to the CT coordinates
                double x = (curr[0] + sx) * vs_inv_x;
                double y = (curr[1] + sy) * vs_inv_y;
                double z = (curr[2] + sz) * vs_inv_z;

                // we get the value of the CT and sum it
                u_curr= trilinear_interpolation(CTvol.data,cv::Point3f(x,y,z));
                sum+=u_curr;
                count++; // we count also the number of voxel crossed for avergaing later

            }
            if (count>0){
                double val=sum/count; // we average over the number of passed voxels
                color_raw.at<double>(r,c) = val;

            }

        }
    // the value inside color_raw are actually very small, so we scale it in between 0 and 255
    double min;
    double max;
    cv::minMaxIdx(color_raw, &min, &max);
    cout<<"max"<<max<<endl;
    cout<<"min"<<min<<endl;
    float scale = 255 / (max-min);
    color_raw.convertTo(color,CV_8UC1, scale, -min*scale);
    // image inversion to get dark values for dense structures
    bitwise_not ( color, color );

    // Rotate images by 90 degrees because our y/z in 3D is flipped
    //cv::transpose(color,color);

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
/* when requesting the CT value at one point, we perform trilinear interpolation around the 8 corners of the voxel around this point.
 * Gives a smoother result
*/

float  DRRgenerator::trilinear_interpolation(short *a,cv::Point3f  pt){

    int dx=CTvol.size_x;
    int dy=CTvol.size_y;

    cv::Point3i p000= cv::Point3i(floor(pt.x),floor(pt.y),floor(pt.z));
    cv::Point3i p111=cv::Point3i(p000.x+1,p000.y+1,p000.z+1);
    cv::Point3i p011=cv::Point3i(p000.x,p000.y+1,p000.z+1);
    cv::Point3i p001=cv::Point3i(p000.x,p000.y,p000.z+1);
    cv::Point3i p101=cv::Point3i(p000.x+1,p000.y,p000.z+1);
    cv::Point3i p100=cv::Point3i(p000.x+1,p000.y,p000.z);
    cv::Point3i p110=cv::Point3i(p000.x+1,p000.y+1,p000.z);
    cv::Point3i p010=cv::Point3i(p000.x,p000.y+1,p000.z);


    float u000= attenuation_lookup(a[p000.x + dx*p000.y + dx*dy*p000.z]-1024)   ;
    float u100= attenuation_lookup(a[p100.x + dx*p100.y + dx*dy*p100.z]-1024) ;
    float u010= attenuation_lookup(a[p010.x + dx*p010.y + dx*dy*p010.z]-1024) ;
    float u101= attenuation_lookup(a[p101.x + dx*p101.y + dx*dy*p101.z]-1024) ;
    float u001= attenuation_lookup(a[p001.x + dx*p001.y + dx*dy*p001.z]-1024) ;
    float u110=attenuation_lookup( a[p110.x + dx*p110.y + dx*dy*p110.z]-1024) ;
    float u011=attenuation_lookup( a[p011.x + dx*p011.y + dx*dy*p011.z]-1024) ;
    float u111= attenuation_lookup(a[p111.x + dx*p111.y + dx*dy*p111.z]-1024) ;

    float xd=pt.x-p000.x;
    float yd=pt.y-p000.y;
    float zd=pt.z-p000.z;

    float c00=u000*(1-xd)+u100*xd;
    float c01=u001*(1-xd)+u101*xd;
    float c10=u010*(1-xd)+u110*xd;
    float c11=u011*(1-xd)+u111*xd;

    float c0=c00*(1-yd)+c10*yd;
    float c1=c01*(1-yd)+c11*yd;

    float c=c0*(1-zd)+c1*zd;

    return c;


}
/* find the ray entry point into the volume and the ray exit point
 * it looks at the intersection of the ray with the 6 planes making the outter surface of the CT box
 * only 2 planes will intersect the ray in the limit of the CT dimension
 */

void DRRgenerator::findentryandexitpoint(Vector3f startpoint, Vector3f ray, double &texitpoint, double &tentrypoint){

    float sx = CTvol.size_x*CTvol.voxel_size_x/2;
    float sy = CTvol.size_y*CTvol.voxel_size_y/2;
    float sz = CTvol.size_z*CTvol.voxel_size_z/2;

    Vector3f vxp=Vector3f(sx,0,0)-startpoint;
    Vector3f vxn=Vector3f(-sx,0,0)-startpoint;
    Vector3f vyp=Vector3f(0,sy,0)-startpoint;
    Vector3f vyn=Vector3f(0,-sy,0)-startpoint;
    Vector3f vzp=Vector3f(0,0,sz)-startpoint;
    Vector3f vzn=Vector3f(0,0,-sz)-startpoint;


    Vector3f nx=Vector3f(1,0,0);
    Vector3f ny=Vector3f(0,1,0);
    Vector3f nz=Vector3f(0,0,1);
    if (vxp[0]<0)
        nx=-nx;
    if (vyp[1]<0)
        ny=-ny;
    if (vzp[2]<0)
        nz=-nz;

    double txp,txn,typ,tyn, tzp,tzn=-1;
    double tval[6]={-1, -1,-1,-1,-1,-1};
    if (nx.dot(ray)!=0) {
        txp=nx.dot(vxp)/nx.dot(ray);
        Vector3f ptxp=startpoint+txp*ray;
        if ( (ptxp[1] > -sy) & (ptxp[1] < sy) & (ptxp[2]> -sz) & (ptxp[2] <sz))
            tval[0]=txp;

    }
    if (nx.dot(ray)!=0){
        txn=nx.dot(vxn)/nx.dot(ray);
        Vector3f ptxn=startpoint+txn*ray;
        if ( ptxn[1] > -sy & ptxn[1] < sy & ptxn[2]> -sz & ptxn[2] <sz)
            tval[1]=txn;
    }
    if (ny.dot(ray)!=0){
        typ=ny.dot(vyp)/ny.dot(ray);
        Vector3f ptyp=startpoint+typ*ray;
        if ( ptyp[0] > -sx & ptyp[0] < sx & ptyp[2]> -sz & ptyp[2] <sz)
            tval[2]=typ;
    }
    if (ny.dot(ray)!=0){
        tyn=ny.dot(vyn)/ny.dot(ray);
        Vector3f ptyn=startpoint+tyn*ray;
        if ( ptyn[0] > -sx & ptyn[0] < sx & ptyn[2]> -sz & ptyn[2] <sz)
            tval[3]=tyn;
    }
    if (nz.dot(ray)!=0){
        tzp=nz.dot(vzp)/nz.dot(ray);
        Vector3f ptzp=startpoint+tzp*ray;
        if ( ptzp[0] > -sx & ptzp[0] < sx & ptzp[1]> -sy & ptzp[1] <sy)
            tval[4]=tzp;
    }
    if (nz.dot(ray)!=0){
        tzn=nz.dot(vzn)/nz.dot(ray);
        Vector3f ptzn=startpoint+tzn*ray;
        if ( ptzn[0] > -sx & ptzn[0] < sx & ptzn[1]> -sy & ptzn[1] <sy)
            tval[5]=tzn;
    }
    //printf(" t value %f %f %f %f %f %f \n", tval[0],tval[1],tval[2], tval[3],tval[4],tval[5]);
    double tmin=0;
    double tmax=0;
    int c=0;
    while (tval[c]<0){
        c++;

    }
    if (c<4){
        tmin =tval[c];
        c++;
    }
    while (tval[c]<0){
        c++;

    }
    if (c<5)
        tmax =tval[c];
    //printf(" tmin: %f , tmax %f , \n",tmin,tmax);
    if (tmax<tmin){
        double temp=tmin;
        tmin=tmax;
        tmax=temp;

    }
    tentrypoint=tmin; // thr lowest value means it is closer to the starting point, therefore it is the entry point
    texitpoint=tmax;

}



