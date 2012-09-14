#include <opencv2/sfm/sfm.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

#include "recon2v.hpp"

using namespace std;
using namespace cv;

static void help()
{
    cout
    << "\n------------------------------------------------------------------\n"
    << " This program shows the two view reconstruction capabilities the \n"
    << " OpenCV Structure From Motion (SFM) module.\n"
    << " It uses the following data from the VGG datasets at ...\n"
    << " Usage:\n"
    << "       reconv2 view1_2D_pts.txt view2_2D_pts.txt\n"
    << "------------------------------------------------------------------\n\n"
    << endl;
}


int main( int, char** )
{
    help();
    
    // Do projective reconstruction
    bool is_projective = true;
    
    // Assume noise free
    bool has_outliers = false;
    
    // Read 2D points from text files
    vector<Mat_<double> > points2d;
//     
//     Mat_<double> points3d_estimated;
//     
//     vector<Mat> Ps_estimated;
//     
//     reconstruct( points2d, Ps_estimated, points3d_estimated, is_projective, has_outliers );
//     
    return 0;
}
