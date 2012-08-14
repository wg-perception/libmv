#include "test_precomp.hpp"

#include <fstream>
#include <cstdlib>

using namespace cv;
using namespace std;

namespace cvtest
{

void generateTwoViewRandomScene( cvtest::TwoViewDataSet &data, int depth )
{
    vector<Mat> points2d, Rs, ts, Ps;
    Mat K, points3d;

    int nviews = 2;
    int npoints = 30;
    bool is_projective = true;

    generateScene(nviews, npoints, is_projective, depth, K, Rs, ts, Ps, points3d, points2d);

    // Internal parameters (same K)
    data.K1 = K;
    data.K2 = K;

    // Rotation
    data.R1 = Rs[0];
    data.R2 = Rs[1];

    // Translation
    data.t1 = ts[0];
    data.t2 = ts[1];

    // Projection matrix, P = K(R|t)
    data.P1 = Ps[0];
    data.P2 = Ps[1];

    // Fundamental matrix
    fundamentalFromProjections( data.P1, data.P2, data.F );

    // 3D points
    data.X = points3d;

    // Projected points
    data.x1 = points2d[0];
    data.x2 = points2d[1];
}

void
parser_2D_tracks( const string &_filename, libmv::Tracks &tracks )
{
    string filename = string(TEST_DATA_DIR) + _filename;
    ifstream file( filename.c_str() );

    double x, y;
    string str;

    for (int track = 0; getline(file, str); ++track)
    {
        istringstream line(str);
        bool is_first_time = true;

        for (int frame = 0; line >> x >> y; ++frame)
        {
            // valid marker
            if ( x > 0 && y > 0 )
            {
                tracks.Insert( frame, track, x, y );

                if ( is_first_time )
                    is_first_time = false;
            }

            // lost track
            else if ( x < 0 && y < 0 )
            {
                is_first_time = true;
            }

            // some error
            else
            {
                exit(1);
            }
        }
    }
}

} // namespace cvtest
