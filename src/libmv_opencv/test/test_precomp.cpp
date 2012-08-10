#include "test_precomp.hpp"

#include <fstream>
#include <cstdlib>

using namespace cv;
using namespace std;

namespace cvtest
{

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


  /*  Read 2d Points*/
  void
  readtestdata(string filename, int nviews, int npts, std::vector<cv::Mat> &points2d)
  {
    FileStorage fs;
    OPEN_TESTFILE(filename, fs)

    /* Read 2d point data */
    for (int m = 0; m < nviews; ++m)
    {
      std::stringstream buf;
      buf << "W" << m + 1;
      cv::Mat pts;
      fs[buf.str()] >> pts;
      points2d.push_back(pts);
    }
  }

  /*  Read 3d Points*/
  void
  readtestdata(string filename, cv::Mat &points3d)
  {
    FileStorage fs;
    OPEN_TESTFILE(filename, fs)

    /* Read 3d point data */
    fs["S"] >> points3d;
  }

  /*Read Projection Matrices*/
  void
  readtestdata(string filename, int nviews, std::vector<cv::Mat> &projection_matrices)
  {
    FileStorage fs;
    OPEN_TESTFILE(filename, fs)

    /* Read projection matrix data */
    for (int m = 0; m < nviews; ++m)
    {
      cv::Mat P;

      std::stringstream buf;
      buf << "P" << m + 1;
      fs[buf.str()] >> P;

      projection_matrices.push_back(P);
    }
  }
#if 0
void
readtestdata(string filename, int nviews, int npts, std::vector<std::vector<cv::Point2d> > &points2d)
{
  FileStorage fs;
  OPEN_TESTFILE(filename, fs)

  /* Read 2d point data */
  std::vector<cv::Mat> W(nviews);
  for (int m = 0; m < nviews; ++m)
  {
    std::stringstream buf;
    buf << "W" << m + 1;
    fs[buf.str()] >> W[m];
  }

  /* Make 2d point data as a vector of vector of points */
  for (int m = 0; m < nviews; ++m)
  {
    std::vector<cv::Point2d> pts;
    for (int n = 0; n < npts; ++n)
    pts.push_back(cv::Point2d(W[m].at<double>(0, n), W[m].at<double>(1, n)));
    points2d.push_back(pts);
  }
}

void
readtestdata(string filename, std::vector<cv::Point3d> &points3d)
{
  FileStorage fs;
  OPEN_TESTFILE(filename, fs)

  /* Read 3d point data */

  cv::Mat S;
  fs["S"] >> S;
  int npoints = S.cols;
//    cout << "S:" << S << endl;
  for (int n = 0; n < npoints; ++n)
  {
    Point3d p;
    p.x = S.at<double>(0, n);
    p.y = S.at<double>(1, n);
    p.z = S.at<double>(2, n);
    points3d.push_back(p);
  }
}

#endif

}

/* namespace cvtest */
