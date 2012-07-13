#include "test_precomp.hpp"

using namespace cv;
using namespace std;

namespace cvtest
{

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
