#include "test_precomp.hpp"

void
readtestdata(string filename, int nviews, int npts, std::vector<std::vector<cv::Point2d> > &points2d)
{
  cout << endl << "Reading: " << filename << endl;
  FileStorage fs;
  fs.open(filename, FileStorage::READ);

  /* Read 2d point data */
  cv::Mat W[nviews];
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
    for (int n = 0; n < 10; ++n)
      pts.push_back(cv::Point2d(W[m].at<double>(0, n), W[m].at<double>(1, n)));
    points2d.push_back(pts);
  }
}
