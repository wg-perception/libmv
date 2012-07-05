#include "test_precomp.hpp"

void
readtestdata(string filename, int nviews, int npts, std::vector<std::vector<cv::Point2d> > &points2d)
{
  cout << endl << "Reading: " << filename << endl;
  FileStorage fs;
  fs.open(filename, FileStorage::READ);

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
readtestdata(string filename, int nviews, std::vector<cv::Mat> &projection_matrices)
{
  cout << endl << "Reading: " << filename << endl;
  FileStorage fs;
  fs.open(filename, FileStorage::READ);
  if (!fs.isOpened())
    std::cerr << "Cannot find file: " << filename << std::endl;

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
