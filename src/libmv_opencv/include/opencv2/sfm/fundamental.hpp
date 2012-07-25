#include <opencv2/core/core.hpp>

namespace cv
{

void
projectionsFromFundamental( const Mat &F,
                            Mat &P1,
                            Mat &P2 );

void
fundamentalFromProjections( const Mat &P1,
                            const Mat &P2,
                            Mat &F );

/**
 * The normalized 8-point fundamental matrix solver.
 */
void
normalizedEightPointSolver( const cv::Mat &x1,
                            const cv::Mat &x2,
                            cv::Mat &F );


} /* namespace cv */
