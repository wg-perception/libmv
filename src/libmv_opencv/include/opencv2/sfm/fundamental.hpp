#include <opencv2/core/core.hpp>

namespace cv
{

void
projectionsFromFundamental( const Mat &F,
                            Mat &P1,
                            Mat &P2 );

/**
 * The normalized 8-point fundamental matrix solver.
 */
void
normalizedEightPointSolver( const cv::Mat &x1,
                            const cv::Mat &x2,
                            cv::Mat &F );


} /* namespace cv */
