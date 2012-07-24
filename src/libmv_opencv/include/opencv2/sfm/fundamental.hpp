#include <opencv2/core/core.hpp>

namespace cv
{

/**
 * The normalized 8-point fundamental matrix solver.
 */
void
normalizedEightPointSolver( const cv::Mat &x1,
                            const cv::Mat &x2,
                            cv::Mat &F );


} /* namespace cv */
