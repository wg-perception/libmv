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

} // namespace cvtest
