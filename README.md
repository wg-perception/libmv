# GSoC 2012 - SfM: adapt libmv for OpenCV

This is a collaborative effort to port libmv to OpenCV. [libmv](http://code.google.com/p/libmv/) is a Structure from Motion (SfM) library, which is divided into different modules (image/detector/descriptor/multiview) that allow to resolve part of the SfM process. We focus on the muliview functionalities, trying to port them to OpenCV.

Most of the our code is going to be in the folder: ./src/libmv_opencv/

## Github repository
  https://github.com/wg-perception/libmv

## Compile
To compile CMake is used: http://www.cmake.org

Prerequisite
- OpenCV: http://opencv.org/
- Qt4 (optional): http://qt.nokia.com
- Sphinx (optional): http://sphinx.pocoo.org/


### How to compile

On Linux,
    mkdir bin-opt
    cd bin-opt
    cmake ../src/
    make


An alternative way is using the top-level Makefile, just doing:
    make

For debug,
    make debug


## Documentation
    make documentation
    cd bin-opt/doc/
    open 'index.html' file

Documentation content:
- ToDo
- **Detailed TODO**
- Docs on what has been done
- Guidelines for coding
- Datasets to use
- **Progress**

Note: Important sections are **Detailed TODO** and **Progress**. The Progress section is duplicated in the github wiki:
   https://github.com/wg-perception/libmv/wiki/Progress


## Test cases
    make
    cd bin-opt
    ./bin/tests/sfm_tests


Note 1: the test case *Sfm_simple_pipeline.backyard* takes times, so it can be skipped using *gtest* options:

    make
    cd bin-opt
    ./bin/tests/sfm_tests --gtest_filter=-Sfm_simple_pipeline.backyard


Note 2: if you want to see the data with blender, take a look at:

    blender ./src/libmv_opencv/testdata/cv/sfm/backyard.blend


## Troubleshooting

If you have troubles compiling the project, use the development version of OpenCV:
   https://github.com/Itseez/opencv

There was a bug in OpenCV, and it was fixed here:
   https://github.com/Itseez/opencv/commit/1baf5209c4df292922f78e2757321c359f8b28e8
