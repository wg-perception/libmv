LIBMV_INSTALL_ALL_HEADERS()

find_package(OpenCV REQUIRED)


##############
#  opencv_sfm
##############

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(include)

# source files
FILE(GLOB OPENCV_SFM_SRC src/*.cpp)

# define the header files (make the headers appear in IDEs.)
FILE(GLOB OPENCV_SFM_HDRS include/opencv2/sfm/*.hpp)

add_library(opencv_sfm ${OPENCV_SFM_SRC} ${OPENCV_SFM_HDRS})


target_link_libraries(opencv_sfm  ${OpenCV_LIBRARIES}
                                  reconstruction
                                  correspondence
                                  camera
                                  multiview
                                  multiview_test_data
                                  numeric
                                  simple_pipeline
)

install(TARGETS opencv_sfm
        DESTINATION lib
)

install(DIRECTORY include/opencv2/sfm
        DESTINATION include/opencv2
        COMPONENT main
)


##############
#  sfm_tests
##############

# Add some tests
include_directories(${GTEST_INCLUDE_DIRS})

# source files
FILE(GLOB SFM_TESTS_SRC test/*.cpp)

# define the header files (make the headers appear in IDEs.)
FILE(GLOB SFM_TESTS_HDRS test/*.hpp)

add_executable(sfm_tests ${SFM_TESTS_SRC} ${SFM_TESTS_HDRS})


find_library(OPENCV_TS_LIBRARY opencv_ts ${OpenCV_LIB_DIR})
target_link_libraries(sfm_tests opencv_sfm
                                ${GTEST_BOTH_LIBRARIES}
                                ${OpenCV_LIBRARIES}
                                ${OPENCV_TS_LIBRARY}
)

set_target_properties(sfm_tests PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/tests)

add_test(sfm_tests  ${PROJECT_BINARY_DIR}/bin/tests/sfm_tests)

ADD_DEFINITIONS(-DTEST_DATA_DIR="\\"${CMAKE_CURRENT_SOURCE_DIR}/testdata/cv/sfm/\\"")
