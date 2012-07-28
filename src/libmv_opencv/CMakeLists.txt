LIBMV_INSTALL_ALL_HEADERS()


find_package(OpenCV REQUIRED)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(include)

add_library(opencv_sfm src/conditioning.cpp
                       src/fundamental.cpp    
                       src/skew.cpp
                       src/numeric.cpp
                       src/projection.cpp
                       src/reconstruct.cpp
                       src/triangulate.cpp
)

target_link_libraries(opencv_sfm  ${OpenCV_LIBRARIES}
                                  reconstruction
                                  correspondence
                                  camera
                                  multiview
                                  multiview_test_data
                                  numeric
)

install(TARGETS opencv_sfm
        DESTINATION lib
)

install(DIRECTORY include/opencv2/sfm
        DESTINATION include/opencv2
        COMPONENT main
)

# Add some tests
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(sfm_tests test/scene.cpp
                         test/test_conditioning.cpp
                         test/test_fundamental.cpp
                         test/test_main.cpp
                         test/test_precomp.cpp
                         test/test_numeric.cpp
                         test/test_projection.cpp
                         test/test_reconstruct.cpp
                         test/test_triangulate.cpp
)

find_library(OPENCV_TS_LIBRARY opencv_ts ${OpenCV_LIB_DIR})
target_link_libraries(sfm_tests opencv_sfm
                                ${GTEST_BOTH_LIBRARIES}
                                ${OpenCV_LIBRARIES}
                                ${OPENCV_TS_LIBRARY}
)

set_target_properties(sfm_tests PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/tests)

add_test(sfm_tests  ${PROJECT_BINARY_DIR}/bin/tests/sfm_tests)
