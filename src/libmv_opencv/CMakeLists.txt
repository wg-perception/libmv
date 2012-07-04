ADD_SUBDIRECTORY(mvr)

LIBMV_INSTALL_ALL_HEADERS()


find_package(OpenCV REQUIRED)

include_directories(include)

add_library(opencv_sfm src/reconstruct.cpp
                       src/triangulate.cpp
                       src/projection.cpp
)

target_link_libraries(opencv_sfm ${OpenCV_LIBS}
                                  ${OpenCV_LIBRARIES}
                                  reconstruction
                                  correspondence
                                  camera
                                  multiview 
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

add_executable(sfm_tests test/test_main.cpp
                         test/test_reconstruct.cpp
                         test/test_projection.cpp

)

find_library(OPENCV_TS_LIBRARY opencv_ts ${OpenCV_LIB_DIR})
target_link_libraries(sfm_tests  ${GTEST_BOTH_LIBRARIES}
				  opencv_sfm            
# This gives error:  there is a cycle in the constraint graph:			  
#                                  ${OPENCV_TS_LIBRARY}
)

set_target_properties(sfm_tests PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/tests)

add_test(sfm_tests sfm_tests)
