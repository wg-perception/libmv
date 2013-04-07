# define the source files
SET(MULTIVIEW_SRC affine.cc
                  affine_kernel.cc
                  autocalibration.cc
                  bundle.cc
                  conditioning.cc
                  essential_kernel.cc
                  euclidean.cc
                  euclidean_kernel.cc
                  euclidean_resection.cc
                  five_point.cc
                  focal_from_fundamental.cc
                  fundamental.cc
                  fundamental_kernel.cc
                  homography.cc
                  homography_kernel.cc
                  panography.cc
                  panography_kernel.cc
                  projection.cc
                  robust_affine.cc
                  robust_estimation.cc
                  robust_euclidean.cc
                  robust_euclidean_resection.cc
                  robust_fundamental.cc
                  robust_homography.cc
                  robust_resection.cc
                  robust_similarity.cc
                  similarity.cc
                  similarity_kernel.cc
                  sixpointnview.cc
                  structure.cc
                  triangulation.cc
                  twoviewtriangulation.cc)

# define the header files (make the headers appear in IDEs.)
FILE(GLOB MULTIVIEW_HDRS *.h)

ADD_LIBRARY(multiview ${MULTIVIEW_SRC} ${MULTIVIEW_HDRS})

TARGET_LINK_LIBRARIES(multiview glog numeric )

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES(multiview PROPERTIES DEBUG_POSTFIX "_d")

# installation rules for the library
LIBMV_INSTALL_LIB(multiview)

IF (BUILD_TESTS)
  ADD_LIBRARY(multiview_test_data test_data_sets.cc)
  TARGET_LINK_LIBRARIES(multiview_test_data multiview)
  # make the name of debug libraries end in _d.
  SET_TARGET_PROPERTIES(multiview_test_data PROPERTIES DEBUG_POSTFIX "_d")
ENDIF (BUILD_TESTS)

MACRO (MULTIVIEW_TEST NAME)
  LIBMV_TEST(${NAME} "multiview_test_data;multiview;numeric")
ENDMACRO (MULTIVIEW_TEST)

