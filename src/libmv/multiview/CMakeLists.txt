# define the source files
SET(MULTIVIEW_SRC projection.cc
                  fundamental.cc
                  triangulation.cc
                  euclidean_resection.cc
                  conditioning.cc
                  homography.cc
                  panography.cc)

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

