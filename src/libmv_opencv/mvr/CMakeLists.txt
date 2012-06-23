# Require OpenCV
FIND_PACKAGE( OpenCV REQUIRED )

# define the source files
SET(MVR_SRC mvr.cc)

# define the header files (make the headers appear in IDEs.)
FILE(GLOB MVR_HDRS *.h)

ADD_LIBRARY(mvr ${MVR_SRC} ${MVR_HDRS})

#TARGET_LINK_LIBRARIES(mvr correspondence multiview glog numeric V3D ${OpenCV_LIBS})
TARGET_LINK_LIBRARIES(mvr reconstruction camera correspondence multiview numeric V3D colamd ldl glog ${OpenCV_LIBS})


# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES(mvr PROPERTIES DEBUG_POSTFIX "_d")

# installation rules for the library
LIBMV_INSTALL_LIB(mvr)

IF (BUILD_TESTS)
#   ADD_LIBRARY(multiview_test_data test_data_sets.cc)
#   TARGET_LINK_LIBRARIES(multiview_test_data multiview)
  # make the name of debug libraries end in _d.
#   SET_TARGET_PROPERTIES(multiview_test_data PROPERTIES DEBUG_POSTFIX "_d")
ENDIF (BUILD_TESTS)

MACRO (MVR_TEST NAME)
  LIBMV_TEST(${NAME} "mvr;multiview_test_data;multiview;numeric")
ENDMACRO (MVR_TEST)

MVR_TEST(mvr)

# TODO(keir): Make tests that depend on generated.cc to use generated sources.
#ADD_GENERATED_SOURCE(generated.cc generator.py)
