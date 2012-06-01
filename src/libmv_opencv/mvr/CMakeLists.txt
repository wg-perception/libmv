# define the source files
SET(MVR_SCR mvr.cc)

# define the header files (make the headers appear in IDEs.)
FILE(GLOB MVR_HDRS *.h)

ADD_LIBRARY(mvr ${MVR_SRC} ${MVR_HDRS})
TARGET_LINK_LIBRARIES(mvr multiview image flann)

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES(mvr PROPERTIES DEBUG_POSTFIX "_d")

# installation rules for the library
LIBMV_INSTALL_LIB(mvr)
            

# # IF (BUILD_TESTS)
# #   ADD_LIBRARY(multiview_test_data test_data_sets.cc)
# #   TARGET_LINK_LIBRARIES(multiview_test_data multiview)
# #   # make the name of debug libraries end in _d.
# #   SET_TARGET_PROPERTIES(multiview_test_data PROPERTIES DEBUG_POSTFIX "_d")
# # ENDIF (BUILD_TESTS)

# # MACRO (MULTIVIEW_TEST NAME)
# #   LIBMV_TEST(${NAME} "multiview_test_data;multiview;numeric")
# # ENDMACRO (MULTIVIEW_TEST)
# # 
# # MULTIVIEW_TEST(projection)

# LIBMV_TEST(tracker "correspondence;reconstruction;numeric;flann")
# LIBMV_TEST(mvr "correspondence;reconstruction;numeric;flann")
LIBMV_TEST(mvr "mvr")