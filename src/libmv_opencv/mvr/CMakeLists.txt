# define the source files
SET(MVR_SCR mvr.cc)

# define the header files (make the headers appear in IDEs.)
FILE(GLOB MVR_HDRS *.h)

ADD_LIBRARY(mvr ${MVR_SRC} ${MVR_HDRS})
TARGET_LINK_LIBRARIES(mvr multiview image flann) #TODO: refined this

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES(mvr PROPERTIES DEBUG_POSTFIX "_d")

# installation rules for the library
LIBMV_INSTALL_LIB(mvr)
            
ADD_LIBRARY(multiview_test_data2 ../../libmv/multiview/test_data_sets.cc)
TARGET_LINK_LIBRARIES(multiview_test_data2 multiview)
SET_TARGET_PROPERTIES(multiview_test_data2 PROPERTIES DEBUG_POSTFIX "_d")
# LIBMV_TEST(mvr "mvr;multiview_test_data2")

# # define the source files
# SET(CORRESPONDENCE_SRC feature.cc 
#                        matches.cc 
#                        feature_matching.cc
# #                      feature_matching_FLANN.cc
#                        tracker.cc
#                        robust_tracker.cc
#                        planar_tracker.cc
#                        nRobustViewMatching.cc
#                        export_matches_txt.cc
#                        import_matches_txt.cc)
# 
# # define the header files (make the headers appear in IDEs.)
# FILE(GLOB CORRESPONDENCE_HDRS *.h)
# 
# ADD_LIBRARY(correspondence ${CORRESPONDENCE_SRC} ${CORRESPONDENCE_HDRS})
# TARGET_LINK_LIBRARIES(correspondence multiview image flann)
# 
# # make the name of debug libraries end in _d.
# SET_TARGET_PROPERTIES(correspondence PROPERTIES DEBUG_POSTFIX "_d")
# 
# # installation rules for the library
# LIBMV_INSTALL_LIB(correspondence)
#             
# LIBMV_TEST(bipartite_graph "")
# LIBMV_TEST(kdtree "")
# LIBMV_TEST(feature_set "correspondence;image;numeric")
# LIBMV_TEST(matches "correspondence;image;numeric")
# LIBMV_TEST(Array_Matcher "correspondence;numeric;flann")
# # LIBMV_TEST(tracker "correspondence;reconstruction;numeric;flann")


  
