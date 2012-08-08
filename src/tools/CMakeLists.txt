find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
include_directories(SYSTEM ../)

ADD_EXECUTABLE(experimental experimental.cc)
TARGET_LINK_LIBRARIES(experimental
                      multiview_test_data
                      multiview
                      numeric
                      pthread
                      gflags
                      glog
                      )
LIBMV_INSTALL_EXE(experimental)

#ADD_EXECUTABLE(extractExifData extractExifData.cc)
#TARGET_LINK_LIBRARIES(extractExifData
#                      image
#                      numeric
#                      multiview
#                      camera
#                      glog
#                      gflags
#                      )
#LIBMV_INSTALL_EXE(extractExifData)

ADD_EXECUTABLE(interest_points interest_points.cc)
TARGET_LINK_LIBRARIES(interest_points
                      image
                      numeric
                      multiview
                      camera
                      glog
                      gflags
                      )
LIBMV_INSTALL_EXE(interest_points)

ADD_EXECUTABLE(mosaicing_video mosaicing_video.cc)
TARGET_LINK_LIBRARIES(mosaicing_video
                      correspondence
                      image
                      numeric
                      multiview
                      camera
                      glog
                      gflags
                      )
LIBMV_INSTALL_EXE(mosaicing_video)

ADD_EXECUTABLE(nViewMatching nViewMatching.cc)
TARGET_LINK_LIBRARIES(nViewMatching correspondence
                      image
                      gflags
                      glog
                      multiview
                      )
LIBMV_INSTALL_EXE(nViewMatching)

ADD_EXECUTABLE(reconstruct_video reconstruct_video.cc)
TARGET_LINK_LIBRARIES(reconstruct_video
                      image
                      numeric
                      multiview
                      glog
                      gflags
                      reconstruction
                      )
LIBMV_INSTALL_EXE(reconstruct_video)

ADD_EXECUTABLE(stabilize stabilize.cc)
TARGET_LINK_LIBRARIES(stabilize
                      image
                      numeric
                      multiview
                      glog
                      gflags
                      reconstruction
                      )
LIBMV_INSTALL_EXE(stabilize)

# TODO(keir): Update this with the new API.
#ADD_EXECUTABLE(track track.cc)
#TARGET_LINK_LIBRARIES(track image image gflags glog)
#LIBMV_INSTALL_EXE(track)

ADD_EXECUTABLE(tracker tracker.cc)
TARGET_LINK_LIBRARIES(tracker
                      image
                      numeric
                      multiview
                      glog
                      gflags
                      fast
                      reconstruction
                      )
LIBMV_INSTALL_EXE(tracker)

ADD_EXECUTABLE(undistort undistort.cc)
TARGET_LINK_LIBRARIES(undistort
                      image
                      numeric
                      multiview
                      camera
                      glog
                      gflags
                      )
LIBMV_INSTALL_EXE(reconstruct_video)
