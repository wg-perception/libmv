OPTION(USE_FFMPEG "Add video support to Qt Tracker using ffmpeg" ON)
IF (USE_FFMPEG)
 MESSAGE(STATUS "Video support enabled. Make sure you have FFMpeg (and ffmpeg-devel for binary distributions)")
 ADD_DEFINITIONS(-DUSE_FFMPEG)
 SET(FFMPEG_LIBRARIES avformat avcodec avutil)
ENDIF (USE_FFMPEG)

ADD_SUBDIRECTORY(tracker)

OPTION(BUILD_CALIBRATION "Build Qt Calibration" ON)
IF (BUILD_CALIBRATION)
 MESSAGE(STATUS "Calibration Tool enabled. Make sure you have OpenCV (and opencv-devel for binary distributions)")
 ADD_SUBDIRECTORY(calibration)
ENDIF (BUILD_CALIBRATION)
