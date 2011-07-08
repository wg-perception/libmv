ADD_SUBDIRECTORY(tracker)

OPTION(BUILD_CALIBRATION "Build Qt Calibration" ON)
IF (BUILD_CALIBRATION)
 MESSAGE(STATUS "Calibration Tool enabled. You need to install OpenCV to build Qt Calibration")
 ADD_SUBDIRECTORY(calibration)
ENDIF (BUILD_CALIBRATION)
