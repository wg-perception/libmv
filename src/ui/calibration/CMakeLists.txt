FIND_PACKAGE(Qt4 REQUIRED)
#FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE(${QT_USE_FILE})
SET(SOURCES main.cc)
QT4_ADD_RESOURCES(SOURCES calibration.qrc)
QT4_WRAP_CPP(SOURCES main.h)
ADD_EXECUTABLE(qt-calibration ${SOURCES})
TARGET_LINK_LIBRARIES(qt-calibration
                      ${QT_LIBRARIES}
                      ${FFMPEG_LIBRARIES}
                      opencv_highgui
                      opencv_calib3d)
LIBMV_INSTALL_EXE(qt-calibration)
