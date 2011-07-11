HEADERS += main.h
SOURCES += main.cc
RESOURCES = calibration.qrc

#in case the user also need to compile OpenCV himself
LIBS += -L/usr/local/lib/

LIBS += -lopencv_highgui -lopencv_calib3d -lopencv_core

exists(/usr/include/libavcodec/avcodec.h):CONFIG+=ffmpeg
ffmpeg {
 DEFINES += USE_FFMPEG
 LIBS += -lavcodec -lavformat
}

OBJECTS_DIR=build
MOC_DIR=build
RCC_DIR=build
DESTDIR=build
