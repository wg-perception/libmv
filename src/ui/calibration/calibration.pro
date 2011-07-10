HEADERS += main.h
SOURCES += main.cc
RESOURCES = calibration.qrc
LIBS += -lopencv_highgui -lopencv_calib3d

exists(/usr/include/libavcodec/avcodec.h):CONFIG+=ffmpeg
ffmpeg {
 DEFINES += USE_FFMPEG
 LIBS += -lavcodec -lavformat
}

OBJECTS_DIR=build
MOC_DIR=build
RCC_DIR=build
DESTDIR=build
