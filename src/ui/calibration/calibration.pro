HEADERS += main.h
SOURCES += main.cc
RESOURCES = calibration.qrc

#in case the user also need to compile OpenCV himself
LIBS += -L/usr/local/lib/
#catch errors which could occur on systems using gold
LIBS += -Wl,--no-as-needed

LIBS += -lopencv_highgui -lopencv_calib3d -lopencv_imgproc -lopencv_core

exists(/usr/include/ffmpeg) {
 CONFIG+=ffmpeg
 QMAKE_CXXFLAGS += -I/usr/include/ffmpeg
}
exists(/usr/include/libavcodec/avcodec.h):CONFIG+=ffmpeg
ffmpeg {
 DEFINES += USE_FFMPEG
 LIBS += -lavcodec -lavformat -lavutil

}

OBJECTS_DIR=build
MOC_DIR=build
RCC_DIR=build
DESTDIR=build
