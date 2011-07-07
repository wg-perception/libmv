HEADERS += main.h
SOURCES += main.cc
RESOURCES = calibration.qrc
LIBS += -lopencv_highgui -lopencv_calib3d

OBJECTS_DIR=build
MOC_DIR=build
RCC_DIR=build
DESTDIR=build
