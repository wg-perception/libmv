QT += opengl
HEADERS += clip.h        tracker.h  zoom.h  main.h
SOURCES += clip.cc gl.cc tracker.cc zoom.cc main.cc
HEADERS += ../../libmv/tracking/sad.h
SOURCES += ../../libmv/tracking/sad.cc
HEADERS += ../../libmv/simple_pipeline/detect.h
SOURCES += ../../libmv/simple_pipeline/detect.cc
#TODO: Qt Tracker should be independent from libmv to be able to use new lens distortion API
HEADERS += calibration.h ../../libmv/simple_pipeline/camera_intrinsics.h
SOURCES += calibration.cc ../../libmv/simple_pipeline/camera_intrinsics.cc
#HEADERS += scene.h
#SOURCES += scene.cc
OTHER_FILES += shader.glsl
RESOURCES = tracker.qrc
INCLUDEPATH += ../..
INCLUDEPATH += /usr/include/eigen3/
#LIBS += -L../../../bin-opt/lib/ -lsimple_pipeline -lmultiview
QMAKE_CXXFLAGS_RELEASE += -Ofast -march=native

win32:CONFIG+=glew
glew {
 DEFINES += GLEW
 win32 {
  HEADERS += GL/glew.h GL/wglew.h
  SOURCES += glew.c
  DEFINES += GLEW_STATIC
 }
 unix {
  LIBS += -lGLEW
 }
}

exists(/usr/include/libavcodec/avcodec.h):CONFIG+=ffmpeg
ffmpeg {
 DEFINES += USE_FFMPEG
 LIBS += -lavcodec -lavformat
}

OBJECTS_DIR=build
MOC_DIR=build
RCC_DIR=build
DESTDIR=build

OTHER_FILES += CMakeLists.txt ../../libmv/simple_pipeline/CMakeLists.txt
OTHER_FILES += ../../libmv/simple_pipeline/*.h ../../libmv/simple_pipeline/*.cc
OTHER_FILES += ../../libmv/multiview/fundamental.cc ../../libmv/multiview/projection.cc
