QT += opengl
HEADERS += clip.h        tracker.h  zoom.h  main.h
SOURCES += clip.cc gl.cc tracker.cc zoom.cc main.cc
HEADERS += ../../libmv/tracking/brute_region_tracker.h \
    ../../libmv/tracking/esm_region_tracker.h \
    ../../libmv/tracking/hybrid_region_tracker.h
SOURCES += ../../libmv/tracking/brute_region_tracker.cc \
    ../../libmv/tracking/esm_region_tracker.cc \
    ../../libmv/tracking/hybrid_region_tracker.cc
HEADERS += ../../libmv/simple_pipeline/detect.h
SOURCES += ../../libmv/simple_pipeline/detect.cc
#TODO: Qt Tracker should be independent from libmv to be able to use new lens distortion API
HEADERS += calibration.h ../../libmv/simple_pipeline/camera_intrinsics.h
SOURCES += calibration.cc ../../libmv/simple_pipeline/camera_intrinsics.cc

#TODO: we don't actually need glog stuff in tracker, but to prevent linking errors for now...
SOURCES += ../../third_party/glog/src/demangle.cc \
    ../../third_party/glog/src/logging.cc \
    ../../third_party/glog/src/raw_logging.cc \
    ../../third_party/glog/src/signalhandler.cc \
    ../../third_party/glog/src/symbolize.cc \
    ../../third_party/glog/src/utilities.cc \
    ../../third_party/glog/src/vlog_is_on.cc \
    ../../third_party/gflags/gflags.cc \
    ../../third_party/gflags/gflags_completions.cc \
    ../../third_party/gflags/gflags_reporting.cc
INCLUDEPATH += ../../third_party/glog/src

SOURCES += ../../third_party/fast/fast_10.c \
    ../../third_party/fast/fast_11.c \
    ../../third_party/fast/fast_12.c \
    ../../third_party/fast/fast_9.c \
    ../../third_party/fast/fast.c \
    ../../third_party/fast/nonmax.c

SOURCES += ../../libmv/image/convolve.cc

#HEADERS += scene.h
#SOURCES += scene.cc
OTHER_FILES += shader.glsl
RESOURCES = tracker.qrc
INCLUDEPATH += ../..
INCLUDEPATH += ../../third_party/eigen
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
 LIBS += -lavcodec -lavformat -lavutil
}

OBJECTS_DIR=build
MOC_DIR=build
RCC_DIR=build
DESTDIR=build

OTHER_FILES += CMakeLists.txt ../../libmv/simple_pipeline/CMakeLists.txt
OTHER_FILES += ../../libmv/simple_pipeline/*.h ../../libmv/simple_pipeline/*.cc
OTHER_FILES += ../../libmv/multiview/fundamental.cc ../../libmv/multiview/projection.cc
