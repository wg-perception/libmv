
SET(MOC_HEADERS main_window.h match_viewer.h 3D_viewer.h)
SET(SOURCES main.cc main_window.cc match_viewer.cc 3D_viewer.cc features.cc)
QT4_WRAP_CPP(SOURCES ${MOC_HEADERS})

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
ADD_EXECUTABLE(tvr ${SOURCES})
TARGET_LINK_LIBRARIES(tvr
                      image
                      correspondence
					  tracking
                      numeric
                      multiview
                      simple_pipeline
                      flann
                      glog
                      gflags
                      fast
                      ${QT_QTCORE_LIBRARY}
                      ${QT_QTGUI_LIBRARY}
                      ${QT_QTOPENGL_LIBRARY}
                      ${OPENGL_gl_LIBRARY}
                      ${OPENGL_glu_LIBRARY}
                      ${OpenCV_LIBRARIES})

LIBMV_INSTALL_EXE(tvr)
