FIND_PACKAGE(Qt4 REQUIRED)
SET(QT_USE_QTOPENGL ON)
INCLUDE(${QT_USE_FILE})
find_package(OpenGL)
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
SET(SOURCES          clip.cc calibration.cc gl.cc tracker.cc zoom.cc main.cc)
QT4_WRAP_CPP(SOURCES clip.h  calibration.h        tracker.h  zoom.h  main.h)
QT4_ADD_RESOURCES(SOURCES tracker.qrc)
ADD_EXECUTABLE(qt-tracker ${SOURCES})
TARGET_LINK_LIBRARIES(qt-tracker
                      ${QT_LIBRARIES}
                      ${OPENGL_LIBRARIES}
                      ${FFMPEG_LIBRARIES}
                      simple_pipeline
                      tracking
                      multiview
                      image
                      ceres)
LIBMV_INSTALL_EXE(qt-tracker)
