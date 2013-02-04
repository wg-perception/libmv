SET(SIMPLE_PIPELINE_SRC
    detect.cc
    resect.cc
    intersect.cc
    bundle.cc
    initialize_reconstruction.cc
    iterative_closest_points.cc
    keyframe_selection.cc
    modal_solver.cc
    pipeline.cc
    reconstruction.cc
    camera_intrinsics.cc
    tracks.cc
    uncalibrated_reconstructor.cc
    autocalibrate.cc
    rigid_registration.cc
    callbacks.cc)

# Define the header files so that they appear in IDEs.
FILE(GLOB SIMPLE_PIPELINE_HDRS *.h)

ADD_LIBRARY(simple_pipeline ${SIMPLE_PIPELINE_SRC} ${SIMPLE_PIPELINE_HDRS})

TARGET_LINK_LIBRARIES(simple_pipeline V3D multiview fast)

# Make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES(simple_pipeline PROPERTIES DEBUG_POSTFIX "_d")

LIBMV_INSTALL_LIB(simple_pipeline)

MACRO (SIMPLE_PIPELINE_TEST NAME)
  LIBMV_TEST(${NAME} "simple_pipeline")
ENDMACRO (SIMPLE_PIPELINE_TEST)

SIMPLE_PIPELINE_TEST(camera_intrinsics)
SIMPLE_PIPELINE_TEST(resect)
SIMPLE_PIPELINE_TEST(intersect)
SIMPLE_PIPELINE_TEST(keyframe_selection)
