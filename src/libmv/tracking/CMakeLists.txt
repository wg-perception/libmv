SET(TRACKING_SRC klt.cc sad.cc)

# Define the header files so that they appear in IDEs.
FILE(GLOB TRACKING_HDRS *.h)

ADD_LIBRARY(tracking ${TRACKING_SRC} ${TRACKING_HDRS})

TARGET_LINK_LIBRARIES(tracking image)

# Make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES(tracking PROPERTIES DEBUG_POSTFIX "_d")

LIBMV_INSTALL_LIB(tracking)
            
