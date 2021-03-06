#Install macro for libmv libraries
MACRO (LIBMV_INSTALL_LIB NAME)
IF (NOT WIN32)
  INSTALL(TARGETS ${NAME} ${NAME} ${NAME}
    LIBRARY DESTINATION ${LIBMV_LIBRARY_OUTPUT_DIR}
    COMPONENT libraries
    ARCHIVE DESTINATION ${LIBMV_LIBRARY_OUTPUT_DIR}
    COMPONENT libraries
    RUNTIME DESTINATION ${LIBMV_LIBRARY_OUTPUT_DIR}
    COMPONENT libraries
  )
ELSE (NOT WIN32)
  INSTALL(TARGETS ${NAME} ${NAME} ${NAME}
    LIBRARY DESTINATION ${LIBMV_LIBRARY_OUTPUT_DIR}
    COMPONENT libraries
    ARCHIVE DESTINATION ${LIBMV_LIBRARY_OUTPUT_DIR}
    COMPONENT libraries
    RUNTIME DESTINATION ${LIBMV_EXECUTABLE_OUTPUT_DIR}
    COMPONENT applications
  )
ENDIF (NOT WIN32)
ENDMACRO (LIBMV_INSTALL_LIB)

# Install all libmv and third parties headers
MACRO (LIBMV_INSTALL_ALL_HEADERS)
  INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
          DESTINATION ${LIBMV_HEADERS_OUTPUT_DIR}
          COMPONENT headers
          FILES_MATCHING PATTERN "*"
                         PATTERN ".svn" EXCLUDE
                         PATTERN "*.cpp" EXCLUDE
                         PATTERN "*.cc" EXCLUDE
                         PATTERN "*.c" EXCLUDE
                         PATTERN "*~" EXCLUDE
                         PATTERN "CMakeLists.txt" EXCLUDE
        )
ENDMACRO (LIBMV_INSTALL_ALL_HEADERS)

# Install macro for libmv binaries
MACRO (LIBMV_INSTALL_EXE NAME)
  # install libraries
  INSTALL(TARGETS ${NAME}
    RUNTIME DESTINATION ${LIBMV_EXECUTABLE_OUTPUT_DIR}
    COMPONENT applications
  )
ENDMACRO (LIBMV_INSTALL_EXE)

#Install macro for third parties libraries
MACRO (LIBMV_INSTALL_THIRD_PARTY_LIB NAME)
IF (NOT WIN32)
  INSTALL(TARGETS ${NAME} ${NAME} ${NAME}
    LIBRARY DESTINATION ${LIBMV_LIBRARY_OUTPUT_DIR}
    COMPONENT libraries
    ARCHIVE DESTINATION ${LIBMV_LIBRARY_OUTPUT_DIR}
    COMPONENT libraries
    RUNTIME DESTINATION ${LIBMV_LIBRARY_OUTPUT_DIR}
    COMPONENT libraries
  )
ELSE (NOT WIN32)
  INSTALL(TARGETS ${NAME} ${NAME} ${NAME}
    LIBRARY DESTINATION ${LIBMV_LIBRARY_OUTPUT_DIR}
    COMPONENT libraries
    ARCHIVE DESTINATION ${LIBMV_LIBRARY_OUTPUT_DIR}
    COMPONENT libraries
    RUNTIME DESTINATION ${LIBMV_EXECUTABLE_OUTPUT_DIR}
    COMPONENT applications
  )
ENDIF (NOT WIN32)
ENDMACRO (LIBMV_INSTALL_THIRD_PARTY_LIB)

# Uninstall target
CONFIGURE_FILE(
    "${CMAKE_CURRENT_SOURCE_DIR}/CMake/cmake_uninstall.cmake.in"
    "${PROJECT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY)
ADD_CUSTOM_TARGET(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${PROJECT_BINARY_DIR}/cmake_uninstall.cmake)

INSTALL(FILES CMake/modules/FindLibmv.cmake
        DESTINATION "${CMAKE_ROOT}/Modules/"
        COMPONENT headers
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
        
INSTALL(FILES CMake/modules/FindLibmv.cmake
        DESTINATION ${LIBMV_SHARE_OUTPUT_DIR}/CMake/
        COMPONENT headers
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)

CONFIGURE_FILE(../README.md ${PROJECT_BINARY_DIR} COPYONLY)
CONFIGURE_FILE(../LICENSE ${PROJECT_BINARY_DIR} COPYONLY)
CONFIGURE_FILE(../AUTHORS ${PROJECT_BINARY_DIR} COPYONLY)
CONFIGURE_FILE(../ChangeLog ${PROJECT_BINARY_DIR} COPYONLY)
CONFIGURE_FILE(../Makefile ${PROJECT_BINARY_DIR} COPYONLY)
  
INSTALL(FILES ../README.md ../LICENSE ../AUTHORS 
        DESTINATION ${LIBMV_SHARE_OUTPUT_DIR}
        COMPONENT headers
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
        
OPTION(INSTALL_SOURCE "Install the source code (C++ files,...)" OFF)
IF (INSTALL_SOURCE)
  INSTALL(DIRECTORY  ../src
          DESTINATION ${LIBMV_SHARE_OUTPUT_DIR}
          COMPONENT sources
          PATTERN ".svn" EXCLUDE)
  INSTALL(DIRECTORY  ../contrib
          DESTINATION ${LIBMV_SHARE_OUTPUT_DIR}
          COMPONENT sources
          PATTERN ".svn" EXCLUDE)
  INSTALL(DIRECTORY  ../extras
          DESTINATION ${LIBMV_SHARE_OUTPUT_DIR}
          COMPONENT sources
          PATTERN ".svn" EXCLUDE)
  INSTALL(FILES ../Makefile ../ChangeLog
          DESTINATION ${LIBMV_SHARE_OUTPUT_DIR}
          COMPONENT sources
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ)
ENDIF (INSTALL_SOURCE)

