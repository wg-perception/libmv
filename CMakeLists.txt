# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.

cmake_minimum_required(VERSION 2.6)
add_subdirectory("src")

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/libmv.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/libmv.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/libmv.pc DESTINATION lib/pkgconfig)
