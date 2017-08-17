cmake_minimum_required (VERSION 2.8)
project (LabCalibrationToolbox)
set (ME_VERSION_MAJOR 1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -std=c++11")
file(GLOB SOURCE_FILES ${CMAKE_SOURCE_DIR}/src/calib_*.cpp ${CMAKE_SOURCE_DIR}/src/patternDetection.cpp)

find_package(OpenCV 3 REQUIRED)

#Documentation
find_package(Doxygen)
if(DOXYGEN_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
add_custom_target(doc ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
endif(DOXYGEN_FOUND)

include_directories(${CMAKE_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
#Monocular calibration
add_executable(lab_mono_calibration ${CMAKE_SOURCE_DIR}/src/main_mono.cpp ${SOURCE_FILES})
target_link_libraries(lab_mono_calibration ${OpenCV_LIBS})
#Stereo calibration
add_executable(lab_stereo_calibration ${CMAKE_SOURCE_DIR}/src/main_stereo.cpp ${SOURCE_FILES})
target_link_libraries(lab_stereo_calibration ${OpenCV_LIBS})
