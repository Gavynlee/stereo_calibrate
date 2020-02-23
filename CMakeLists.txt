cmake_minimum_required(VERSION 2.8)
project( stereo )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# OpenCV 
find_package( OpenCV REQUIRED )

include_directories( 
${PROJECT_SOURCE_DIR}/include
${OpenCV_INCLUDE_DIRS} 
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/monocular.cc
src/stereo.cc
)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable( Stereo_Calibrate main.cc )
target_link_libraries( Stereo_Calibrate ${PROJECT_NAME} )