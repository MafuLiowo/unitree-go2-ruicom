cmake_minimum_required(VERSION 3.5)
project(go2_video_client)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(default_build_type "Release")
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Try to find unitree_sdk2 in system paths
find_library(UNITREE_SDK_LIB unitree_sdk2
  PATHS
    /usr/local/lib
    /usr/lib
    ${CMAKE_SOURCE_DIR}/unitree_sdk2/lib/${CMAKE_SYSTEM_PROCESSOR}
)

# Try to find DDS libraries
find_library(DDSC_LIB ddsc
  PATHS
    /usr/local/lib
    /usr/lib
    ${CMAKE_SOURCE_DIR}/unitree_sdk2/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}
)

find_library(DDSCXX_LIB ddscxx
  PATHS
    /usr/local/lib
    /usr/lib
    ${CMAKE_SOURCE_DIR}/unitree_sdk2/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}
)

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

if(DDSC_LIB)
  add_library(ddsc SHARED IMPORTED GLOBAL)
  set_target_properties(ddsc PROPERTIES IMPORTED_LOCATION ${DDSC_LIB})
else()
  message(FATAL_ERROR "DDS library ddsc not found")
endif()

if(DDSCXX_LIB)
  add_library(ddscxx SHARED IMPORTED GLOBAL)
  set_target_properties(ddscxx PROPERTIES IMPORTED_LOCATION ${DDSCXX_LIB})
else()
  message(FATAL_ERROR "DDS library ddscxx not found")
endif()

if(UNITREE_SDK_LIB)
  add_library(unitree_sdk2 SHARED IMPORTED GLOBAL)
  set_target_properties(unitree_sdk2 PROPERTIES IMPORTED_LOCATION ${UNITREE_SDK_LIB})
else()
  message(FATAL_ERROR "Unitree SDK2 library not found")
endif()

include_directories(${CMAKE_SOURCE_DIR}/unitree_sdk2/include)
include_directories(${CMAKE_SOURCE_DIR}/unitree_sdk2/thirdparty/include)
include_directories(${CMAKE_SOURCE_DIR}/unitree_sdk2/thirdparty/include/ddscxx)
include_directories(${CMAKE_SOURCE_DIR}/include)

# Add system DDS include paths if they exist
include_directories(/usr/local/include/ddscxx)
include_directories(/usr/local/include/ddsc)
include_directories(/usr/include/ddscxx)
include_directories(/usr/include/ddsc)

# LineProcessor library
add_library(LineProcessor src/LineProcessor.cpp)
target_link_libraries(LineProcessor ${OpenCV_LIBS})

# Line following executable
add_executable(go2_line_following src/go2_line_following.cpp)
target_link_libraries(go2_line_following 
    LineProcessor
    unitree_sdk2 
    ddsc 
    ddscxx 
    ${OpenCV_LIBS} 
    Threads::Threads 
    realsense2::realsense2
)

# Example executable
add_executable(go2_video_client src/go2_video_client.cpp)
target_link_libraries(go2_video_client unitree_sdk2 ddsc ddscxx ${OpenCV_LIBS} Threads::Threads)

# Video display executable
add_executable(go2_video_display src/go2_video_display.cpp)
target_link_libraries(go2_video_display unitree_sdk2 ddsc ddscxx ${OpenCV_LIBS} Threads::Threads)

# Interactive sport client
add_executable(go2_sport_interactive src/go2_sport_interactive.cpp)
target_link_libraries(go2_sport_interactive unitree_sdk2 ddsc ddscxx Threads::Threads)

# YOLO detector library
add_library(YOLODetector src/YOLODetector.cpp)
target_include_directories(YOLODetector PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(YOLODetector ${OpenCV_LIBS})

# Safety sign detection main program
add_executable(go2_safety_detection src/go2_safety_detection.cpp)
target_link_libraries(go2_safety_detection
    YOLODetector
    unitree_sdk2
    ddsc
    ddscxx
    ${OpenCV_LIBS}
    Threads::Threads
)

# Image processing library (shared between go2_process_images and go2_seeking_way)
add_library(ImageProcessor src/go2_process_images.cpp)
target_include_directories(ImageProcessor PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(ImageProcessor ${OpenCV_LIBS})

# Visual path seeking main program
add_executable(go2_seeking_way src/go2_seeking_way.cpp)
target_link_libraries(go2_seeking_way
    ImageProcessor
    unitree_sdk2
    ddsc
    ddscxx
    ${OpenCV_LIBS}
    Threads::Threads
)

# Standalone image processing executable
add_executable(go2_process_images src/go2_process_images_main.cpp)
target_link_libraries(go2_process_images
    ImageProcessor
    unitree_sdk2
    ddsc
    ddscxx
    ${OpenCV_LIBS}
    Threads::Threads
)

# Test program for YOLO detector
# add_executable(test_yolo src/test_yolo.cpp)
# target_link_libraries(test_yolo
#     YOLODetector
#     ${OpenCV_LIBS}
# )