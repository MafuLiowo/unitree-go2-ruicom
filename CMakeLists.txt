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

# 宇树 SDK 新版 DDS (CycloneDDS 0.10.5) 路径，优先级高于系统安装版本
set(UNITREE_DDS_LIB_DIR /home/mafu/unitree_ros2/cyclonedds_ws/install/cyclonedds/lib)
set(UNITREE_DDS_INCLUDE_DIR /home/mafu/unitree_ros2/cyclonedds_ws/install/cyclonedds/include)

# Try to find unitree_sdk2 in system paths
find_library(UNITREE_SDK_LIB unitree_sdk2
  PATHS
    /usr/local/lib
    /usr/lib
    ${CMAKE_SOURCE_DIR}/unitree_sdk2/lib/${CMAKE_SYSTEM_PROCESSOR}
)

# Try to find DDS libraries — 优先使用宇树 SDK 附带的新版 DDS (CycloneDDS 0.10.5)
find_library(DDSC_LIB ddsc
  PATHS
    ${UNITREE_DDS_LIB_DIR}
    ${CMAKE_SOURCE_DIR}/unitree_sdk2/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}
    /usr/local/lib
    /usr/lib
)

find_library(DDSCXX_LIB ddscxx
  PATHS
    ${UNITREE_DDS_LIB_DIR}
    ${CMAKE_SOURCE_DIR}/unitree_sdk2/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}
    /usr/local/lib
    /usr/lib
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

# ---- Unitree SDK2 头文件路径（供非 ROS2 目标使用） ----
# 新版 DDS (CycloneDDS 0.10.5) 头文件优先
set(UNITREE_INCLUDES
    ${CMAKE_SOURCE_DIR}/unitree_sdk2/include
    ${CMAKE_SOURCE_DIR}/unitree_sdk2/thirdparty/include
    ${CMAKE_SOURCE_DIR}/unitree_sdk2/thirdparty/include/ddscxx
    ${UNITREE_DDS_INCLUDE_DIR}
    ${UNITREE_DDS_INCLUDE_DIR}/ddsc
    /usr/local/include/ddscxx
    /usr/local/include/ddsc
    /usr/include/ddscxx
    /usr/include/ddsc
)

# 项目头文件路径
set(PROJECT_INCLUDES
    ${CMAKE_SOURCE_DIR}/include
)

# ---- 通用目标：为所有需要 Unitree SDK2 的目标添加头文件路径 ----
function(add_unitree_includes target)
    target_include_directories(${target} PRIVATE ${UNITREE_INCLUDES} ${PROJECT_INCLUDES})
endfunction()

# ---- 不含 Unitree SDK2 的目标 ----

# ImageProcessor library (连通域分析/路径检测) — 暂不可编译
# add_library(ImageProcessor src/go2_process_images.cpp)
# target_link_libraries(ImageProcessor ${OpenCV_LIBS})

# Process image executable (HSV颜色识别与V通道反二值化)
add_executable(go2_process_image src/go2_process_image.cpp)
target_include_directories(go2_process_image PRIVATE ${PROJECT_INCLUDES})
target_link_libraries(go2_process_image ${OpenCV_LIBS} realsense2::realsense2)

# Video display executable (RealSense实时画面显示)
add_executable(go2_video_display src/go2_video_display.cpp)
target_include_directories(go2_video_display PRIVATE ${PROJECT_INCLUDES})
target_link_libraries(go2_video_display ${OpenCV_LIBS} realsense2::realsense2)

# ---- 含 Unitree SDK2 的目标 ----

# Seeking way executable (视觉寻路)
# add_executable(go2_seeking_way src/go2_seeking_way.cpp)
# add_unitree_includes(go2_seeking_way)
# target_link_libraries(go2_seeking_way ImageProcessor unitree_sdk2 ddsc ddscxx ${OpenCV_LIBS} Threads::Threads)

# Sport switch executable (高层运动控制，支持 Stretch/Hello/StopMove/FrontJump)
add_executable(go2_sport_switch src/go2_sport_switch.cpp)
add_unitree_includes(go2_sport_switch)
target_link_libraries(go2_sport_switch unitree_sdk2 ddsc ddscxx Threads::Threads)

# Light controller executable (前灯控制，支持闪烁等灯效)
add_executable(go2_light_controller src/go2_light_controller.cpp)
add_unitree_includes(go2_light_controller)
target_link_libraries(go2_light_controller unitree_sdk2 ddsc ddscxx Threads::Threads)

# Origin camera display executable (Go2 原生摄像头画面显示)
add_executable(go2_video_display_origin src/go2_video_display_origin.cpp)
add_unitree_includes(go2_video_display_origin)
target_link_libraries(go2_video_display_origin unitree_sdk2 ddsc ddscxx ${OpenCV_LIBS} Threads::Threads)

# Photo capture executable (Go2 原生相机拍照，保存到 ./photo/IMG_xxx.jpg)
add_executable(go2_photo src/go2_photo.cpp)
add_unitree_includes(go2_photo)
target_link_libraries(go2_photo unitree_sdk2 ddsc ddscxx ${OpenCV_LIBS} Threads::Threads)

# YOLO detector library (ONNX 模型加载、推理、后处理)
add_library(YOLODetector src/YOLODetector.cpp)
target_include_directories(YOLODetector PUBLIC ${PROJECT_INCLUDES})
target_link_libraries(YOLODetector ${OpenCV_LIBS})

# YOLO identify executable (目标检测识别程序，使用 Go2 原生摄像头)
add_executable(go2_yolo_identify src/go2_yolo_identify.cpp)
add_unitree_includes(go2_yolo_identify)
target_link_libraries(go2_yolo_identify
    YOLODetector
    unitree_sdk2
    ddsc
    ddscxx
    ${OpenCV_LIBS}
    Threads::Threads
)

# Go2 action executable (视觉动作联动程序，YOLO识别触发运动/灯光动作)
add_executable(go2_action src/go2_action.cpp)
add_unitree_includes(go2_action)
target_link_libraries(go2_action
    YOLODetector
    unitree_sdk2
    ddsc
    ddscxx
    ${OpenCV_LIBS}
    Threads::Threads
)

# Walking executable (视觉巡线行走程序，RealSense二值化+路径检测+运动控制)
add_executable(go2_walking src/go2_walking.cpp)
add_unitree_includes(go2_walking)
target_link_libraries(go2_walking
    unitree_sdk2
    ddsc
    ddscxx
    ${OpenCV_LIBS}
    Threads::Threads
    realsense2::realsense2
)

# Jump executable (RealSense深度横棒检测 → FrontJump前跳)
add_executable(go2_jump src/go2_jump.cpp)
add_unitree_includes(go2_jump)
target_link_libraries(go2_jump
    unitree_sdk2
    ddsc
    ddscxx
    ${OpenCV_LIBS}
    Threads::Threads
    realsense2::realsense2
)

# Diagnostic check executable (段错误诊断工具) — 暂不可编译
# add_executable(check src/check.cpp)
# add_unitree_includes(check)
# target_link_libraries(check
#     YOLODetector
#     unitree_sdk2
#     ddsc
#     ddscxx
#     ${OpenCV_LIBS}
#     Threads::Threads
# )

# ---- Go2 运动桥接库（仅链接 Unitree SDK2，不含 ROS2） ----
add_library(go2_motion_bridge src/go2_motion_bridge.cpp)
add_unitree_includes(go2_motion_bridge)
target_link_libraries(go2_motion_bridge
    unitree_sdk2
    ddsc
    ddscxx
    Threads::Threads
)

# ---- ROS2 包 ----
# 注意: 构建前需先 source ROS2 环境 (如 source /opt/ros/foxy/setup.bash)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# SLAM + Nav2 桥接节点（仅含 ROS2 + motion bridge，不含 Unitree DDS 头文件）
add_executable(go2_slam src/go2_slam.cpp)
target_include_directories(go2_slam PRIVATE
    ${PROJECT_INCLUDES}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${nav2_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${builtin_interfaces_INCLUDE_DIRS}
)
target_link_libraries(go2_slam
    go2_motion_bridge
    unitree_sdk2
    ddsc
    ddscxx
    Threads::Threads
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${nav2_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${builtin_interfaces_LIBRARIES}
)

# ---- NavFn 全局规划器库 ----
add_library(NavFnPlanner src/NavFnPlanner.cpp)
target_include_directories(NavFnPlanner PUBLIC ${PROJECT_INCLUDES})

# ---- DWB 局部规划器库 ----
add_library(DWBPlanner src/DWBPlanner.cpp)
target_include_directories(DWBPlanner PUBLIC ${PROJECT_INCLUDES})
target_link_libraries(DWBPlanner PUBLIC NavFnPlanner)

# ---- 地图加载器库 ----
add_library(MapLoader src/MapLoader.cpp)
target_include_directories(MapLoader PUBLIC ${PROJECT_INCLUDES})

# ---- Go2 自主导航节点 (Nav2 FollowWaypoints / NavigateToPose) ----
add_executable(go2_navigation "src/ go2_navigation.cpp")
target_include_directories(go2_navigation PRIVATE
    ${PROJECT_INCLUDES}
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${nav2_msgs_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${builtin_interfaces_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
)
target_link_libraries(go2_navigation
    go2_motion_bridge
    unitree_sdk2
    ddsc
    ddscxx
    yaml-cpp
    Threads::Threads
    ${rclcpp_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${nav2_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${builtin_interfaces_LIBRARIES}
)
