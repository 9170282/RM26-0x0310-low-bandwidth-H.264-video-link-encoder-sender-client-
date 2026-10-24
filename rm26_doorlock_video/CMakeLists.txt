cmake_minimum_required(VERSION 3.16)

project(rm26_doorlock_video CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# 依赖
# ---------------------------------------------------------------------------
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0>=1.4 gstreamer-app-1.0>=1.4)

# 海康 MVS SDK（相机用）。默认安装路径 /opt/MVS，
# 可用 cmake -DMVS_SDK_ROOT=/your/MVS 覆盖。
set(MVS_SDK_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK root")
find_path(MVS_INCLUDE_DIR MvCameraControl.h
  HINTS ${MVS_SDK_ROOT}/include
  PATHS /opt/MVS/include)
find_library(MVS_LIBRARY NAMES MvCameraControl
  HINTS ${MVS_SDK_ROOT}/lib/64 ${MVS_SDK_ROOT}/lib/aarch64 ${MVS_SDK_ROOT}/lib)
if(NOT MVS_INCLUDE_DIR OR NOT MVS_LIBRARY)
  message(FATAL_ERROR
    "找不到海康 MVS SDK (MvCameraControl.h / libMvCameraControl.so)。\n"
    "请先安装海康 MVS SDK，或使用 -DMVS_SDK_ROOT=<MVS 安装目录> 指定。")
endif()

# ---------------------------------------------------------------------------
# 串口库（wjwwood/serial 精简移植，位于 io/serial）
# ---------------------------------------------------------------------------
add_subdirectory(io/serial)

# ---------------------------------------------------------------------------
# tools（crc / exiter / logger / yaml / thread_safe_queue）
# ---------------------------------------------------------------------------
add_library(tools STATIC
  tools/crc.cpp
  tools/exiter.cpp
  tools/logger.cpp
)
target_include_directories(tools PUBLIC ${PROJECT_SOURCE_DIR})
target_link_libraries(tools PUBLIC fmt::fmt spdlog::spdlog)

# ---------------------------------------------------------------------------
# io（海康相机）
# ---------------------------------------------------------------------------
add_library(io STATIC
  io/camera.cpp
  io/hikrobot/hikrobot.cpp
)
target_include_directories(io PUBLIC
  ${PROJECT_SOURCE_DIR}
  ${MVS_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(io PUBLIC
  ${OpenCV_LIBS}
  ${MVS_LIBRARY}
  usb-1.0
  yaml-cpp
  serial
  tools
)

# ---------------------------------------------------------------------------
# doorlock（H.264 编码器 + 0x0310 串口发送器）
# ---------------------------------------------------------------------------
add_library(doorlock STATIC
  tasks/doorlock/src/doorlock_encoder.cpp
  tasks/doorlock/src/referee_serial_sender.cpp
)
target_include_directories(doorlock PUBLIC
  ${PROJECT_SOURCE_DIR}
  ${GSTREAMER_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(doorlock PUBLIC
  ${OpenCV_LIBS}
  ${GSTREAMER_LIBRARIES}
  yaml-cpp
  serial
  tools
)

# ---------------------------------------------------------------------------
# 可执行程序
# ---------------------------------------------------------------------------
add_executable(doorlock_sniper_usb src/doorlock_sniper_usb.cpp)
target_link_libraries(doorlock_sniper_usb PRIVATE
  doorlock
  io
  tools
  ${OpenCV_LIBS}
  yaml-cpp
  fmt::fmt
)
