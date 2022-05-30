cmake_minimum_required(VERSION 3.5)
project(mujoco_panda)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mujoco_panda REQUIRED)

set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

find_package(GLEW REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Commands.srv"
)

add_executable(send_topic src/send_command_topic_node.cpp)
ament_target_dependencies(send_topic
  rclcpp
  std_msgs
)

add_executable(send_request src/send_command_request_node.cpp)
ament_target_dependencies(send_request
  rclcpp
  mujoco_panda
)

set(SOURCES
  src/model.cpp
  src/trajectory.cpp
  src/controller.cpp
  src/quadratic_programming.cpp
)

set(MUJOCO_PATH
  $ENV{HOME}/Simulators/mujoco-2.1.1
)

file(GLOB LIB_MUJOCO ${MUJOCO_PATH}/lib/libmujoco.so)
file(GLOB UI_SRC ${MUJOCO_PATH}/sample/uitools.c)

message("Mujoco Path : " ${MUJOCO_PATH})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
  ${MUJOCO_PATH}/include
  ${MUJOCO_PATH}/sample
)

link_directories(
    ${MUJOCO_PATH}/lib
)

## Declare a C++ library
add_library(panda_lib
  ${UI_SRC}
  ${SOURCES}
)

target_link_libraries(panda_lib
  Eigen3::Eigen
  -lrbdl
  -lrbdl_urdfreader
  -lpthread
  -lqpOASES
)
ament_target_dependencies(panda_lib
  rclcpp
  mujoco_panda
)

add_executable(panda src/simulate.cc
  ${MUJOCO_PATH}/include/mjxmacro.h
  ${MUJOCO_PATH}/sample/uitools.h
)

## Specify libraries to link a library or executable target against
target_link_libraries(panda
PUBLIC
  ${MUJOCO_PATH}/lib/libglew.so
  #${MUJOCO_PATH}/lib/libglfw.so.3
  #GL
  ${LIB_MUJOCO}
  OpenGL::GL
  OpenGL::OpenGL
  OpenGL::EGL
  GLEW::GLEW
  -lglfw
  ${GLEW_LIBRARIES}
  ${OPENGL_LIBRARIES}
  panda_lib
)

install(TARGETS
  panda
  send_topic
  send_request
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
