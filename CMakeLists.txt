cmake_minimum_required(VERSION 3.8)
project(blueberry-console)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(edu_robot REQUIRED)

# edu_robot dependencies
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# find OpenGL, GLEW and GLFW
cmake_policy(SET CMP0072 NEW)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 3.3 REQUIRED)

# add blueberry_node
add_executable(blueberry_node
  src/stb_image.cpp
  src/blueberry_node.cpp
  src/node.cpp
  
  include/imgui/imgui_demo.cpp
  include/imgui/imgui_draw.cpp
  include/imgui/imgui_tables.cpp
  include/imgui/imgui_widgets.cpp
  include/imgui/imgui.cpp
  include/imgui/backends/imgui_impl_glfw.cpp
  include/imgui/backends/imgui_impl_opengl3.cpp
  
  include/implot/implot.cpp
  include/implot/implot_items.cpp)

# link with OpenGL, GLEW and GLFW
target_link_libraries(blueberry_node OpenGL::GL GLEW::GLEW glfw)
# link with rclcpp and std_msgs
ament_target_dependencies(blueberry_node
  rclcpp
  std_msgs
  edu_robot
    Eigen3
    nav_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
)

# add includes
target_include_directories(blueberry_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  include/stb
  include/imgui
  include/implot)
target_compile_features(blueberry_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS blueberry_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
