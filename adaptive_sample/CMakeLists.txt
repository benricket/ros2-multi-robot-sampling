cmake_minimum_required(VERSION 3.8)
project(adaptive_sample)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message(STATUS "PYTHONPATH=$ENV{PYTHONPATH}")
message(STATUS "CMAKE_PREFIX_PATH=$ENV{CMAKE_PREFIX_PATH}")
message(STATUS "VSROS_ENV_APPLIED=$ENV{VSROS_ENV_APPLIED}")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sampling_interfaces REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

include(FetchContent)
FetchContent_Declare(
gauss_process
GIT_REPOSITORY https://github.com/andreacasalino/GaussianProcesses
GIT_TAG        main
)
FetchContent_MakeAvailable(gauss_process)

add_executable(orchestrator src/orchestrator.cpp src/sampler.cpp)
ament_target_dependencies(orchestrator rclcpp std_msgs sampling_interfaces)

target_include_directories(orchestrator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(orchestrator Eigen3::Eigen GaussianProcess)
target_compile_features(orchestrator PUBLIC c_std_99 cxx_std_17)

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
