cmake_minimum_required(VERSION 3.8)
project(rm_mppi_controller)

set(CXXFLAGS="-march=native $CXXFLAGS")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(EIGEN_USE_THREADS)  # 使用 Eigen 多线程

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)
if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(TBB REQUIRED)

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

include_directories(include)
set(library_name ${PROJECT_NAME}_plugin)
add_library(${library_name} SHARED
  src/mppi_controller.cpp
)

target_link_libraries(${library_name}
  # /usr/lib/x86_64-linux-gnu/libtbb.so
  TBB::tbb
)

ament_target_dependencies(${library_name}
  pluginlib
  nav2_core
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)
ament_package()
