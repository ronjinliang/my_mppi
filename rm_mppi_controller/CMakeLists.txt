cmake_minimum_required(VERSION 3.22)
project(rm_mppi_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 方法：直接设置 nvcc 标志，避免 CMake 自动架构检测
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -arch=sm_89")
# 关闭 CMake 的自动架构检测，防止其添加不支持的标志
set(CMAKE_CUDA_ARCHITECTURES OFF CACHE BOOL "Disable auto detection" FORCE)

set(CMAKE_CUDA_COMPILER "/usr/local/cuda-13.2/bin/nvcc")
# 启用 CUDA 语言（这会自动提供 CUDA::cudart 等目标，如果 CMake 版本足够新）
enable_language(CUDA)

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(TBB REQUIRED)

# 可选：显式查找 CUDA 组件以获得 cuRAND 库路径（如果上述未提供 CUDA::curand）
find_package(CUDA REQUIRED)   # 提供 CUDA_LIBRARIES, CUDA_curand_LIBRARY 等

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(library_name ${PROJECT_NAME}_plugin)
add_library(${library_name} SHARED
  src/mppi_controller.cu
)

# 强制指定链接语言为 CUDA（解决 linker language 问题）
set_target_properties(${library_name} PROPERTIES
  CXX_STANDARD 17
  CUDA_STANDARD 17
  POSITION_INDEPENDENT_CODE ON
  LINKER_LANGUAGE CUDA
)

target_include_directories(${library_name} PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CUDA_INCLUDE_DIRS}
)

# 链接库：优先使用 CUDA:: 目标（如果可用），否则回退到传统变量
if(TARGET CUDA::cudart)
  target_link_libraries(${library_name}
    TBB::tbb
    CUDA::cudart
    CUDA::curand
  )
else()
  target_link_libraries(${library_name}
    tbb
    ${CUDA_LIBRARIES}
    ${CUDA_curand_LIBRARY}
  )
endif()

# 处理 ament 依赖
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