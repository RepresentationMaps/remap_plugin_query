cmake_minimum_required(VERSION 3.8)
project(remap_plugin_query)

add_compile_options(-fPIC)

# Global flags
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_MODULE_PATH "${FIND_OPENVDB_PATH}")

find_package(ament_cmake_auto REQUIRED)
find_package(yaml-cpp REQUIRED)
ament_auto_find_build_dependencies()

# Targets
add_library(remap_plugin_query SHARED
src/remap_plugin_query/plugin_query.cpp)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${nlohmann-json_LIBRARIES}
  yaml-cpp
)

pluginlib_export_plugin_description_file(remap_plugin_base plugins.xml)

target_include_directories(remap_plugin_query PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  remap_plugin_query
  kb_msgs
  pcl_ros
  pluginlib
  rclcpp
  remap_map_handler
  remap_msgs
  remap_plugin_base
  remap_regions_register
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  remap_plugin_query
  yaml-cpp
)

ament_export_targets(
  export_${PROJECT_NAME}
)

install(
  TARGETS remap_plugin_query
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

# ament_index_register_resource("remap.plugin_configuration" CONTENT queries/kitchen_query.yml)
set(TARGET_DIRECTORY ${PROJECT_SOURCE_DIR}/queries)

file(GLOB QUERIES ${TARGET_DIRECTORY}/*)

file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/queries_tmp "")

foreach(QUERY_PATH ${QUERIES})
  file(SIZE ${CMAKE_CURRENT_BINARY_DIR}/queries_tmp FILE_SIZE)
  if(FILE_SIZE GREATER 0)
    file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/queries_tmp ";")
  endif()
  string(REGEX REPLACE ".*/([^/]+/[^/]+)$" "\\1" QUERY_PATH_RELATIVE "${QUERY_PATH}")
  file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/queries_tmp ${QUERY_PATH_RELATIVE})
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/queries_tmp DESTINATION share/ament_index/resource_index/remap.plugin_configuration/ RENAME ${PROJECT_NAME})
endforeach()
ament_index_register_resource("pal_configuration.remap_manager" CONTENT config/00-query.yml)

ament_auto_package(INSTALL_TO_SHARE config queries)
