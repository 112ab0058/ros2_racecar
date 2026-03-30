cmake_minimum_required(VERSION 3.5)
project(polygon_plugins)

find_package(ament_cmake REQUIRED)
find_package(polygon_base REQUIRED)
find_package(pluginlib REQUIRED)

add_library(polygon_plugins SHARED src/polygon_plugins.cpp)

ament_target_dependencies(
  polygon_plugins
  polygon_base
  pluginlib
)

# 關鍵：匯出插件描述檔
pluginlib_export_plugin_description_file(polygon_base plugins.xml)

# 安裝設定（加入 EXPORT 標記）
install(
  TARGETS polygon_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 關鍵：匯出庫與目標
ament_export_libraries(polygon_plugins)
ament_export_targets(export_${PROJECT_NAME})

ament_package()