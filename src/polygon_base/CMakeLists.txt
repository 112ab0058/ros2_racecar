cmake_minimum_required(VERSION 3.5)
project(polygon_base)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 尋找依賴
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

# 定義執行檔（雖然目前還沒寫好 area_node.cpp，但先留著）
add_executable(area_node src/area_node.cpp)
target_include_directories(area_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(area_node pluginlib)

# --- 關鍵部分：匯出標頭檔 ---
install(
  DIRECTORY include/
  DESTINATION include)

ament_export_include_directories(include)
# --------------------------------------

install(TARGETS area_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()