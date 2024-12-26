find_package( OpenCV REQUIRED )

add_library(gp_planner_sdf)
add_library(gp_planner::sdf ALIAS gp_planner_sdf)

target_sources(gp_planner_sdf PRIVATE
    signed_distance_field_2d.cc
    grid_map_2d.cc
)
target_link_libraries(gp_planner_sdf PUBLIC ${OpenCV_LIBS} 
                         planning_core_common)

target_include_directories(gp_planner_sdf
  PUBLIC
    ${CMAKE_SOURCE_DIR}  # 添加这一行，使其能找到planning_core的头文件
)                         