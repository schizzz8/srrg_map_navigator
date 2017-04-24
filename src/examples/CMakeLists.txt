add_executable(srrg_map_navigator_app srrg_map_navigator_app.cpp)
target_link_libraries(srrg_map_navigator_app 
  srrg_map_navigator_viewers_library
  ${catkin_LIBRARIES}
)
