target_link_libraries(level_planner_lib
    pid_lib
)

target_link_libraries(level_planner_node
level_planner_lib
${catkin_LIBRARIES}
)

target_link_libraries(test_wheel_node
level_planner_lib
${catkin_LIBRARIES}
)

target_link_libraries(test_entry_node
level_planner_lib
${catkin_LIBRARIES}
)