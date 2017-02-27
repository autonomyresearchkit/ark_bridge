############# Create a simple LCM publisher for testing ##################
add_executable(example_send_lcm example_send_lcm.cpp)
target_link_libraries( example_send_lcm
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(example_send_lcm
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(tf_bridge tf_bridge.cpp)
target_link_libraries( tf_bridge
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(tf_bridge
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
###########################################################################
add_executable(odom_bridge odom_bridge.cpp)
target_link_libraries( odom_bridge
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(odom_bridge
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
###########################################################################
add_executable(twist_bridge twist_bridge.cpp)
target_link_libraries( twist_bridge
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(twist_bridge
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

