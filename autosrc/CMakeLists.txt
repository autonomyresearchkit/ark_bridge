add_executable(tf_republisher tf_republisher.cpp)
target_link_libraries( tf_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(tf_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(tf_static_republisher tf_static_republisher.cpp)
target_link_libraries( tf_static_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(tf_static_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(platform_control_cmd_vel_republisher platform_control_cmd_vel_republisher.cpp)
target_link_libraries( platform_control_cmd_vel_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(platform_control_cmd_vel_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(platform_control_odom_republisher platform_control_odom_republisher.cpp)
target_link_libraries( platform_control_odom_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(platform_control_odom_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(platform_odometry_republisher platform_odometry_republisher.cpp)
target_link_libraries( platform_odometry_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(platform_odometry_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(clock_republisher clock_republisher.cpp)
target_link_libraries( clock_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(clock_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(strategy_manager_path_complete_republisher strategy_manager_path_complete_republisher.cpp)
target_link_libraries( strategy_manager_path_complete_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(strategy_manager_path_complete_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(strategy_manager_pause_status_republisher strategy_manager_pause_status_republisher.cpp)
target_link_libraries( strategy_manager_pause_status_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(strategy_manager_pause_status_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(strategy_manager_simple_goal_republisher strategy_manager_simple_goal_republisher.cpp)
target_link_libraries( strategy_manager_simple_goal_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(strategy_manager_simple_goal_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(strategy_manager_simple_path_republisher strategy_manager_simple_path_republisher.cpp)
target_link_libraries( strategy_manager_simple_path_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(strategy_manager_simple_path_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(strategy_manager_distance_to_goal_republisher strategy_manager_distance_to_goal_republisher.cpp)
target_link_libraries( strategy_manager_distance_to_goal_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(strategy_manager_distance_to_goal_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(slam_localization_score_republisher slam_localization_score_republisher.cpp)
target_link_libraries( slam_localization_score_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(slam_localization_score_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_features_zones_exclusion_republisher map_data_features_zones_exclusion_republisher.cpp)
target_link_libraries( map_data_features_zones_exclusion_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_features_zones_exclusion_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_features_markers_waypoint_republisher map_data_features_markers_waypoint_republisher.cpp)
target_link_libraries( map_data_features_markers_waypoint_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_features_markers_waypoint_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_saved_map_data_republisher map_data_saved_map_data_republisher.cpp)
target_link_libraries( map_data_saved_map_data_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_saved_map_data_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_map_republisher map_data_map_republisher.cpp)
target_link_libraries( map_data_map_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_map_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_md5sum_republisher map_data_md5sum_republisher.cpp)
target_link_libraries( map_data_md5sum_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_md5sum_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_loaded_map_filename_republisher map_data_loaded_map_filename_republisher.cpp)
target_link_libraries( map_data_loaded_map_filename_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_loaded_map_filename_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(e_stop_republisher e_stop_republisher.cpp)
target_link_libraries( e_stop_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(e_stop_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_control_state_republisher control_selection_control_state_republisher.cpp)
target_link_libraries( control_selection_control_state_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_control_state_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(path_republisher path_republisher.cpp)
target_link_libraries( path_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(path_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(move_base_status_republisher move_base_status_republisher.cpp)
target_link_libraries( move_base_status_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(move_base_status_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(slam_estimated_pose_republisher slam_estimated_pose_republisher.cpp)
target_link_libraries( slam_estimated_pose_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(slam_estimated_pose_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(clock_echo_republisher clock_echo_republisher.cpp)
target_link_libraries( clock_echo_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(clock_echo_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(rear_scan_republisher rear_scan_republisher.cpp)
target_link_libraries( rear_scan_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(rear_scan_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(front_scan_republisher front_scan_republisher.cpp)
target_link_libraries( front_scan_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(front_scan_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_autonomy_clear_all_disabled_call_republisher control_selection_autonomy_clear_all_disabled_call_republisher.cpp)
target_link_libraries( control_selection_autonomy_clear_all_disabled_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_autonomy_clear_all_disabled_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_autonomy_clear_all_paused_call_republisher control_selection_autonomy_clear_all_paused_call_republisher.cpp)
target_link_libraries( control_selection_autonomy_clear_all_paused_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_autonomy_clear_all_paused_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_autonomy_disable_call_republisher control_selection_autonomy_disable_call_republisher.cpp)
target_link_libraries( control_selection_autonomy_disable_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_autonomy_disable_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_autonomy_enable_call_republisher control_selection_autonomy_enable_call_republisher.cpp)
target_link_libraries( control_selection_autonomy_enable_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_autonomy_enable_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_autonomy_pause_call_republisher control_selection_autonomy_pause_call_republisher.cpp)
target_link_libraries( control_selection_autonomy_pause_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_autonomy_pause_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_autonomy_resume_call_republisher control_selection_autonomy_resume_call_republisher.cpp)
target_link_libraries( control_selection_autonomy_resume_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_autonomy_resume_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_core_clear_all_disabled_call_republisher control_selection_core_clear_all_disabled_call_republisher.cpp)
target_link_libraries( control_selection_core_clear_all_disabled_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_core_clear_all_disabled_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_core_clear_all_paused_call_republisher control_selection_core_clear_all_paused_call_republisher.cpp)
target_link_libraries( control_selection_core_clear_all_paused_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_core_clear_all_paused_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_core_disable_call_republisher control_selection_core_disable_call_republisher.cpp)
target_link_libraries( control_selection_core_disable_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_core_disable_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_core_enable_call_republisher control_selection_core_enable_call_republisher.cpp)
target_link_libraries( control_selection_core_enable_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_core_enable_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_core_pause_call_republisher control_selection_core_pause_call_republisher.cpp)
target_link_libraries( control_selection_core_pause_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_core_pause_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_core_resume_call_republisher control_selection_core_resume_call_republisher.cpp)
target_link_libraries( control_selection_core_resume_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_core_resume_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_set_mode_call_republisher control_selection_set_mode_call_republisher.cpp)
target_link_libraries( control_selection_set_mode_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_set_mode_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_clear_call_republisher map_data_clear_call_republisher.cpp)
target_link_libraries( map_data_clear_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_clear_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_delete_map_call_republisher map_data_delete_map_call_republisher.cpp)
target_link_libraries( map_data_delete_map_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_delete_map_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_get_current_map_call_republisher map_data_get_current_map_call_republisher.cpp)
target_link_libraries( map_data_get_current_map_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_get_current_map_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_get_map_info_call_republisher map_data_get_map_info_call_republisher.cpp)
target_link_libraries( map_data_get_map_info_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_get_map_info_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_list_maps_call_republisher map_data_list_maps_call_republisher.cpp)
target_link_libraries( map_data_list_maps_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_list_maps_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_load_map_from_disk_call_republisher map_data_load_map_from_disk_call_republisher.cpp)
target_link_libraries( map_data_load_map_from_disk_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_load_map_from_disk_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_save_map_to_disk_call_republisher map_data_save_map_to_disk_call_republisher.cpp)
target_link_libraries( map_data_save_map_to_disk_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_save_map_to_disk_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(mode_manager_set_mode_call_republisher mode_manager_set_mode_call_republisher.cpp)
target_link_libraries( mode_manager_set_mode_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(mode_manager_set_mode_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(slam_set_initial_pose_call_republisher slam_set_initial_pose_call_republisher.cpp)
target_link_libraries( slam_set_initial_pose_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(slam_set_initial_pose_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(strategy_manager_abort_path_call_republisher strategy_manager_abort_path_call_republisher.cpp)
target_link_libraries( strategy_manager_abort_path_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(strategy_manager_abort_path_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(strategy_manager_pause_call_republisher strategy_manager_pause_call_republisher.cpp)
target_link_libraries( strategy_manager_pause_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(strategy_manager_pause_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(strategy_manager_resume_call_republisher strategy_manager_resume_call_republisher.cpp)
target_link_libraries( strategy_manager_resume_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(strategy_manager_resume_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(ark_config_settings_call_republisher ark_config_settings_call_republisher.cpp)
target_link_libraries( ark_config_settings_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(ark_config_settings_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(ark_start_call_republisher ark_start_call_republisher.cpp)
target_link_libraries( ark_start_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(ark_start_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(ark_stop_call_republisher ark_stop_call_republisher.cpp)
target_link_libraries( ark_stop_call_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(ark_stop_call_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_autonomy_clear_all_disabled_response_republisher control_selection_autonomy_clear_all_disabled_response_republisher.cpp)
target_link_libraries( control_selection_autonomy_clear_all_disabled_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_autonomy_clear_all_disabled_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_autonomy_clear_all_paused_response_republisher control_selection_autonomy_clear_all_paused_response_republisher.cpp)
target_link_libraries( control_selection_autonomy_clear_all_paused_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_autonomy_clear_all_paused_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_autonomy_disable_response_republisher control_selection_autonomy_disable_response_republisher.cpp)
target_link_libraries( control_selection_autonomy_disable_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_autonomy_disable_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_autonomy_enable_response_republisher control_selection_autonomy_enable_response_republisher.cpp)
target_link_libraries( control_selection_autonomy_enable_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_autonomy_enable_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_autonomy_pause_response_republisher control_selection_autonomy_pause_response_republisher.cpp)
target_link_libraries( control_selection_autonomy_pause_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_autonomy_pause_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_autonomy_resume_response_republisher control_selection_autonomy_resume_response_republisher.cpp)
target_link_libraries( control_selection_autonomy_resume_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_autonomy_resume_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_core_clear_all_disabled_response_republisher control_selection_core_clear_all_disabled_response_republisher.cpp)
target_link_libraries( control_selection_core_clear_all_disabled_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_core_clear_all_disabled_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_core_clear_all_paused_response_republisher control_selection_core_clear_all_paused_response_republisher.cpp)
target_link_libraries( control_selection_core_clear_all_paused_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_core_clear_all_paused_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_core_disable_response_republisher control_selection_core_disable_response_republisher.cpp)
target_link_libraries( control_selection_core_disable_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_core_disable_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_core_enable_response_republisher control_selection_core_enable_response_republisher.cpp)
target_link_libraries( control_selection_core_enable_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_core_enable_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_core_pause_response_republisher control_selection_core_pause_response_republisher.cpp)
target_link_libraries( control_selection_core_pause_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_core_pause_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_core_resume_response_republisher control_selection_core_resume_response_republisher.cpp)
target_link_libraries( control_selection_core_resume_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_core_resume_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(control_selection_set_mode_response_republisher control_selection_set_mode_response_republisher.cpp)
target_link_libraries( control_selection_set_mode_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(control_selection_set_mode_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_clear_response_republisher map_data_clear_response_republisher.cpp)
target_link_libraries( map_data_clear_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_clear_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_delete_map_response_republisher map_data_delete_map_response_republisher.cpp)
target_link_libraries( map_data_delete_map_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_delete_map_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_get_current_map_response_republisher map_data_get_current_map_response_republisher.cpp)
target_link_libraries( map_data_get_current_map_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_get_current_map_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_get_map_info_response_republisher map_data_get_map_info_response_republisher.cpp)
target_link_libraries( map_data_get_map_info_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_get_map_info_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_list_maps_response_republisher map_data_list_maps_response_republisher.cpp)
target_link_libraries( map_data_list_maps_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_list_maps_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_load_map_from_disk_response_republisher map_data_load_map_from_disk_response_republisher.cpp)
target_link_libraries( map_data_load_map_from_disk_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_load_map_from_disk_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(map_data_save_map_to_disk_response_republisher map_data_save_map_to_disk_response_republisher.cpp)
target_link_libraries( map_data_save_map_to_disk_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(map_data_save_map_to_disk_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(mode_manager_set_mode_response_republisher mode_manager_set_mode_response_republisher.cpp)
target_link_libraries( mode_manager_set_mode_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(mode_manager_set_mode_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(slam_set_initial_pose_response_republisher slam_set_initial_pose_response_republisher.cpp)
target_link_libraries( slam_set_initial_pose_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(slam_set_initial_pose_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(strategy_manager_abort_path_response_republisher strategy_manager_abort_path_response_republisher.cpp)
target_link_libraries( strategy_manager_abort_path_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(strategy_manager_abort_path_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(strategy_manager_pause_response_republisher strategy_manager_pause_response_republisher.cpp)
target_link_libraries( strategy_manager_pause_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(strategy_manager_pause_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(strategy_manager_resume_response_republisher strategy_manager_resume_response_republisher.cpp)
target_link_libraries( strategy_manager_resume_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(strategy_manager_resume_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(ark_config_settings_response_republisher ark_config_settings_response_republisher.cpp)
target_link_libraries( ark_config_settings_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(ark_config_settings_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(ark_start_response_republisher ark_start_response_republisher.cpp)
target_link_libraries( ark_start_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(ark_start_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


add_executable(ark_stop_response_republisher ark_stop_response_republisher.cpp)
target_link_libraries( ark_stop_response_republisher
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(ark_stop_response_republisher 
    ${${PROJECT_NAME}_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)


