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

###########################################################################
add_executable(bool_bridge bool_bridge.cpp)
target_link_libraries( bool_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(bool_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(compressedmapdata_bridge compressedmapdata_bridge.cpp)
target_link_libraries( compressedmapdata_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(compressedmapdata_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(controlselectionstate_bridge controlselectionstate_bridge.cpp)
target_link_libraries( controlselectionstate_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(controlselectionstate_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(distancetogoal_bridge distancetogoal_bridge.cpp)
target_link_libraries( distancetogoal_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(distancetogoal_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(float64_bridge float64_bridge.cpp)
target_link_libraries( float64_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(float64_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(geometryarray_bridge geometryarray_bridge.cpp)
target_link_libraries( geometryarray_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(geometryarray_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(goalstatusarray_bridge goalstatusarray_bridge.cpp)
target_link_libraries( goalstatusarray_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(goalstatusarray_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(laserscan_bridge laserscan_bridge.cpp)
target_link_libraries( laserscan_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(laserscan_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(occupancygrid_bridge occupancygrid_bridge.cpp)
target_link_libraries( occupancygrid_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(occupancygrid_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(path_bridge path_bridge.cpp)
target_link_libraries( path_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(path_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(pausestatus_bridge pausestatus_bridge.cpp)
target_link_libraries( pausestatus_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(pausestatus_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(posearray_bridge posearray_bridge.cpp)
target_link_libraries( posearray_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(posearray_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(posestamped_bridge posestamped_bridge.cpp)
target_link_libraries( posestamped_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(posestamped_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(posewithcovariancestamped_bridge posewithcovariancestamped_bridge.cpp)
target_link_libraries( posewithcovariancestamped_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(posewithcovariancestamped_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(string_bridge string_bridge.cpp)
target_link_libraries( string_bridge
   ${LCM_LIBRARY}
   ${catkin_LIBRARIES}
)
add_dependencies(string_bridge
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
)

##########################################################################
##########################################################################
##########################################################################
###########################################################################
add_executable(empty_service empty_service.cpp)
target_link_libraries( empty_service
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(empty_service
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(string_service string_service.cpp)
target_link_libraries( string_service
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(string_service
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(setcontrolmode_service setcontrolmode_service.cpp)
target_link_libraries( setcontrolmode_service
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(setcontrolmode_service
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(setslammode_service setslammode_service.cpp)
target_link_libraries( setslammode_service
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(setslammode_service
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(loadmapfromdisk_service loadmapfromdisk_service.cpp)
target_link_libraries( loadmapfromdisk_service
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(loadmapfromdisk_service
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(deletemap_service deletemap_service.cpp)
target_link_libraries( deletemap_service
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(deletemap_service
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(savemaptodisk_service savemaptodisk_service.cpp)
target_link_libraries( savemaptodisk_service
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(savemaptodisk_service
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(listmaps_service listmaps_service.cpp)
target_link_libraries( listmaps_service
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(listmaps_service
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(mapinfo_service mapinfo_service.cpp)
target_link_libraries( mapinfo_service
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(mapinfo_service
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(pausecontrol_service pausecontrol_service.cpp)
target_link_libraries( pausecontrol_service
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(pausecontrol_service
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(arkconfigsettings_service arkconfigsettings_service.cpp)
target_link_libraries( arkconfigsettings_service
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(arkconfigsettings_service
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

###########################################################################
add_executable(setpose_service setpose_service.cpp)
target_link_libraries( setpose_service
    ${LCM_LIBRARY}
    ${catkin_LIBRARIES}
)
add_dependencies(setpose_service
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
