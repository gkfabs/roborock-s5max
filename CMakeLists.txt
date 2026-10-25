cmake_minimum_required(VERSION 3.8)
project(roborock_s5max)

find_package(ament_cmake REQUIRED)
find_package(nav2_util REQUIRED)
find_package(opennav_docking REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vacuum_ros2_bridge REQUIRED)
find_library(
  SIMPLE_CHARGING_DOCK_LIBRARY
  NAMES simple_charging_dock
  REQUIRED
)

add_library(s5max_charging_dock SHARED
  src/s5max_charging_dock.cpp
)
target_include_directories(s5max_charging_dock PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(s5max_charging_dock
  nav2_util
  opennav_docking
  opennav_docking_core
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  vacuum_ros2_bridge
)
target_link_libraries(s5max_charging_dock
  ${SIMPLE_CHARGING_DOCK_LIBRARY}
)
pluginlib_export_plugin_description_file(
  opennav_docking_core s5max_dock_plugins.xml)

install(
  TARGETS s5max_charging_dock
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/bumper_adapter.py
    scripts/cleaning_actuator_manager.py
    scripts/contact_reflex.py
    scripts/dock_beacon_adapter.py
    scripts/cancel_docking.py
    scripts/dock_pose_estimator.py
    scripts/dock_robot.py
    scripts/dock_scan_fault_adapter.py
    scripts/dock_sim_adapter.py
    scripts/dock_stage.py
    scripts/localization_pose_adapter.py
    scripts/safety_adapter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    contact_reflex_test
    test/test_contact_reflex.py
  )
  ament_add_pytest_test(
    safety_adapter_test
    test/test_safety_adapter.py
  )
  ament_add_pytest_test(
    cleaning_actuator_manager_test
    test/test_cleaning_actuator_manager.py
  )
  ament_add_pytest_test(
    dock_beacon_adapter_test
    test/test_dock_beacon_adapter.py
  )
  ament_add_pytest_test(
    dock_pose_estimator_test
    test/test_dock_pose_estimator.py
  )
  ament_add_pytest_test(
    dock_robot_test
    test/test_dock_robot.py
  )
  ament_add_pytest_test(
    cancel_docking_test
    test/test_cancel_docking.py
  )
  ament_add_pytest_test(
    dock_sim_adapter_test
    test/test_dock_sim_adapter.py
  )
  ament_add_pytest_test(
    dock_scan_fault_adapter_test
    test/test_dock_scan_fault_adapter.py
  )
  ament_add_pytest_test(
    dock_stage_test
    test/test_dock_stage.py
  )
endif()

ament_export_dependencies(opennav_docking_core)
ament_export_include_directories(include)
ament_export_libraries(s5max_charging_dock)
ament_package()
