cmake_minimum_required(VERSION 3.8)
project(roborock_s5max)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/bumper_adapter.py
    scripts/cleaning_actuator_manager.py
    scripts/contact_reflex.py
    scripts/dock_beacon_adapter.py
    scripts/dock_final_approach.py
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
    dock_beacon_adapter_test
    test/test_dock_beacon_adapter.py
  )
  ament_add_pytest_test(
    dock_final_approach_test
    test/test_dock_final_approach.py
  )
  ament_add_pytest_test(
    dock_stage_test
    test/test_dock_stage.py
  )
endif()

ament_package()
