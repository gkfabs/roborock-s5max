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
endif()

ament_package()
