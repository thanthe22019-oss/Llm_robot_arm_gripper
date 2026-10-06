cmake_minimum_required(VERSION 3.8)
project(ur3_workcell_description)

find_package(ament_cmake REQUIRED)
find_package(robotiq_description REQUIRED)
find_package(ur_description REQUIRED)

# The upstream UR MoveIt launch resolves calibration files relative to the
# selected description package. Reinstall the official UR3/UR3e YAML files so
# this composed description remains compatible without duplicating their data.
get_filename_component(UR_DESCRIPTION_SHARE_DIR "${ur_description_DIR}/.." ABSOLUTE)

install(
  DIRECTORY config launch rviz srdf urdf worlds
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY
    "${UR_DESCRIPTION_SHARE_DIR}/config/ur3"
    "${UR_DESCRIPTION_SHARE_DIR}/config/ur3e"
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_workcell_description test/test_workcell_description.py)
endif()

ament_package()
