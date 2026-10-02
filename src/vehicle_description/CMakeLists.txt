cmake_minimum_required(VERSION 3.8)
project(vehicle_description)

find_package(ament_cmake REQUIRED)

find_program(XACRO_EXE NAMES xacro)
if(NOT XACRO_EXE)
  message(FATAL_ERROR "xacro not found; install ros-${ROS_DISTRO}-xacro")
endif()

set(XACRO_SRC ${CMAKE_CURRENT_SOURCE_DIR}/urdf/vehicle.urdf.xacro)
set(URDF_GEN ${CMAKE_CURRENT_BINARY_DIR}/vehicle.urdf)

add_custom_command(
  OUTPUT ${URDF_GEN}
  COMMAND ${XACRO_EXE} ${XACRO_SRC} -o ${URDF_GEN}
  DEPENDS ${XACRO_SRC}
  COMMENT "Generating vehicle.urdf from xacro"
)

add_custom_target(generate_vehicle_urdf ALL DEPENDS ${URDF_GEN})

install(DIRECTORY urdf worlds
  DESTINATION share/${PROJECT_NAME}
)
install(FILES ${URDF_GEN}
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
