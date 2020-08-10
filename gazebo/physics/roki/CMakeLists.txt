include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/deps/opende/include)

link_directories(
  ${CCD_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

if (WIN32)
  add_definitions(-DBUILDING_DLL_GZ_PHYSICS_ROKI)
endif()

set (sources
  RokiModel.cc
  RokiCollision.cc
  RokiLink.cc
  RokiPhysics.cc
  RokiUtils.cc
  RokiBoxShape.cc
  RokiSphereShape.cc
  RokiMeshShape.cc
  RokiCylinderShape.cc
  RokiPlaneShape.cc
  RokiJoint.cc
  RokiHingeJoint.cc
  RokiSliderJoint.cc
  RokiFixedJoint.cc
  RokiSurfaceParams.cc
  rk_motor_pseudo.cc
)

SET (headers
  RokiModel.cc
  RokiCollision.hh
  RokiLink.hh
  RokiPhysics.hh
  RokiUtils.hh
  RokiBoxShape.hh
  RokiCylinderShape.hh
  RokiSphereShape.hh
  RokiMeshShape.hh
  RokiPlaneShape.hh
  RokiJoint.hh
  RokiHingeJoint.hh
  RokiSliderJoint.hh
  RokiFixedJoint.hh
  RokiSurfaceParams.hh
  rk_motor_pseudo.h
)

gz_add_library(gazebo_physics_roki ${headers} ${sources})
target_link_libraries(gazebo_physics_roki
  gazebo_opcode
  gazebo_common
  gazebo_math
  roki
  zeo
  zm
  cure
  m
)

gz_install_library(gazebo_physics_roki)
gz_install_includes("physics/roki" ${headers})

# unit tests
set (gtest_sources
  RokiPhysics_TEST.cc
)
gz_build_tests(${gtest_sources})
