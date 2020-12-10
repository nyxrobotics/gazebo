include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ${sources}
  roki/RokiModel.cc
  roki/RokiCollision.cc
  roki/RokiLink.cc
  roki/RokiPhysics.cc
  roki/RokiUtils.cc
  roki/RokiBoxShape.cc
  roki/RokiSphereShape.cc
  roki/RokiMeshShape.cc
  roki/RokiCylinderShape.cc
  roki/RokiPlaneShape.cc
  roki/RokiJoint.cc
  roki/RokiHingeJoint.cc
  roki/RokiSliderJoint.cc
  roki/RokiFixedJoint.cc
  roki/RokiSurfaceParams.cc
  roki/rk_motor_pseudo.cc
  PARENT_SCOPE
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

# gz_add_library(gazebo_physics_roki ${headers} ${sources})
# target_link_libraries(gazebo_physics_roki
#   gazebo_opcode
#   gazebo_common
#   gazebo_math
#   roki
#   zeo
#   zm
#   cure
#   m
# )
#gz_install_library(gazebo_physics_roki)
#gz_install_includes("physics/roki" ${headers})

# unit tests
set (gtest_sources
  RokiPhysics_TEST.cc
)

gz_build_tests(${gtest_sources}
  EXTRA_LIBS gazebo_physics gazebo_test_fixture)

gz_install_includes("physics/roki" ${headers})
