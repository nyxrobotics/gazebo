#include "gazebo/common/Console.hh"
#include "gazebo/physics/roki/RokiPhysics.hh"
#include "gazebo/physics/roki/RokiCollision.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/physics/roki/RokiPlaneShape.hh"
#include "gazebo/physics/roki/RokiUtils.hh"
#include "gazebo/util/system.hh"


using namespace gazebo;
using namespace physics;

RokiPlaneShape::RokiPlaneShape(CollisionPtr _parent) : PlaneShape(_parent)
{
  DEBUG_PRINT("RokiPlaneShape::RokiPlaneShape()\n");
}

RokiPlaneShape::~RokiPlaneShape()
{
  DEBUG_PRINT("RokiPlaneShape::~RokiPlaneShape()\n");
}

void RokiPlaneShape::CreatePlane()
{
  RokiCollisionPtr collision = boost::dynamic_pointer_cast<RokiCollision>(this->collisionParent);
  RokiLinkPtr link = boost::dynamic_pointer_cast<RokiLink>(this->collisionParent->GetLink());

  DEBUG_PRINT("RokiBoxShape::CreatePlane() : link=%s, name=%s\n", link->GetName().c_str(), GetName().c_str());

  zShape3D *shape = zAlloc(zShape3D, 1);
  zShape3DInit(shape);
  zVec3D zp = {{ 0, 0, -1 }};
  zVec3D ax = {{ 1, 0, 0 }};
  zVec3D ay = {{ 0, 1, 0 }};
  zVec3D az = {{ 0, 0, 1 }};
  zShape3DCreateBox(shape, &zp, &ax, &ay, &az, 2000, 2000, 2);
  zBox3DInit(zShape3DBB(shape));

  zNameSet(shape, const_cast<char*>(link->GetPathName().c_str()));

  collision->shape_ = shape;
}

void RokiPlaneShape::SetAltitude(const ignition::math::Vector3d &_pos)
{
  PlaneShape::SetAltitude(_pos);
}
