#include "gazebo/common/Console.hh"
#include "gazebo/physics/roki/RokiPhysics.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/physics/roki/RokiCollision.hh"
#include "gazebo/physics/roki/RokiBoxShape.hh"
#include "gazebo/physics/roki/RokiUtils.hh"
#include "gazebo/util/system.hh"

using namespace gazebo;
using namespace physics;

RokiBoxShape::RokiBoxShape(RokiCollisionPtr _parent)
  : BoxShape(_parent)
{
  DEBUG_PRINT("RokiBoxShape::RokiBoxShape()\n");
}

//////////////////////////////////////////////////
RokiBoxShape::~RokiBoxShape()
{
  DEBUG_PRINT("RokiBoxShape::~RokiBoxShape()\n");
}

void RokiBoxShape::SetSize(const math::Vector3 &_size)
{
  RokiCollisionPtr collision = boost::dynamic_pointer_cast<RokiCollision>(this->collisionParent);
  RokiLinkPtr link = boost::dynamic_pointer_cast<RokiLink>(this->collisionParent->GetLink());

  DEBUG_PRINT("RokiBoxShape::SetSize() : link=%s, name=%s, size=(%s)\n", link->GetName().c_str(), GetName().c_str(), conv2str(_size));

  if (_size.x <= 0 || _size.y <= 0 || _size.z <= 0) {
    gzerr << "Box shape does not support negative size\n";
    return;
  }

  BoxShape::SetSize(_size);

  zShape3D *shape = zAlloc(zShape3D, 1);
  zShape3DInit(shape);
  zVec3D zp = {{ 0, 0, 0 }};
  zVec3D ax = {{ 1, 0, 0 }};
  zVec3D ay = {{ 0, 1, 0 }};
  zVec3D az = {{ 0, 0, 1 }};
  zShape3DCreateBox(shape, &zp, &ax, &ay, &az, _size.x, _size.y, _size.z);
  zBox3DInit(zShape3DBB(shape));

  zNameSet(shape, const_cast<char*>(link->GetPathName().c_str()));

  collision->shape_ = shape;
}
