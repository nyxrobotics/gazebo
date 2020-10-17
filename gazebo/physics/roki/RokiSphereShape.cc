#include "gazebo/common/Console.hh"
#include "gazebo/physics/roki/RokiPhysics.hh"
#include "gazebo/physics/roki/RokiCollision.hh"
#include "gazebo/physics/roki/RokiSphereShape.hh"
#include "gazebo/physics/roki/RokiUtils.hh"
#include "gazebo/util/system.hh"

using namespace gazebo;
using namespace physics;

RokiSphereShape::RokiSphereShape(RokiCollisionPtr _parent)
  : SphereShape(_parent)
{
  DEBUG_PRINT("RokiSphereShape::RokiSphereShape()\n");
}

RokiSphereShape::~RokiSphereShape()
{
  DEBUG_PRINT("RokiSphereShape::~RokiSphereShape()\n");
}

void RokiSphereShape::SetRadius(double _radius)
{
  RokiCollisionPtr collision = boost::dynamic_pointer_cast<RokiCollision>(this->collisionParent);
  RokiLinkPtr link = boost::dynamic_pointer_cast<RokiLink>(this->collisionParent->GetLink());

  DEBUG_PRINT("RokiSphereShape::SetRadius() : link=%s, name=%s, radius=(%f)\n", link->GetName().c_str(), GetName().c_str(), _radius);

  if (_radius <= 0)
  {
    gzerr << "Sphere shape does not support negative radius.\n";
    return;
  }

  SphereShape::SetRadius(_radius);

  zShape3D* shape = zAlloc(zShape3D, 1);
  zShape3DInit(shape);
  zVec3D zp = {{ 0, 0, 0 }};
  zShape3DCreateSphere(shape, &zp, _radius, 8);
  zBox3DInit(zShape3DBB(shape));

  zNameSet(shape, const_cast<char*>(link->GetPathName().c_str()));

  collision->shape_ = shape;
}
