#include "gazebo/common/Console.hh"
#include "gazebo/physics/roki/RokiPhysics.hh"
#include "gazebo/physics/roki/RokiCollision.hh"
#include "gazebo/physics/roki/RokiCylinderShape.hh"
#include "gazebo/physics/roki/RokiUtils.hh"
#include "gazebo/util/system.hh"

using namespace gazebo;
using namespace physics;

RokiCylinderShape::RokiCylinderShape(CollisionPtr _parent)
  : CylinderShape(_parent)
{
  DEBUG_PRINT("RokiCylinderShape::RokiCylinderShape()\n");
}

RokiCylinderShape::~RokiCylinderShape()
{
  DEBUG_PRINT("RokiCylinderShape::~RokiCylinderShape()\n");
}

void RokiCylinderShape::SetSize(double _radius, double _length)
{
  RokiCollisionPtr collision = boost::dynamic_pointer_cast<RokiCollision>(this->collisionParent);
  RokiLinkPtr link = boost::dynamic_pointer_cast<RokiLink>(this->collisionParent->GetLink());

  DEBUG_PRINT("RokiCylinderShape::SetSize() : link=%s, name=%s, radius=%f, length=%f\n", link->GetName().c_str(), GetName().c_str(), _radius, _length);

  if (_radius <= 0)
  {
    gzerr << "Cylinder shape does not support negative radius\n";
    return;
  }

  if (_length <= 0)
  {
    gzerr << "Cylinder shape does not support negative length\n";
    return;
  }

  CylinderShape::SetSize(_radius, _length);

  zShape3D* shape = zAlloc(zShape3D, 1);

  zVec3D c1 = {{0, 0, _length *  0.5}};
  zVec3D c2 = {{0, 0, _length * -0.5}};

  zShape3DCreateCyl(shape, &c1, &c2, _radius, 8);
  zBox3DInit(zShape3DBB(shape));

  zNameSet(shape, const_cast<char*>(link->GetPathName().c_str()));

  collision->shape_ = shape;
}


