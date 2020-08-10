#ifndef _ROKICYLINDERSHAPE_HH_
#define _ROKICYLINDERSHAPE_HH_

#include "gazebo/physics/CylinderShape.hh"
#include "gazebo/physics/roki/RokiPhysics.hh"
#include "gazebo/physics/roki/RokiTypes.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class GZ_PHYSICS_VISIBLE RokiCylinderShape : public CylinderShape
    {
      public: explicit RokiCylinderShape(CollisionPtr _parent);
      public: virtual ~RokiCylinderShape();
      public: void SetSize(double _radius, double _length);
    };
  }
}
#endif
