#ifndef _ROKISPHERESHAPE_HH_
#define _ROKISPHERESHAPE_HH_

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/SphereShape.hh"
#include "gazebo/physics/roki/RokiTypes.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class GZ_PHYSICS_VISIBLE RokiSphereShape : public SphereShape
    {
      public: explicit RokiSphereShape(RokiCollisionPtr _parent);
      public: virtual ~RokiSphereShape();
      public: virtual void SetRadius(double _radius);
    };
  }
}
#endif
