#ifndef _ROKIBOXSHAPE_HH_
#define _ROKIBOXSHAPE_HH_

#include "ignition/math/Vector3.hh"
#include "gazebo/physics/BoxShape.hh"
#include "gazebo/physics/roki/RokiTypes.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class GZ_PHYSICS_VISIBLE RokiBoxShape : public BoxShape
    {
      public: explicit RokiBoxShape(RokiCollisionPtr _parent);
      public: virtual ~RokiBoxShape();
      public: virtual void SetSize(const ignition::math::Vector3d &_size);
    };
  }
}
#endif
