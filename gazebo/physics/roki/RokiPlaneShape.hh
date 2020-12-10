#ifndef _ROKIPLANESHAPE_HH_
#define _ROKIPLANESHAPE_HH_

#include "gazebo/physics/PlaneShape.hh"
#include "gazebo/physics/roki/RokiPhysics.hh"
#include "gazebo/physics/roki/RokiTypes.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class GZ_PHYSICS_VISIBLE RokiPlaneShape : public PlaneShape
    {
      public: explicit RokiPlaneShape(CollisionPtr _parent);
      public: virtual ~RokiPlaneShape();
      public: virtual void CreatePlane();
      public: virtual void SetAltitude(const math::Vector3 &_pos);
    };
  }
}
#endif
