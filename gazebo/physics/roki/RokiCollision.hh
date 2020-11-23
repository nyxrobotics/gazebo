#ifndef _ROKICOLLISION_HH_
#define _ROKICOLLISION_HH_

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/roki/RokiTypes.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/util/system.hh"

// for "gazebo::physics::Shape" in RokiCollision::Load
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Contact.hh"
#include "gazebo/physics/Shape.hh"


#include <gazebo/ode/common.h>

#include "roki/rk_link.h"

namespace gazebo
{
  namespace physics
  {
    class GZ_PHYSICS_VISIBLE RokiCollision : public Collision
    {
      public: explicit RokiCollision(LinkPtr _parent);
      public: virtual ~RokiCollision();
      public: virtual void Load(sdf::ElementPtr _sdf);

      public: virtual void OnPoseChange();
      public: virtual void SetCategoryBits(unsigned int _bits);
      public: virtual void SetCollideBits(unsigned int _bits);

      public: virtual unsigned int GetCategoryBits() const;
      public: virtual unsigned int GetCollideBits() const;

      public: virtual ignition::math::Box BoundingBox() const;

      public: RokiSurfaceParamsPtr GetRokiSurface() const;

      public:
        std::string GetPathName() const;
        zShape3D* shape_;
      private: unsigned int categoryBits;
      private: unsigned int collideBits;
    };
  }
}
#endif
