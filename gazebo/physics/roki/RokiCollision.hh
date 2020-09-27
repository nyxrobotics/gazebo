#ifndef _ROKICOLLISION_HH_
#define _ROKICOLLISION_HH_

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/util/system.hh"
#include "gazebo/physics/roki/RokiTypes.hh"

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
      public: virtual void Init();
      public: virtual void Fini();

      public: virtual void OnPoseChange();
      public: virtual void SetCategoryBits(unsigned int _bits);
      public: virtual void SetCollideBits(unsigned int _bits);

      public: virtual unsigned int GetCategoryBits() const;
      public: virtual unsigned int GetCollideBits() const;

      public: virtual ignition::math::Box BoundingBox() const;

      public:
        std::string GetPathName() const;
        zShape3D* shape_;
    };
  }
}
#endif
