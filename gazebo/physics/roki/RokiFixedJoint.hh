#ifndef _ROKIFIXEDJOINT_HH_
#define _ROKIFIXEDJOINT_HH_

#include "ignition/math/Angle.hh"
#include "ignition/math/Vector3.hh"

#include "gazebo/util/system.hh"

#include "gazebo/physics/FixedJoint.hh"
#include "gazebo/physics/roki/RokiJoint.hh"

namespace gazebo
{
  namespace physics
  {
    class GZ_PHYSICS_VISIBLE RokiFixedJoint : public FixedJoint<RokiJoint>
    {
      public: explicit RokiFixedJoint(BasePtr _parent);
      public: virtual ~RokiFixedJoint();
      public: virtual void Load(sdf::ElementPtr _sdf);
      public: virtual void Init();
      public: virtual double PositionImpl(const unsigned int _index) const;
      public: virtual void SetVelocity(unsigned int _index, double _vel);
      public: virtual double GetVelocity(unsigned int _index) const;
      protected: virtual void SetForceImpl(unsigned int _index, double _effort);
      protected: virtual double GetForceImpl(unsigned int _index);
    };
  }
}
#endif
