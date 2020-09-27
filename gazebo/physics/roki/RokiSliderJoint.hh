#ifndef _ROKISLIDERJOINT_HH_
#define _ROKISLIDERJOINT_HH_

#include "ignition/math/Angle.hh"
#include "ignition/math/Vector3.hh"

#include "gazebo/util/system.hh"

#include "gazebo/physics/SliderJoint.hh"
#include "gazebo/physics/roki/RokiJoint.hh"

namespace gazebo
{
  namespace physics
  {
    class GZ_PHYSICS_VISIBLE RokiSliderJoint : public SliderJoint<RokiJoint>
    {
      public: RokiSliderJoint(BasePtr _parent);
      public: virtual ~RokiSliderJoint();
      public: virtual void Load(sdf::ElementPtr _sdf);
      public: virtual void Init();
      public: virtual math::Angle GetAngleImpl(unsigned int _index) const;
      public: virtual void SetVelocity(unsigned int _index, double _vel);
      public: virtual double GetVelocity(unsigned int _index) const;
      protected: virtual void SetForceImpl(unsigned int _index, double _effort);
      protected: virtual double GetForceImpl(unsigned int _index);

      public:
        virtual void SetFriction(const unsigned int _index, const double value);

      public:
        rkJointPrpPrism *prp_;
    };
  }
}
#endif
