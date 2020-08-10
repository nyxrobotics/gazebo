#ifndef _ROKIJOINT_HH_
#define _ROKIJOINT_HH_

#include <boost/any.hpp>
#include <string>

#include "gazebo/common/Exception.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/util/system.hh"

#include "gazebo/physics/roki/RokiTypes.hh"

#include <map>

#include "roki/rk_fd.h"
#include "roki/rk_link.h"
#include "roki/rk_joint.h"

namespace gazebo
{
  namespace physics
  {
    class GZ_PHYSICS_VISIBLE RokiJoint : public Joint
    {
      public: RokiJoint(BasePtr _parent);
      public: virtual ~RokiJoint();
      public: virtual void Load(sdf::ElementPtr _sdf);
      public: virtual void Init();
      public: virtual void Reset();
      public: virtual LinkPtr GetJointLink(unsigned int _index) const;
      public: virtual bool AreConnected(LinkPtr _one, LinkPtr _two) const;
      public: virtual void Attach(LinkPtr _parent, LinkPtr _child);
      public: virtual void Detach();
      public: virtual math::Vector3 GetAnchor(unsigned int _index) const;
      public: virtual void SetAnchor(unsigned int _index, const gazebo::math::Vector3 & _anchor);
      public: virtual math::Vector3 GetAxis(unsigned int _index) const;
      public: virtual math::Vector3 GetGlobalAxis(unsigned int _index) const;
      public: virtual void SetAxis(unsigned int _index, const math::Vector3& _axis);
      public: virtual bool SetHighStop(unsigned int _index, const math::Angle &_angle);
      public: virtual bool SetLowStop(unsigned int _index, const math::Angle &_angle);
      public: virtual math::Angle GetHighStop(unsigned int _index);
      public: virtual math::Angle GetLowStop(unsigned int _index);
      public: virtual void SetDamping(unsigned int _index, double _damping);
      public: virtual void SetStiffness(unsigned int _index, const double _stiffness);
      public: virtual void SetStiffnessDamping(unsigned int _index, double _stiffness, double _damping, double _reference = 0);
      public: virtual void SetMaxForce(unsigned int _index, double _force);
      public: virtual double GetMaxForce(unsigned int _index);
      public: virtual math::Vector3 GetLinkForce(unsigned int _index) const;
      public: virtual math::Vector3 GetLinkTorque(unsigned int _index) const;
      public: virtual bool SetParam(const std::string &_key, unsigned int _index, const boost::any &_value);
      public: virtual double GetParam(const std::string &_key, unsigned int _index);
      public: virtual JointWrench GetForceTorque(unsigned int _index);
      public: virtual void SetForce(unsigned int _index, double _force);
      public: virtual void SetForceImpl(unsigned int _index, double _force) = 0;
      public: virtual double GetForce(unsigned int _index);
      public: virtual double GetForceImpl(unsigned int _index) = 0;
      public: virtual unsigned int GetAngleCount() const;
      public: virtual void ApplyDamping();

      public:
        RokiModelPtr GetRokiModel() const;
        rkFDCell* GetRkFDCell() const;
        rkChain*  GetRkChain() const;

        RokiLinkPtr GetParentRokiLink() const;
        RokiLinkPtr GetChildRokiLink() const;

        virtual void SetFriction(const unsigned int _index, const double value);

      public:
        math::Vector3 anchor_pos_;
        math::Vector3 anchor_axis_;
        rkJoint *rkjoint_;
        rkMotor *rkmotor_;
        math::Pose org2anchor_pose_;

        math::Angle highStop_[MAX_JOINT_AXIS];
        math::Angle lowStop_[MAX_JOINT_AXIS];
        double friction_[MAX_JOINT_AXIS];

        std::map<std::string, boost::any> params_;
    };
  }
}
#endif
