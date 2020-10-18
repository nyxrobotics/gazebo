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
      public: explicit RokiJoint(BasePtr _parent);
      public: virtual ~RokiJoint();
      public: virtual void Load(sdf::ElementPtr _sdf) override;
      public: virtual void Init() override;
      public: virtual void Reset() override;
      public: virtual LinkPtr GetJointLink(unsigned int _index) const override;
      public: virtual bool AreConnected(LinkPtr _one, LinkPtr _two) const override;
      public: virtual void Attach(LinkPtr _parent, LinkPtr _child) override;
      public: virtual void Detach() override;
      public: virtual ignition::math::Vector3d Anchor(const unsigned int _index) const override;
      public: virtual void SetAnchor(const unsigned int _index, const ignition::math::Vector3d &_anchor) override;
      // public: virtual ignition::math::Vector3d LocalAxis(unsigned int _index) const override;
      public: virtual ignition::math::Vector3d GlobalAxis(unsigned int _index) const override;
      public: virtual void SetAxis(const unsigned int _index, const ignition::math::Vector3d &_axis) override;
      public: virtual void SetUpperLimit(const unsigned int _index, const double _limit) override;
      public: virtual void SetLowerLimit(const unsigned int _index, const double _limit) override;
      public: virtual double UpperLimit(const unsigned int _index) const override;
      public: virtual double LowerLimit(const unsigned int _index) const override;
      public: virtual void SetDamping(unsigned int _index, double _damping) override;
      public: virtual void SetStiffness(unsigned int _index, const double _stiffness) override;
      public: virtual void SetStiffnessDamping(unsigned int _index, double _stiffness, double _damping, double _reference = 0) override;
      // public: virtual void SetMaxForce(unsigned int _index, double _force);
      // public: virtual double GetMaxForce(unsigned int _index);
      public: virtual ignition::math::Vector3d LinkForce(const unsigned int _index) const override;
      public: virtual ignition::math::Vector3d LinkTorque(const unsigned int _index) const override;
      public: virtual bool SetParam(const std::string &_key, unsigned int _index, const boost::any &_value) override;
      public: virtual double GetParam(const std::string &_key, unsigned int _index) override;
      public: virtual JointWrench GetForceTorque(unsigned int _index) override;
      public: virtual void SetForce(unsigned int _index, double _force) override;
      public: virtual void SetForceImpl(unsigned int _index, double _force) = 0;
      public: virtual double GetForce(unsigned int _index) override;
      public: virtual double GetForceImpl(unsigned int _index) = 0;
      public: virtual unsigned int DOF() const override;
      public: virtual void ApplyStiffnessDamping() override;
      // ------------------Additional Functions------------------
      // public: virtual void CacheForceTorque() override;
      // public: virtual bool SetPosition( const unsigned int _index, const double _position, const bool _preserveWorldVelocity = false) override;
      // public: virtual void SetProvideFeedback(bool _enable) override;
      // private: void SaveForce(unsigned int _index, double _force);

      public:
        RokiModelPtr GetRokiModel() const;
        rkFDCell* GetRkFDCell() const;
        rkChain*  GetRkChain() const;

        RokiLinkPtr GetParentRokiLink() const;
        RokiLinkPtr GetChildRokiLink() const;

        virtual void SetFriction(const unsigned int _index, const double value);

      public:
        ignition::math::Vector3d anchor_pos_;
        ignition::math::Vector3d anchor_axis_;
        rkJoint *rkjoint_;
        rkMotor *rkmotor_;
        ignition::math::Pose3d org2anchor_pose_;

        double highStop_[MAX_JOINT_AXIS];
        double lowStop_[MAX_JOINT_AXIS];
        double friction_[MAX_JOINT_AXIS];

        std::map<std::string, boost::any> params_;
    };
  }
}
#endif
