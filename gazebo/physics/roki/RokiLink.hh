#ifndef _ROKILINK_HH_
#define _ROKILINK_HH_

#include "gazebo/physics/roki/RokiTypes.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/util/system.hh"

#include "gazebo/physics/roki/RokiTypes.hh"

#include "roki/rk_fd.h"
#include "roki/rk_link.h"

namespace gazebo
{
  namespace physics
  {
    class GZ_PHYSICS_VISIBLE RokiLink : public Link
    {
      public: explicit RokiLink(EntityPtr _parent);
      public: virtual ~RokiLink();
      public: virtual void Load(sdf::ElementPtr _sdf);
      public: virtual void Init();
      public: virtual void Fini();

      public: virtual void OnPoseChange();
      public: virtual void SetEnabled(bool _enable) const;
      public: virtual bool GetEnabled() const;
      public: virtual void UpdateMass();

      public: virtual void UpdateSurface();
      public: virtual void SetLinearVel(const ignition::math::Vector3d &_vel);
      public: virtual void SetAngularVel(const ignition::math::Vector3d &_vel);

      public: virtual void SetForce(const ignition::math::Vector3d &_force);
      public: virtual void SetTorque(const ignition::math::Vector3d &_torque);

      public: virtual void AddForce(const ignition::math::Vector3d &_force);
      public: virtual void AddRelativeForce(const ignition::math::Vector3d &_force);

      public: virtual void AddForceAtWorldPosition(const ignition::math::Vector3d &_force,
                                                   const ignition::math::Vector3d &_pos);

      public: virtual void AddForceAtRelativePosition(
                  const ignition::math::Vector3d &_force,
                  const ignition::math::Vector3d &_relpos);

      public: virtual void AddLinkForce(const ignition::math::Vector3d &_force,
          const ignition::math::Vector3d &_offset = ignition::math::Vector3d::Zero);

      public: virtual void AddTorque(const ignition::math::Vector3d &_torque);
      public: virtual void AddRelativeTorque(const ignition::math::Vector3d &_torque);

      public: virtual ignition::math::Vector3d WorldLinearVel(
          const ignition::math::Vector3d &_offset) const;

      public: virtual ignition::math::Vector3d WorldLinearVel(
                  const ignition::math::Vector3d &_offset,
                  const ignition::math::Quaterniond &_q) const;

      public: virtual ignition::math::Vector3d WorldCoGLinearVel() const;
      public: virtual ignition::math::Vector3d WorldAngularVel() const;
      public: virtual ignition::math::Vector3d WorldForce() const;

      public: virtual ignition::math::Vector3d WorldTorque() const;

      public: virtual void SetGravityMode(bool _mode);
      public: virtual bool GetGravityMode() const;

      public: void SetSelfCollide(bool _collide);

      public: virtual void SetLinearDamping(double _damping);
      public: virtual void SetAngularDamping(double _damping);

      public: virtual void SetKinematic(const bool &_state);
      public: virtual bool GetKinematic() const;

      public: virtual void SetAutoDisable(bool _disable);

      public: virtual void SetLinkStatic(bool _static);

      public:
        void SetStateToRoki();
        void SetStateFromRoki();
        std::string  GetPathName() const;
        RokiModelPtr GetRokiModel() const;
        RokiJointPtr GetConnectedRokiJoint() const;
        RokiLinkPtr GetParentRokiLink() const;
        bool HaveParentRokiLink() const;
        void SetGazeboPoseToRokiFrame();
        ignition::math::Pose3d GetDirtyPoseFromRoki() const;
        rkFDCell *GetRkFDCell() const;
        rkChain  *GetRkChain() const;
        RokiCollisionPtr GetRokiCollision() const;

      public:
        rkLink    *rklink_anchor_;
        rkLink    *rklink_org_;
        int        rklink_anchor_id_;
        int        rklink_org_id_;
        rkJoint   *rkjoint_anchor_;
        rkJoint   *rkjoint_org_;
    };
  }
}
#endif
