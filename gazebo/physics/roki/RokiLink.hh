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
    class GZ_PHYSICS_ODE_VISIBLE RokiLink : public Link
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
      public: virtual void SetLinearVel(const math::Vector3 &_vel);
      public: virtual void SetAngularVel(const math::Vector3 &_vel);

      public: virtual void SetForce(const math::Vector3 &_force);
      public: virtual void SetTorque(const math::Vector3 &_torque);

      public: virtual void AddForce(const math::Vector3 &_force);
      public: virtual void AddRelativeForce(const math::Vector3 &_force);

      public: virtual void AddForceAtWorldPosition(const math::Vector3 &_force,
                                                   const math::Vector3 &_pos);

      public: virtual void AddForceAtRelativePosition(
                  const math::Vector3 &_force,
                  const math::Vector3 &_relpos);

      public: virtual void AddLinkForce(const math::Vector3 &_force,
          const math::Vector3 &_offset = math::Vector3::Zero);

      public: virtual void AddTorque(const math::Vector3 &_torque);
      public: virtual void AddRelativeTorque(const math::Vector3 &_torque);

      public: virtual math::Vector3 GetWorldLinearVel(
          const math::Vector3 &_offset) const;

      public: virtual math::Vector3 GetWorldLinearVel(
                  const math::Vector3 &_offset,
                  const math::Quaternion &_q) const;

      public: virtual math::Vector3 GetWorldCoGLinearVel() const;
      public: virtual math::Vector3 GetWorldAngularVel() const;
      public: virtual math::Vector3 GetWorldForce() const;

      public: virtual math::Vector3 GetWorldTorque() const;

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
        math::Pose GetDirtyPoseFromRoki() const;
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
