#ifndef _ROKIPHYSICS_HH_
#define _ROKIPHYSICS_HH_

#include "gazebo/physics/PhysicsEngine.hh"
#include <string>

#include "gazebo/physics/roki/RokiTypes.hh"
#include "gazebo/physics/roki/RokiUtils.hh"

#include "roki/rk_fd.h"

namespace gazebo
{
  namespace physics
  {
    class GZ_PHYSICS_VISIBLE RokiPhysics : public PhysicsEngine 
    {
      public: enum RokiParam
      {
        SOLVER_VERT,
        SOLVER_VOLUME
      };

      public: RokiPhysics(WorldPtr _world);
      public: virtual ~RokiPhysics();

      public: virtual void Load(sdf::ElementPtr _sdf);
      public: virtual void Init();
      public: virtual void Reset();
      public: virtual void InitForThread();
      public: virtual void UpdateCollision();
      public: virtual void UpdatePhysics();
      public: virtual void Fini();

      public: virtual std::string GetType() const
                      { return "roki"; }
      public: virtual void SetSeed(uint32_t _seed);

      public: virtual ModelPtr CreateModel(BasePtr _base);

      public: virtual LinkPtr CreateLink(ModelPtr _parent);
      public: virtual CollisionPtr CreateCollision(
                  const std::string &_shapeType, LinkPtr _parent);
      public: virtual ShapePtr CreateShape(const std::string &_shapeType,
                                           CollisionPtr _collision);
      public: virtual JointPtr CreateJoint(const std::string &_type,
                                           ModelPtr _parent);

      public: virtual void SetGravity(const ignition::math::Vector3d &_gravity);

      public: virtual void DebugPrint() const;

      public: virtual bool SetParam(const std::string &_key,
                  const boost::any &_value);

      public: virtual boost::any GetParam(const std::string &_key) const;

      public: virtual bool GetParam(const std::string &_key,
                  boost::any &_value) const;

      protected: virtual void OnRequest(ConstRequestPtr &_msg);
      protected: virtual void OnPhysicsMsg(ConstPhysicsPtr &_msg);

      private:
        void InitFD_();
        void ReleaseFD_();
        bool IsInitFD_();
        RokiLinkPtr GetRokiLinkByPathName(const char *path_name);

      public:
        rkFD *fd_;

        double contact_info_compensation_;
        double contact_info_relaxation_;
        double contact_info_static_friction_;
        double contact_info_friction_;
        enum RokiParam solver_type_;

      private:
        DummyRokiObjectPtr dummy_roki_object_;
    };
  }
}
#endif
