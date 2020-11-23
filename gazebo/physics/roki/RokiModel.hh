#ifndef _ROKIMODEL_HH_
#define _ROKIMODEL_HH_

#include "gazebo/physics/Model.hh"
#include "gazebo/util/system.hh"
#include "roki/rk_fd.h"
#include "roki/rk_link.h"
#include "roki/rk_joint.h"

namespace gazebo
{
  namespace physics
  {
    class GZ_PHYSICS_VISIBLE RokiModel : public Model
    {
      public: explicit RokiModel(BasePtr _parent);
      public: virtual ~RokiModel();
      public: virtual void Init();

      public:
        void DisableSelfCollide();
        rkFD* GetRkFD() const;

      public:
        rkFDCell *cell_;
        rkChain  *chain_;
        int      link_id_counter_;
        bool     self_collide_;
    };
  }
}
#endif
