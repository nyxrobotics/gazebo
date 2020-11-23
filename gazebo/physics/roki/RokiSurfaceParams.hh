#ifndef _ROKISURFACEPARAMS_HH_
#define _ROKISURFACEPARAMS_HH_

#include <sdf/sdf.hh>

#include "ignition/math/Vector3.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class GZ_PHYSICS_VISIBLE RokiSurfaceParams : public SurfaceParams
    {
      public: RokiSurfaceParams();
      public: virtual ~RokiSurfaceParams();
      public: virtual void Load(sdf::ElementPtr _sdf);
      public: virtual void FillMsg(msgs::Surface &_msg);
      public: virtual void ProcessMsg(const msgs::Surface &_msg);
      public: double maxVel;
    };
  }
}
#endif
