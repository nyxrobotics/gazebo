#include <boost/shared_ptr.hpp>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class RokiCollision;
    class RokiJoint;
    class RokiLink;
    class RokiModel;
    class RokiSurfaceParams;
    class RokiPhysics;

    typedef boost::shared_ptr<RokiCollision>     RokiCollisionPtr;
    typedef boost::shared_ptr<RokiLink>          RokiLinkPtr;
    typedef boost::shared_ptr<RokiModel>         RokiModelPtr;
    typedef boost::shared_ptr<RokiJoint>         RokiJointPtr;
    typedef boost::shared_ptr<RokiSurfaceParams> RokiSurfaceParamsPtr;
    typedef boost::shared_ptr<RokiPhysics>       RokiPhysicsPtr;
  }
}
