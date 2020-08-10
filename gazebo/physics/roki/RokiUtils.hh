#ifndef _ROKIUTILS_HH_
#define _ROKIUTILS_HH_

#include "gazebo/physics/PhysicsEngine.hh"
#include "roki/rk_fd.h"
#include "RokiTypes.hh"

#include <boost/any.hpp>
#include <string>

namespace gazebo
{
  namespace physics
  {
    // for debug
    #define ENABLE_DEBUG_PRINT(...)  gazebo::physics::set_roki_debug_print_flag_(true)
    #define DEBUG_PRINT(...)  gazebo::physics::roki_debug_print_(__VA_ARGS__)
    void set_roki_debug_print_flag_(const bool flag);
    void roki_debug_print_(const char *format, ...);

    #define any2str(a) (any2str_(a).c_str())
    std::string any2str_(const boost::any &a); 

    #define conv2str(a) (conv2str_(a).c_str())
    std::string conv2str_(const math::Angle &a);
    std::string conv2str_(const math::Vector3 &v);
    std::string conv2str_(const math::Quaternion &q);
    std::string conv2str_(const math::Matrix3 &m);
    std::string conv2str_(const math::Matrix4 &m);
    std::string conv2str_(const math::Pose &p);
    std::string conv2str_(const zFrame3D *f);

    math::Pose conv2pose(const zVec dis, const int chain_offset_idx);
    math::Pose conv2pose(const zFrame3D *f);

    void pose2zFrame3D(const math::Pose &pose, zFrame3D *p);
    void pose2rklinkdis(const math::Pose &pose, rkFDCell *lc, rkChain *chain, const int &rklink_idx);

    class DummyRokiObject {
      public:
        DummyRokiObject(rkFD *fd_);
        virtual ~DummyRokiObject();

        rkFDCell *lc;
        rkChain  *chain;
        rkLink   *rklink;
        rkJoint  *joint;
    };

    typedef boost::shared_ptr<DummyRokiObject>       DummyRokiObjectPtr;
  }
}

#endif
