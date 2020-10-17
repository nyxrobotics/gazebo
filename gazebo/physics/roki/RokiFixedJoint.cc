#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Link.hh"

#include "gazebo/physics/roki/RokiTypes.hh"
#include "gazebo/physics/roki/RokiUtils.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/physics/roki/RokiJoint.hh"
#include "gazebo/physics/roki/RokiFixedJoint.hh"

using namespace gazebo;
using namespace physics;

RokiFixedJoint::RokiFixedJoint(BasePtr _parent)
  : FixedJoint<RokiJoint>(_parent)
{
  DEBUG_PRINT("RokiFixedJoint::RokiFixedJoint() : joint_name=%s\n", GetName().c_str());
}

RokiFixedJoint::~RokiFixedJoint()
{
  DEBUG_PRINT("RokiFixedJoint::~RokiFixedJoint() : joint_name=%s\n", GetName().c_str());
}

void RokiFixedJoint::Load(sdf::ElementPtr _sdf)
{
  DEBUG_PRINT("RokiFixedJoint::Load() : joint_name=%s\n", GetName().c_str());
  FixedJoint<RokiJoint>::Load(_sdf);
}

void RokiFixedJoint::Init()
{
  DEBUG_PRINT("RokiFixedJoint::Init() enter : joint_name=%s\n", GetName().c_str());
  DEBUG_PRINT("RokiFixedJoint::Init() call FixedJoint<RokiJoint>::Init() : joint_name=%s\n", GetName().c_str());
  FixedJoint<RokiJoint>::Init();
  DEBUG_PRINT("RokiFixedJoint::Init() return FixedJoint<RokiJoint>::Init() : joint_name=%s\n", GetName().c_str());

  RokiLinkPtr link_parent = GetParentRokiLink();
  RokiLinkPtr link_child  = GetChildRokiLink();

  rkJoint* child_anchor_joint = link_child->rkjoint_anchor_;
  
  DEBUG_PRINT("RokiFixedJoint::init() : joint_name=%s\n", GetName().c_str());

  rkJointCreate(child_anchor_joint, RK_JOINT_FIXED);

  if (!link_parent) {
    // world -> root joint
    DEBUG_PRINT("RokiFixedJoint::Init() : joint_name=%s (world->root)\n", GetName().c_str());
    DEBUG_PRINT("RokiFixedJoint::Init() leave : joint_name=%s\n", GetName().c_str());
    return;
  }

  rkLink* parent_org_frame   = link_parent->rklink_org_;
  rkLink* child_anchor_frame = link_child->rklink_anchor_;
  rkLinkAddChild(parent_org_frame, child_anchor_frame);

  DEBUG_PRINT("RokiFixedJoint::Init() : rkLinkAddChild() parent=%s, child=%s\n", link_parent->GetName().c_str(), link_child->GetName().c_str());

  DEBUG_PRINT("RokiFixedJoint::Init() leave : joint_name=%s\n", GetName().c_str());
}

 double RokiFixedJoint::PositionImpl(unsigned int _index) const
{
  gzwarn << "ODEFixedJoint: called method "
         << "PositionImpl that is not valid for joints of type fixed.\n";
  return ignition::math::NAN_D;
}

void RokiFixedJoint::SetVelocity(unsigned int _index, double _vel)
{
  DEBUG_PRINT("RokiFixedJoint::SetVelocity() : joint_name=%s, _index=%d, _vt=%f\n", GetName().c_str(), _index, _vel);
}

double RokiFixedJoint::GetVelocity(unsigned int _index) const
{
  DEBUG_PRINT("RokiFixedJoint::GetVelocity() : joint_name=%s, _index=%d\n", GetName().c_str(), _index);
  return 0.0;
}

void RokiFixedJoint::SetForceImpl(unsigned int _index, double _effort)
{
  DEBUG_PRINT("RokiFixedJoint::SetForceImpl() : joint_name=%s, _index=%d, _effort=%f\n", GetName().c_str(), _index, _effort);
}

double RokiFixedJoint::GetForceImpl(unsigned int _index)
{
  double f = 0;
  DEBUG_PRINT("RokiFixedJoint::GetForceImpl() : joint_name=%s, _index=%d, force=%f\n", GetName().c_str(), _index, f);
  return f;
}
