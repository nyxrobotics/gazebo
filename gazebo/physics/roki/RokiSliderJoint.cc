#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Link.hh"

#include "gazebo/physics/roki/RokiTypes.hh"
#include "gazebo/physics/roki/RokiUtils.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/physics/roki/RokiJoint.hh"
#include "gazebo/physics/roki/RokiSliderJoint.hh"

#include "rk_motor_pseudo.h"

using namespace gazebo;
using namespace physics;

RokiSliderJoint::RokiSliderJoint(BasePtr _parent)
  : SliderJoint<RokiJoint>(_parent), prp_(nullptr)
{
  DEBUG_PRINT("RokiSliderJoint::RokiSliderJoint() : joint_name=%s\n", GetName().c_str());
}

RokiSliderJoint::~RokiSliderJoint()
{
  DEBUG_PRINT("RokiSliderJoint::~RokiSliderJoint() : joint_name=%s\n", GetName().c_str());
}

void RokiSliderJoint::Load(sdf::ElementPtr _sdf)
{
  DEBUG_PRINT("RokiSliderJoint::Load() : joint_name=%s\n", GetName().c_str());
  SliderJoint<RokiJoint>::Load(_sdf);
}

void RokiSliderJoint::Init()
{
  DEBUG_PRINT("RokiSliderJoint::Init() enter : joint_name=%s\n", GetName().c_str());

  SliderJoint<RokiJoint>::Init();

  // create rkJoint
  rkjoint_ = GetChildRokiLink()->rkjoint_anchor_;
  rkJointCreate(rkjoint_, RK_JOINT_PRISM);

  // roki/rk_joint_prism.h : 
  //   typedef struct{
  //     /* joint displacement, velocity, acceleration and torque */
  //     double dis, vel, acc, trq;
  //     double min, max; /* limiter */
  //     /* joint stiffness, viscosity and coulomb friction */
  //     double stiff, viscos, coulomb;
  //     /* friction */
  //     double tf;
  //     /* static friction */
  //     double sf;
  //
  //     /* motor */
  //     rkMotor m;
  //
  //     /* for forward dynamics */
  //     rkJointRef _ref;
  //   } rkJointPrpPrism;

  // set joint parameters
  prp_ = (rkJointPrpPrism*)rkjoint_->prp;
  sdf::ElementPtr elm_limit = this->sdf->GetElement("axis")->GetElement("limit");
  prp_->min = elm_limit->Get<double>("lower");
  prp_->max = elm_limit->Get<double>("upper");
  prp_->tf  = friction_[0];
  prp_->sf  = friction_[0];

  // create rkMotor
  rkJointGetMotor(rkjoint_, &rkmotor_);
  rkMotorInit(rkmotor_);
  rkMotorCreatePseudo(rkmotor_);
  
  // connect rkLinks
  RokiLinkPtr link_parent = GetParentRokiLink();
  RokiLinkPtr link_child  = GetChildRokiLink();

  if (link_parent) {
    rkLink* parent_org_frame   = link_parent->rklink_org_;
    rkLink* child_anchor_frame = link_child->rklink_anchor_;
    rkLinkAddChild(parent_org_frame, child_anchor_frame);
  }

  //
  // create org2anchor offset pose
  //

  // position
  math::Vector3 pos_org_wld = GetChildRokiLink()->GetWorldPose().pos;
  org2anchor_pose_.pos = anchor_pos_ - pos_org_wld;
  DEBUG_PRINT("RokiSliderJoint::Init() : name=%s, pos_org_wld=%s, anchor_pos_=%s, org2anchor_pose_.pos=%s\n", GetName().c_str(), conv2str(pos_org_wld), conv2str(anchor_pos_), conv2str(org2anchor_pose_.pos));

  // rotate Z axis -> anchor_axis_ 
  math::Vector3 axis_z(0, 0, 1);
  math::Vector3 axis_rot = axis_z.Cross(anchor_axis_);
  double th = acos(axis_z.Dot(anchor_axis_)/(axis_z.GetLength() * anchor_axis_.GetLength()));

  org2anchor_pose_.rot.SetFromAxis(axis_rot.Normalize(), th);

  DEBUG_PRINT("RokiSliderJoint::Init() : name=%s, joint_axis_=%s, axis_rot=%s, org2anchor_pose_.rot=%s\n", GetName().c_str(), conv2str(anchor_axis_), conv2str(axis_rot), conv2str(org2anchor_pose_.rot));

  DEBUG_PRINT("RokiSliderJoint::Init() leave : joint_name=%s\n", GetName().c_str());
}

math::Angle RokiSliderJoint::GetAngleImpl(unsigned int _index) const
{
  DEBUG_PRINT("RokiSliderJoint::GetAngleImpl() : joint_name=%s, _index=%d\n", GetName().c_str(), _index);
  return math::Angle();
}

void RokiSliderJoint::SetVelocity(unsigned int _index, double _vel)
{
  DEBUG_PRINT("RokiSliderJoint::SetVelocity() : joint_name=%s, _index=%d, _vt=%f\n", GetName().c_str(), _index, _vel);
  double v = 0;
  rkJointSetVel(rkjoint_, &v);
}

double RokiSliderJoint::GetVelocity(unsigned int _index) const
{
  DEBUG_PRINT("RokiSliderJoint::GetVelocity() : joint_name=%s, _index=%d\n", GetName().c_str(), _index);
  double v = 0;
  rkJointGetVel(rkjoint_, &v);
  return v;
}

void RokiSliderJoint::SetForceImpl(unsigned int _index, double _effort)
{
  DEBUG_PRINT("RokiSliderJoint::SetForceImpl() : joint_name=%s, _index=%d, _effort=%f\n", GetName().c_str(), _index, _effort);
  double e = _effort;
  rkJointMotorSetInput(rkjoint_, &e);
}

double RokiSliderJoint::GetForceImpl(unsigned int _index)
{
  double f = 0;
  rkJointGetTrq(rkjoint_, &f);
  DEBUG_PRINT("RokiSliderJoint::GetForceImpl() : joint_name=%s, _index=%d, force=%f\n", GetName().c_str(), _index, f);
  return f;
}

void RokiSliderJoint::SetFriction(const unsigned int _index, const double value)
{
  RokiJoint::SetFriction(_index, value);

  if (_index > 0) return;
  if (rkmotor_ == nullptr) return;

  prp_->tf  = value;
  prp_->sf  = value;
}
