#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/Link.hh"

#include "gazebo/physics/roki/RokiTypes.hh"
#include "gazebo/physics/roki/RokiUtils.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/physics/roki/RokiJoint.hh"
#include "gazebo/physics/roki/RokiHingeJoint.hh"

#include "rk_motor_pseudo.h"

using namespace gazebo;
using namespace physics;


RokiHingeJoint::RokiHingeJoint(BasePtr _parent)
  : HingeJoint<RokiJoint>(_parent), prp_(nullptr)
{
  DEBUG_PRINT("RokiHingeJoint::RokiHingeJoint() : joint_name=%s\n", GetName().c_str());
}

RokiHingeJoint::~RokiHingeJoint()
{
  DEBUG_PRINT("RokiHingeJoint::~RokiHingeJoint() : joint_name=%s\n", GetName().c_str());
}

void RokiHingeJoint::Load(sdf::ElementPtr _sdf)
{
  DEBUG_PRINT("RokiHingeJoint::Load() : joint_name=%s\n", GetName().c_str());
  HingeJoint<RokiJoint>::Load(_sdf);
}

void RokiHingeJoint::Init()
{
  DEBUG_PRINT("RokiHingeJoint::Init() enter : joint_name=%s\n", GetName().c_str());

  HingeJoint<RokiJoint>::Init();

  // create rkJoint
  rkjoint_ = GetChildRokiLink()->rkjoint_anchor_;
  rkJointCreate(rkjoint_, RK_JOINT_REVOL);

  // roki/rk_joint_revol.h : 
  //  typedef struct{
  //    /* joint displacement, velocity, acceleration and torque */
  //    double dis, vel, acc, trq;
  //    double min, max; /* limiter */
  //    /* joint stiffness, viscosity and coulomb friction */
  //    double stiff, viscos, coulomb;
  //    /* friction */
  //    double tf;
  //    /* static friction */
  //    double sf;
  //    /* trigonometric values */
  //    double _s, _c;

  //    /* motor */
  //    rkMotor m;

  //    /* for forward dynamics */
  //    rkJointRef _ref;
  //  } rkJointPrpRevol;
  
  // set joint parameters
  prp_ = (rkJointPrpRevol*)rkjoint_->prp;
  prp_->min = lowStop_[0].Radian();
  prp_->max = highStop_[0].Radian();
  prp_->tf  = friction_[0];
  prp_->sf  = friction_[0];
  
  DEBUG_PRINT("RokiHingeJoint::Init() enter : joint_name=%s, friction=%f\n", GetName().c_str(), friction_[0]);

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
  math::Vector3 pos_org_wld = link_child->GetWorldPose().pos;
  org2anchor_pose_.pos = anchor_pos_ - pos_org_wld;
  DEBUG_PRINT("RokiHingeJoint::Init() : name=%s, pos_org_wld=%s, anchor_pos_=%s, org2anchor_pose_.pos=%s\n", GetName().c_str(), conv2str(pos_org_wld), conv2str(anchor_pos_), conv2str(org2anchor_pose_.pos));

  // rotate Z axis -> anchor_axis_ 
  math::Vector3 axis_z(0, 0, 1);
  math::Vector3 axis_rot = axis_z.Cross(anchor_axis_);
  double th = acos(axis_z.Dot(anchor_axis_)/(axis_z.GetLength() * anchor_axis_.GetLength()));

  org2anchor_pose_.rot.SetFromAxis(axis_rot.Normalize(), th);

  DEBUG_PRINT("RokiHingeJoint::Init() : name=%s, joint_axis_=%s, axis_rot=%s, org2anchor_pose_.rot=%s\n", GetName().c_str(), conv2str(anchor_axis_), conv2str(axis_rot), conv2str(org2anchor_pose_.rot));

  DEBUG_PRINT("RokiHingeJoint::Init() leave : joint_name=%s\n", GetName().c_str());
}

math::Angle RokiHingeJoint::GetAngleImpl(unsigned int _index) const
{
  DEBUG_PRINT("RokiHingeJoint::GetAngleImpl() : joint_name=%s, _index=%d\n", GetName().c_str(), _index);

  double dis;
  rkJointGetDis(rkjoint_, &dis);
  math::Angle angle;
  angle.SetFromRadian(dis);

  return angle;
}

void RokiHingeJoint::SetVelocity(unsigned int _index, double _vel)
{
  DEBUG_PRINT("RokiHingeJoint::SetVelocity() : joint_name=%s, _index=%d, _vt=%f\n", GetName().c_str(), _index, _vel);
  double v = 0;
  rkJointSetVel(rkjoint_, &v);
}

double RokiHingeJoint::GetVelocity(unsigned int _index) const
{
  DEBUG_PRINT("RokiHingeJoint::GetVelocity() : joint_name=%s, _index=%d\n", GetName().c_str(), _index);
  double v = 0;
  rkJointGetVel(rkjoint_, &v);
  return v;
}

void RokiHingeJoint::SetForceImpl(unsigned int _index, double _effort)
{
  DEBUG_PRINT("RokiHingeJoint::SetForceImpl() : joint_name=%s, _index=%d, _effort=%f\n", GetName().c_str(), _index, _effort);
  double e = _effort;
  rkJointMotorSetInput(rkjoint_, &e);
}

double RokiHingeJoint::GetForceImpl(unsigned int _index)
{
  double f = 0;
  rkJointMotorDrivingTrq(rkjoint_, &f);
  DEBUG_PRINT("RokiHingeJoint::GetForceImpl() : joint_name=%s, _index=%d, force=%f\n", GetName().c_str(), _index, f);
  return f;
}

void RokiHingeJoint::SetFriction(const unsigned int _index, const double value)
{
  RokiJoint::SetFriction(_index, value);

  if (_index > 0) return;
  if (rkmotor_ == nullptr) return;

  prp_->tf  = value;
  prp_->sf  = value;
}
