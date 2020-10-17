#include <math.h>
#include <sstream>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/WorldPrivate.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Shape.hh"

#include "gazebo/physics/roki/RokiModel.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/physics/roki/RokiJoint.hh"
#include "gazebo/physics/roki/RokiCollision.hh"
#include "gazebo/physics/roki/RokiUtils.hh"

#include <sstream>

using namespace gazebo;
using namespace physics;

RokiLink::RokiLink(EntityPtr _parent)
  : Link(_parent), rklink_anchor_(nullptr), rklink_org_(nullptr), rklink_anchor_id_(-1), rklink_org_id_(-1)
{
  DEBUG_PRINT("RokiLink::RokiLink()\n");
}

RokiLink::~RokiLink()
{
  DEBUG_PRINT("RokiLink::~RokiLink()\n");
}

void RokiLink::Load(sdf::ElementPtr _sdf)
{
  DEBUG_PRINT("RokiLink::Load()\n");
  Link::Load(_sdf);
}

void RokiLink::Init()
{
  rklink_anchor_id_ = GetRokiModel()->link_id_counter_ ++;
  rklink_org_id_    = GetRokiModel()->link_id_counter_ ++;
  rklink_anchor_    = rkChainLink(GetRkChain(), rklink_anchor_id_);
  rklink_org_       = rkChainLink(GetRkChain(), rklink_org_id_);

  DEBUG_PRINT("RokiLink::Roki() enter : name=%s, rklink_anchor_id_=%d, rklink_org_id_=%d\n", GetPathName().c_str(), rklink_anchor_id_, rklink_org_id_);

  // initialize link
  rkLinkInit(rklink_anchor_);
  zNameSet(rklink_anchor_, const_cast<char*>(GetPathName().c_str()));
  rkLinkSetStuff(rklink_anchor_, const_cast<char*>("dummy_contact_info_rigid"));

  rkLinkInit(rklink_org_);
  zNameSet(rklink_org_, const_cast<char*>(GetPathName().c_str()));
  rkLinkSetStuff(rklink_org_, const_cast<char*>("dummy_contact_info_rigid"));

  // create org joint
  rkjoint_org_ = rkLinkJoint(rklink_org_);
  rkJointCreate(rkjoint_org_, RK_JOINT_FIXED);
  rkLinkAddChild(rklink_anchor_, rklink_org_);

  // create anchor joint
  rkjoint_anchor_ = rkLinkJoint(rklink_anchor_);
  RokiJointPtr joint = GetConnectedRokiJoint();

  RokiModelPtr model = GetRokiModel();
  if (model->IsStatic() == true) {
    DEBUG_PRINT("RokiLink::Init() rkJointCreate : name=%s, RK_JOINT_FIXED\n", GetPathName().c_str());
    rkJointCreate(rkjoint_anchor_, RK_JOINT_FIXED);
  }
  else if (!joint) {
    DEBUG_PRINT("RokiLink::InitRoki() rkJointCreate : name=%s, RK_JOINT_FLOAT\n", GetPathName().c_str());
    rkJointCreate(rkjoint_anchor_, RK_JOINT_FLOAT);
  }

  // set position
  SetGazeboPoseToRokiFrame();

  Link::Init();

  // set shapes
  Collision_V cs = GetCollisions();
  for (Collision_V::iterator ci = cs.begin();ci != cs.end(); ++ci) {
    RokiCollisionPtr c = boost::dynamic_pointer_cast<RokiCollision>(*ci);
    rkLinkShapePush(rklink_org_, c->shape_);
  } 

  this->UpdateMass();

  DEBUG_PRINT("RokiLink::Init() leave : name=%s\n", GetPathName().c_str());
}

void RokiLink::Fini()
{
  DEBUG_PRINT("RokiLink::Fini() : name~%s\n", GetPathName().c_str());
  Link::Fini();
}

void RokiLink::SetGravityMode(bool _mode)
{
  DEBUG_PRINT("RokiLink::SetGravityMode() : name=%s, mode=%s\n", GetPathName().c_str(), _mode?"true":"false");
  this->sdf->GetElement("gravity")->Set(_mode);
}

bool RokiLink::GetGravityMode() const
{
  bool val = this->sdf->GetElement("gravity")->Get<bool>();
  DEBUG_PRINT("RokiLink::GetGravityMode() : name=%s, mode=%s\n", GetPathName().c_str(), val?"true":"false");
  return val;
}

void RokiLink::SetSelfCollide(bool _collide)
{
  DEBUG_PRINT("RokiLink::SetSelfCollide() : name=%s, collide=%s\n", GetPathName().c_str(), _collide?"true":"false");
  this->sdf->GetElement("self_collide")->Set(_collide);

  if (!_collide) {
    GetRokiModel()->DisableSelfCollide();
  }
}

void RokiLink::OnPoseChange()
{
  math::Pose p = GetWorldPose();
  DEBUG_PRINT("RokiLink::OnPoseChange() enter : name=%s, world_pose=(%s)\n", GetPathName().c_str(), conv2str(p));

  Link::OnPoseChange();
  SetStateToRoki();

  DEBUG_PRINT("RokiLink::OnPoseChange() leave : name=%s\n", GetPathName().c_str());
}

void RokiLink::SetEnabled(bool _enable) const
{
  DEBUG_PRINT("RokiLink::SetEnabled() : name=%s, _enable=%s\n", GetPathName().c_str(), _enable?"true":"false");
}

bool RokiLink::GetEnabled() const
{
  DEBUG_PRINT("RokiLink::GetEnabled() : name=%s\n", GetPathName().c_str());

  bool result = true;
  return result;
}

void RokiLink::UpdateSurface()
{
  DEBUG_PRINT("RokiLink::UpdateSurface() : name=%s\n", GetPathName().c_str());
}

void RokiLink::UpdateMass()
{
  double mass = this->inertial->GetMass();
  math::Vector3 cog = this->inertial->GetCoG();
  math::Matrix3 moi = this->inertial->GetMOI();

  DEBUG_PRINT("RokiLink::UpdateMass() : name=%s, mass=%f, cog=%s, moi=%s\n", GetPathName().c_str(), mass, conv2str(cog), conv2str(moi));

  rkLinkSetMass(rklink_anchor_, 0.00001);
  zVec3DCreate(rkLinkCOM(rklink_anchor_), 0, 0, 0);
  zMat3DCreate(
      rkLinkInertia(rklink_anchor_),
      moi[0][0], moi[0][1], moi[0][2],
      moi[1][0], moi[1][1], moi[1][2],
      moi[2][0], moi[2][1], moi[2][2]);

  rkLinkSetMass(rklink_org_, mass);
  zVec3DCreate(rkLinkCOM(rklink_org_), cog.x, cog.y, cog.z);
  zMat3DCreate(
      rkLinkInertia(rklink_org_),
      moi[0][0], moi[0][1], moi[0][2],
      moi[1][0], moi[1][1], moi[1][2],
      moi[2][0], moi[2][1], moi[2][2]);
}

void RokiLink::SetLinearVel(const math::Vector3 &_vel)
{
  DEBUG_PRINT("RokiLink::SetLinerVel() : name=%s, vel~(%s)\n", GetPathName().c_str(), conv2str(_vel));
}

math::Vector3 RokiLink::GetWorldLinearVel(const math::Vector3 &_offset) const
{
  DEBUG_PRINT("RokiLink::GetWorldLinearVel() : name=%s, offset=(%s)\n", GetPathName().c_str(), conv2str(_offset));

  math::Vector3 vec;
  return vec;
}

math::Vector3 RokiLink::GetWorldLinearVel(const math::Vector3 &_offset,
                                         const math::Quaternion &_q) const
{
  DEBUG_PRINT("RokiLink::GetWorldCoGLinearVel() : name=%s, offset=(%s), q=(%s)\n", GetPathName().c_str(), conv2str(_offset), conv2str(_q));

  math::Vector3 vel;
  return vel;
}

math::Vector3 RokiLink::GetWorldCoGLinearVel() const
{
  DEBUG_PRINT("RokiLink::GetWorldCoGLinearVel() : name=%s\n", GetPathName().c_str());

  math::Vector3 vel;
  return vel;
}

void RokiLink::SetAngularVel(const math::Vector3 &_vel)
{
  DEBUG_PRINT("RokiLink::SetAngularVel() : name=%s, vel~(%s)\n", GetPathName().c_str(), conv2str(_vel));
}

math::Vector3 RokiLink::GetWorldAngularVel() const
{
  DEBUG_PRINT("RokiLink::GetWorldAngularVel() : name=%s\n", GetPathName().c_str());

  math::Vector3 vel;
  return vel;
}

void RokiLink::SetForce(const math::Vector3 &_force)
{
  DEBUG_PRINT("RokiLink::SetForce() : name-%s, force=(%s)\n", GetPathName().c_str(), conv2str(_force));
}

void RokiLink::SetTorque(const math::Vector3 &_torque)
{
  DEBUG_PRINT("RokiLink::SetTorque() : name=%s, torque=(%s)\n", GetPathName().c_str(), conv2str(_torque));
}

void RokiLink::AddForce(const math::Vector3 &_force)
{
  DEBUG_PRINT("RokiLink::AddForce() : name-%s, force=(%s)\n", GetPathName().c_str(), conv2str(_force));
}

void RokiLink::AddRelativeForce(const math::Vector3 &_force)
{
  DEBUG_PRINT("RokiLink::AddRelativeForce() : name=%s, force=(%s)\n", GetPathName().c_str(), conv2str(_force));
}

void RokiLink::AddForceAtRelativePosition(const math::Vector3 &_force,
                               const math::Vector3 &_relpos)
{
  DEBUG_PRINT("RokiLink::AddForceAtRelativePosition() : name=%s, force=(%s), relpos=(%s)\n", GetPathName().c_str(), conv2str(_force), conv2str(_relpos));
}

void RokiLink::AddForceAtWorldPosition(const math::Vector3 &_force,
                                      const math::Vector3 &_pos)
{
  DEBUG_PRINT("RokiLink::AddForceAtWorldPosition() : name%s, force=(%s), pos=(%s)\n", GetPathName().c_str(), conv2str(_force), conv2str(_pos));
}

void RokiLink::AddLinkForce(const math::Vector3 &_force, const math::Vector3 &_offset)
{
  DEBUG_PRINT("RokiLink::AddLinkForce() : name-%s, force=(%s), offset=(%s)\n", GetPathName().c_str(), conv2str(_force), conv2str(_offset));
}

void RokiLink::AddTorque(const math::Vector3 &_torque)
{
  DEBUG_PRINT("RokiLink::AddTorque() : name=%s, torque=(%s)\n", GetPathName().c_str(), conv2str(_torque));
}

void RokiLink::AddRelativeTorque(const math::Vector3 &_torque)
{
  DEBUG_PRINT("RokiLink::AddRelativeTorque() name=%s, torque=(%s)\n", GetPathName().c_str(), conv2str(_torque));
}

math::Vector3 RokiLink::GetWorldForce() const
{
  DEBUG_PRINT("RokiLink::GetWorldForce() : name=%s\n", GetPathName().c_str());

  math::Vector3 force;
  return force;
}

math::Vector3 RokiLink::GetWorldTorque() const
{
  DEBUG_PRINT("RokiLink::GetWorldTorque() : name=%s\n", GetPathName().c_str());

  math::Vector3 torque;
  return torque;
}

void RokiLink::SetLinearDamping(double _damping)
{
  DEBUG_PRINT("RokiLink::SetLinearDamping() : name=%s, damping=%f\n", GetPathName().c_str(), _damping);
  gzerr << "Roki does not provide RokiLink::SetLinearDamping()\n";
}

void RokiLink::SetAngularDamping(double _damping)
{
  DEBUG_PRINT("RokiLink::SetAngularDamping() : name=%s, damping=%f\n", GetPathName().c_str(), _damping);
  gzerr << "Roki does not provide RokiLink::SetAngularDamping()\n";
}

void RokiLink::SetKinematic(const bool &_state)
{
  DEBUG_PRINT("RokiLink::SetKinematic() : name=%s\n, state=%s", GetPathName().c_str(), _state?"true":"false");
  gzerr << "Roki does not provide RokiLink::SetKinematic()\n";
}

bool RokiLink::GetKinematic() const
{
  DEBUG_PRINT("RokiLink::GetKinematic() : name=%s\n", GetPathName().c_str());
  return false;
}

void RokiLink::SetAutoDisable(bool _disable)
{
  DEBUG_PRINT("RokiLink::SetAutoDisable() : name=%s, disalbe~%s\n", GetPathName().c_str(), _disable?"true":"false");
  gzerr << "Roki does not provide RokiLink::SetAutoDisable()\n";
}

void RokiLink::SetLinkStatic(bool _flag)
{
  DEBUG_PRINT("RokiLink::SetLinkStatic() : name=%s, flag=%s\n", GetPathName().c_str(), _flag?"true":"false");
}

///////////////////////////////////////////////////
void RokiLink::SetStateToRoki()
{
  DEBUG_PRINT("RokiLink::SetStateToRoki() enter : name=%s\n", GetPathName().c_str());

  if (rklink_anchor_ == nullptr || rklink_org_ == nullptr) {
    DEBUG_PRINT("RokiLink::SetStateToRoki() leave : name=%s, rklink_ is null...\n", GetPathName().c_str());
    return;
  }

  if (rkLinkOffset(rklink_anchor_) < 0) {
    DEBUG_PRINT("RokiLink::SetStateToRoki() leave : name=%s, rklink_->offset<0...\n", GetPathName().c_str());
    return;
  }
  
  // gazebo pose -> roki org frame
  SetGazeboPoseToRokiFrame();

  // clear displacement
  byte joint_type = rkLinkJointType(rklink_anchor_);
  if (joint_type == RK_JOINT_FLOAT) {
    rkFDCell *lc    = GetRkFDCell();
    rkChain  *chain = GetRkChain();
    zVec dis = zVecAlloc(rkChainJointSize(chain));
    rkChainGetJointDisAll(chain, dis);

    int chain_offset_idx = rkChainLinkOffset(chain, rklink_anchor_id_);
    zVecElem(dis, chain_offset_idx + 0) = 0;
    zVecElem(dis, chain_offset_idx + 1) = 0;
    zVecElem(dis, chain_offset_idx + 2) = 0;
    zVecElem(dis, chain_offset_idx + 3) = 0;
    zVecElem(dis, chain_offset_idx + 4) = 0;
    zVecElem(dis, chain_offset_idx + 5) = 0;

    // apply chain displacementj
    rkFDChainSetDis(lc, dis);

    zVecFree(dis);
  }

  DEBUG_PRINT("RokiLink::SetStateToRoki() leave : name=%s\n", GetPathName().c_str());
}

void RokiLink::SetStateFromRoki()
{
  //DEBUG_PRINT("RokiLink::SetStateFromRoki() enter : name=%s\n", GetPathName().c_str());

  if (rklink_anchor_ == nullptr || rklink_org_ == nullptr) {
    //DEBUG_PRINT("RokiLink::SetStateFromRoki() leave : name=%s, rklink_ is null...\n", GetPathName().c_str());
    return;
  }

  if (rkLinkOffset(rklink_anchor_) < 0 && !HaveParentRokiLink()) {
    //DEBUG_PRINT("RokiLink::SetStateFromRoki() leave : name=%s, rklink_->offset<0 && rkLinkParent(l) is null...\n", GetPathName().c_str());
    return;
  }

  // get frame
  math::Pose dirty_pose  = GetDirtyPoseFromRoki(); // related pose
  //DEBUG_PRINT("RokiLink::SetStateFromRoki() : name=%s, dirty_pose=(%s)\n", GetPathName().c_str(), conv2str(dirty_pose));

  // set dirty pose (world coordinate pose)
  this->dirtyPose = dirty_pose;

  // notify dirty pose
  this->world->dataPtr->dirtyPoses.push_back(this);

  //DEBUG_PRINT("RokiLink::SetStateFromRoki() leave : name=%s\n", GetPathName().c_str());
}

std::string RokiLink::GetPathName() const
{ 
  RokiModelPtr model = GetRokiModel();
  std::string model_name = model->GetName();
  std::string link_name = GetName();

  std::string link_path_name = model_name + "::" + link_name;

  return link_path_name;
}

RokiModelPtr RokiLink::GetRokiModel() const
{
  RokiModelPtr model = boost::dynamic_pointer_cast<RokiModel>(GetModel());
  return model;
}

RokiJointPtr RokiLink::GetConnectedRokiJoint() const
{
  ModelPtr model = GetModel();
  Joint_V js = model->GetJoints();  

  RokiJointPtr result_joint;

  Joint_V::iterator ji;
  for (ji = js.begin(); ji != js.end(); ++ji) {
    LinkPtr child_link = (*ji)->GetChild();
    if (child_link.get() == this) {
      result_joint = boost::dynamic_pointer_cast<RokiJoint>(*ji);
    }
  }

  return result_joint;
}

RokiLinkPtr RokiLink::GetParentRokiLink() const
{
  RokiJointPtr joint = GetConnectedRokiJoint();
  if (joint) {
    return joint->GetParentRokiLink();
  }
  return RokiLinkPtr();
}

bool RokiLink::HaveParentRokiLink() const
{
  RokiLinkPtr p = GetParentRokiLink();
  if (p) return true;
  return false;
}

void RokiLink::SetGazeboPoseToRokiFrame()
{
  math::Pose p_org_wld = GetWorldPose();
  math::Pose frame_anchor, frame_org;

  RokiJointPtr joint = GetConnectedRokiJoint();
  RokiLinkPtr  link_parent = GetParentRokiLink();

  if (!link_parent) {
    // root link
    frame_anchor = p_org_wld;
    frame_org    = math::Pose::Zero;
  }
  else {
    math::Pose p_parent_wld = link_parent->GetWorldPose();
    math::Pose org2anchor_pose = joint->org2anchor_pose_;
    math::Pose anchor2org_pose = org2anchor_pose.GetInverse();

    DEBUG_PRINT("RokiLink::SetGazeboPoseToRokiFrame() : name=%s, p_org_wld=%s, p_parent_wld=%s, p_parent_wld.GetInverse()=%s\n", GetName().c_str(), conv2str(p_org_wld), conv2str(p_parent_wld), conv2str(p_parent_wld.GetInverse()));
    DEBUG_PRINT("RokiLink::SetGazeboPoseToRokiFrame() : name=%s, org2anchor_pose=%s, anchor2org_pose=%s\n", GetName().c_str(), conv2str(org2anchor_pose), conv2str(anchor2org_pose));

    // child link
    frame_anchor = org2anchor_pose + (p_org_wld - p_parent_wld);
    frame_org    = anchor2org_pose;
  }

  DEBUG_PRINT("RokiLink::SetGazeboPoseToRokiFrame() : name=%s, frame_anchor=%s, frame_org=%s\n", GetName().c_str(), conv2str(frame_anchor), conv2str(frame_org));

  pose2zFrame3D(frame_anchor, rkLinkOrgFrame(rklink_anchor_));
  pose2zFrame3D(frame_org,    rkLinkOrgFrame(rklink_org_));
}

math::Pose RokiLink::GetDirtyPoseFromRoki() const
{
  math::Pose result;

  math::Pose adj_frame = conv2pose(rkLinkWldFrame(rklink_org_));
  result = adj_frame;

  return result;
}

rkChain *RokiLink::GetRkChain() const
{
  RokiModelPtr model = GetRokiModel();
  return model->chain_;
}

rkFDCell *RokiLink::GetRkFDCell() const
{
  RokiModelPtr model = GetRokiModel();
  return model->cell_;
}

RokiCollisionPtr RokiLink::GetRokiCollision() const
{
  RokiCollisionPtr c;
  CollisionPtr p = GetCollision(0);
  if (p) {
    c = boost::dynamic_pointer_cast<RokiCollision>(p);
  }
  return c;
}
