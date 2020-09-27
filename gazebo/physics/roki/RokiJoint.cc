#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/PhysicsEngine.hh"

#include "gazebo/physics/roki/RokiModel.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/physics/roki/RokiJoint.hh"
#include "gazebo/physics/roki/RokiUtils.hh"

using namespace gazebo;
using namespace physics;

RokiJoint::RokiJoint(BasePtr _parent)
  : Joint(_parent), rkjoint_(nullptr), rkmotor_(nullptr)
{
  DEBUG_PRINT("RokiJoint::RokiJoint()\n");
}

RokiJoint::~RokiJoint()
{
  DEBUG_PRINT("RokiJoint::~RokiJoint()\n");
  this->Detach();
  if (rkmotor_) {
    rkMotorDestroy(rkmotor_);
    rkmotor_ = nullptr;
  }
}

void RokiJoint::Load(sdf::ElementPtr _sdf)
{
  DEBUG_PRINT("RokiJoint::Load()\n");
  Joint::Load(_sdf);
}

void RokiJoint::Init()
{
  DEBUG_PRINT("RokiJoint::Init() enter : joint_name=%s\n", GetName().c_str());

  Joint::Init();

  DEBUG_PRINT("RokiJoint::Init() leave : joint_name=%s\n", GetName().c_str());
}

void RokiJoint::Reset()
{
  DEBUG_PRINT("RokiJoint::Reset() : joint_name=%s\n", GetName().c_str());
  Joint::Reset();
}

LinkPtr RokiJoint::GetJointLink(unsigned int _index) const
{
  DEBUG_PRINT("RokiJoint::GetJointLink() : joint_name=%s, _index~%d\n", GetName().c_str(), _index);

  LinkPtr result;
  return result;
}

bool RokiJoint::AreConnected(LinkPtr _one, LinkPtr _two) const
{
  DEBUG_PRINT("RokiJoint::AreConnected() : joint_name=%s\n", GetName().c_str());

  if (_one.get() == NULL && _two.get() == NULL) return false;

  if ((this->childLink.get() == _one.get() && this->parentLink.get() == _two.get())
      || (this->childLink.get() == _two.get() && this->parentLink.get() == _one.get())) {
    return true;
  }
  return false;
}

void RokiJoint::Attach(LinkPtr _parent, LinkPtr _child)
{
  DEBUG_PRINT("RokiJoint::Attach() : joint_name=%s\n", GetName().c_str());

  Joint::Attach(_parent, _child);
  if (this->AreConnected(_parent, _child)) return;
}

void RokiJoint::Detach()
{
  DEBUG_PRINT("RokiJoint::Detach() : joint_name=%s\n", GetName().c_str());

  if (!this->AreConnected(this->parentLink, this->childLink)) return;

  this->childLink.reset();
  this->parentLink.reset();
  gzerr << "Roki does not support joint dettaching.\n";

  Joint::Detach();
}

ignition::math::Vector3d RokiJoint::GetAnchor(unsigned int _index) const
{
  DEBUG_PRINT("RokiFixedJoint::GetAnchor() : joint_name=%s, _index=%d, anchor_pos_=%s\n", GetName().c_str(), _index, conv2str(anchor_pos_));
  return anchor_pos_;
}

void RokiJoint::SetAnchor(unsigned int _index, const ignition::math::Vector3d & _anchor)
{
  DEBUG_PRINT("RokiJoint::SetAnchor() : joint_name=%s, _index=%d, _anchor=%s\n", GetName().c_str(), _index, conv2str(_anchor));
  anchor_pos_ = _anchor;
}

ignition::math::Vector3d RokiJoint::GetAxis(unsigned int _index) const
{
  DEBUG_PRINT("RokiFixedJoint::GetAxis() : joint_name=%s, _index=%d, anchor_axis_=%s\n", GetName().c_str(), _index, conv2str(anchor_axis_));
  return anchor_axis_;
}

ignition::math::Vector3d RokiJoint::GetGlobalAxis(unsigned int _index) const
{
  DEBUG_PRINT("RokiFixedJoint::GetGlobalAxis() : joint_name=%s, _index=%d, anchor_axis_=%s\n", GetName().c_str(), _index, conv2str(anchor_axis_));
  return anchor_axis_;
}

void RokiJoint::SetAxis(unsigned int _index, const ignition::math::Vector3d& _axis)
{
  DEBUG_PRINT("RokiJoint::SetAxis() : joint_name=%s, _index=%d, _axis=%s\n", GetName().c_str(), _index, conv2str(_axis));
  anchor_axis_ = _axis;
}

bool RokiJoint::SetHighStop(unsigned int _index, const ignition::math::Angle &_angle)
{
  highStop_[_index] = _angle;
  return true;
}

bool RokiJoint::SetLowStop(unsigned int _index, const ignition::math::Angle &_angle)
{
  lowStop_[_index] = _angle;
  return true;
}

ignition::math::Angle RokiJoint::GetHighStop(unsigned int _index)
{
  return highStop_[_index];
}

ignition::math::Angle RokiJoint::GetLowStop(unsigned int _index)
{
  return lowStop_[_index];
}

void RokiJoint::SetDamping(unsigned int _index, double _damping)
{
  DEBUG_PRINT("RokiJoint::SetDamping() : joint_name=%s, _index=%d, _damping=%f\n", GetName().c_str(), _index, _damping);
  if (_index < GetAngleCount()) {
    this->SetStiffnessDamping(_index, this->stiffnessCoefficient[_index], _damping);
  }
}

void RokiJoint::SetStiffness(unsigned int _index, const double _stiffness)
{
  DEBUG_PRINT("RokiJoint::SetStiffness() : joint_name=%s, _index=%d, _stiffness=%f\n", GetName().c_str(), _index, _stiffness);
  if (_index < GetAngleCount()) {
    this->SetStiffnessDamping(_index, _stiffness, this->dissipationCoefficient[_index]);
  }
}

void RokiJoint::SetStiffnessDamping(unsigned int _index, double _stiffness, double _damping, double _reference)
{
  DEBUG_PRINT("RokiJoint::SetStiffness() : joint_name=%s, _index=%d, _stiffness=%f\n", GetName().c_str(), _index, _stiffness);
  if (_index < this->GetAngleCount()) {
    this->stiffnessCoefficient[_index]    = _stiffness;
    this->dissipationCoefficient[_index]  = _damping;
    this->springReferencePosition[_index] = _reference;
  }
}

void RokiJoint::SetMaxForce(unsigned int _index, double _force)
{
  DEBUG_PRINT("RokiJoint::SetMaxForce() : joint_name=%s, _index=%d, _force=%f\n", GetName().c_str(), _index, _force);
}

double RokiJoint::GetMaxForce(unsigned int _index)
{
  DEBUG_PRINT("RokiJoint::GetMaxForce() : joint_name=%s, _index=%d\n", GetName().c_str(), _index);
  return 0.0;
}

ignition::math::Vector3d RokiJoint::GetLinkForce(unsigned int _index) const
{
  DEBUG_PRINT("RokiJoint::GetLinkForce() : joint_name=%s, _index=%d\n", GetName().c_str(), _index);

  ignition::math::Vector3d result;
  return result;
}

ignition::math::Vector3d RokiJoint::GetLinkTorque(unsigned int _index) const
{
  DEBUG_PRINT("RokiJoint::GetLinkTorque() : joint_name=%s, _index=%d\n", GetName().c_str(), _index);

  ignition::math::Vector3d result;
  return result;
}

bool RokiJoint::SetParam(const std::string &_key, unsigned int _index, const boost::any &_value)
{
  DEBUG_PRINT("RokiJoint::SetParam() : joint_name=%s, _key=%s, _index=%d, _value=%s\n", GetName().c_str(), _key.c_str(), _index, any2str(_value));

  std::stringstream key;
  key << _key << ":" << _index;
  params_[_key] = _value;

  try {
    if (_key == "hi_stop") {
      this->SetHighStop(_index, boost::any_cast<double>(_value));
    }
    else if (_key == "lo_stop") {
      this->SetLowStop(_index, boost::any_cast<double>(_value));
    }
    else if (_key == "friction") {
      this->SetFriction(_index, boost::any_cast<double>(_value));
    }
    else {
      gzerr << "Unable to handle joint attribute[" << _key << "]\n";
      return false;
    }
  }
  catch(const boost::bad_any_cast &e) {
    gzerr << "SetParam(" << _key << ")" << " boost any_cast error:" << e.what() << std::endl;
    return false;
  }

  return true;
}

double RokiJoint::GetParam(const std::string &_key, unsigned int _index)
{
  DEBUG_PRINT("RokiJoint::GetParam() : joint_name=%s, _key=%s, _index=%d\n", GetName().c_str(), _key.c_str(), _index);

  std::stringstream key;
  key << _key << ":" << _index;

  if (params_.find(key.str()) != params_.end()) {
    return boost::any_cast<double>(params_[key.str()]);
  }

  return Joint::GetParam(_key, _index);
}

JointWrench RokiJoint::GetForceTorque(unsigned int _index)
{
  DEBUG_PRINT("RokiJoint::GetForceTorque() : joint_name=%s, _index=%d\n", GetName().c_str(), _index);

  JointWrench jointWrench;
  return jointWrench;
}

void RokiJoint::SetForce(unsigned int _index, double _force)
{
  DEBUG_PRINT("RokiJoint::SetForce() : joint_name=%s, _index=%d, _force=%f\n", GetName().c_str(), _index, _force);

  this->SetForceImpl(_index, _force);

  if (this->childLink) this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);
}

double RokiJoint::GetForce(unsigned int _index)
{
  DEBUG_PRINT("RokiJoint::GetForce() : joint_name=%s, _index=%d\n", GetName().c_str(), _index);

  if (this->GetAngleCount() <= _index) {
    gzerr << "Invalid joint index [" << _index << "] when trying to get force\n";
    return 0;
  }
  return this->GetForceImpl(_index);
}

unsigned int RokiJoint::GetAngleCount() const
{
  DEBUG_PRINT("RokiJoint::GetAngleCount() : joint_name=%s\n", GetName().c_str());
  unsigned int angleCount = 0;
  return angleCount;
}

void RokiJoint::ApplyDamping()
{
  DEBUG_PRINT("RokiJoint::GetAngleCount() : joint_name=%s\n", GetName().c_str());
}

RokiModelPtr RokiJoint::GetRokiModel() const
{
  RokiLinkPtr link = GetParentRokiLink();
  return boost::dynamic_pointer_cast<RokiModel>(link->GetModel());
}

rkFDCell* RokiJoint::GetRkFDCell() const
{
  return GetRokiModel()->cell_;
}

rkChain* RokiJoint::GetRkChain() const
{
  return GetRokiModel()->chain_;
}

RokiLinkPtr RokiJoint::GetParentRokiLink() const
{
  LinkPtr link = GetParent();
  return boost::dynamic_pointer_cast<RokiLink>(link);
}

RokiLinkPtr RokiJoint::GetChildRokiLink() const
{
  LinkPtr link = GetChild();
  return boost::dynamic_pointer_cast<RokiLink>(link);
}

void RokiJoint::SetFriction(const unsigned int _index, const double value)
{
  if (_index >= MAX_JOINT_AXIS) return;
  friction_[_index] = value;
}
