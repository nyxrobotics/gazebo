#ifdef _WIN32
  #include <Winsock2.h>
#endif

#include <sdf/sdf.hh>

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "gazebo/util/Diagnostics.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Rand.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Timer.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/MapShape.hh"
#include "gazebo/physics/ContactManager.hh"

#include "gazebo/physics/roki/RokiPhysics.hh"
#include "gazebo/physics/roki/RokiModel.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/physics/roki/RokiCollision.hh"
#include "gazebo/physics/roki/RokiBoxShape.hh"
#include "gazebo/physics/roki/RokiCylinderShape.hh"
#include "gazebo/physics/roki/RokiSphereShape.hh"
#include "gazebo/physics/roki/RokiPlaneShape.hh"
#include "gazebo/physics/roki/RokiMeshShape.hh"
#include "gazebo/physics/roki/RokiHingeJoint.hh"
#include "gazebo/physics/roki/RokiSliderJoint.hh"
#include "gazebo/physics/roki/RokiFixedJoint.hh"
#include "gazebo/physics/roki/RokiUtils.hh"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("roki", RokiPhysics)

RokiPhysics::RokiPhysics(WorldPtr _world)
  : PhysicsEngine(_world) , fd_(nullptr),
    contact_info_compensation_(1000),
    contact_info_relaxation_(.01),
    contact_info_static_friction_(5.0),
    contact_info_friction_(3.0),
    solver_type_(SOLVER_VOLUME)
{
  DEBUG_PRINT("RokiPhysics::RokiPhysics()\n");
}

RokiPhysics::~RokiPhysics()
{
  DEBUG_PRINT("RokiPhysics::~RokiPhysics()\n");
}

bool RokiPhysics::IsInitFD_()
{
  if (fd_ != nullptr) return true;
  return false;
}

void RokiPhysics::InitFD_()
{
  DEBUG_PRINT("RokiPhysics::InitFD_() : enter\n");
  if (IsInitFD_()) ReleaseFD_();

  fd_ = zAlloc(rkFD, 1);
  rkFDCreate(fd_);

  // dummy contact info
  zArrayAlloc(&(fd_->ci), rkContactInfo, 1);
  rkContactInfo *ci;
  ci = zArrayElem(&fd_->ci, 0);
  rkContactInfoInit(ci);
  rkContactInfoRigidCreate(
      ci, 
        contact_info_compensation_,
        contact_info_relaxation_,
        contact_info_static_friction_,
        contact_info_friction_,
        (char*)"dummy_contact_info_rigid",
        (char*)"dummy_contact_info_rigid");

  // append dummy object
  dummy_roki_object_ = DummyRokiObjectPtr(new DummyRokiObject(fd_));

  // setup rkFD
  rkFDODE2Assign(fd_, Regular);
  //rkFDODE2AssignRegular(fd_, Euler);
  //rkFDODE2AssignRegular(fd_, Heun);
  //rkFDODE2AssignRegular(fd_, RK4);
  rkFDODE2AssignRegular(fd_, RKG);
  //rkFDODE2AssignRegular(fd_, RKF45);  // NG
  //rkFDODE2AssignRegular(fd_, Adams);  // NG
  //rkFDODE2AssignRegular(fd_, BEuler); // NG
  //rkFDODE2AssignRegular(fd_, TR);     // NG
  //rkFDODE2AssignRegular(fd_, BK4);    // NG
  //rkFDODE2AssignRegular(fd_, Gear);   // NG

  DEBUG_PRINT("RokiPhysics::InitFD_() : this->real_time_factor=%f\n", this->targetRealTimeFactor);
  DEBUG_PRINT("RokiPhysics::InitFD_() : this->max_step_size=%f\n", this->maxStepSize);
  DEBUG_PRINT("RokiPhysics::InitFD_() : this->realTimeUpdateRate=%f\n", this->realTimeUpdateRate);
  if(this->realTimeUpdateRate < 1.0){
    DEBUG_PRINT("RokiPhysics::InitFD_() :  realTimeUpdateRate TOO SMALL !! \n");
    this->SetRealTimeUpdateRate(1000.0);
    this->SetMaxStepSize(0.001);
    this->SetTargetRealTimeFactor(1.0);
  }
  rkFDSetDT(fd_, this->maxStepSize); // dummy
  DEBUG_PRINT("RokiPhysics::InitFD_() : this->realTimeUpdateRate=%f\n", this->realTimeUpdateRate);
  rkFDSetDT(fd_, 0.01); // dummy
  if (solver_type_ == SOLVER_VERT) {
    rkFDSetSolver(fd_, Vert);  // Volume or Vert. default: Vert
  }
  else {
    rkFDSetSolver(fd_, Volume);
  }
  rkFDUpdateInit(fd_);

  DEBUG_PRINT("RokiPhysics::InitFD_() : leave\n");
}

void RokiPhysics::ReleaseFD_()
{
  DEBUG_PRINT("RokiPhysics::ReleaseFD_() : enter\n");
  if (fd_ != nullptr) {
    //rkFDUpdateDestroy(fd_);
    rkFDDestroy(fd_);
    zFree(fd_);
    fd_ = nullptr;
  }
  DEBUG_PRINT("RokiPhysics::ReleaseFD_() : leave\n");
}

void RokiPhysics::Load(sdf::ElementPtr _sdf)
{
  sdf::ElementPtr e_roki;
  sdf::ElementPtr e_ci;

  // enable debug print
  if (_sdf->HasElement("roki") == true) {
    e_roki = _sdf->GetElement("roki");
    if(e_roki->Get<bool>("debug_print")) {
        ENABLE_DEBUG_PRINT();
        DEBUG_PRINT("RokiPhysics::Load() : ENABLE_DEBUG_PRINT()\n");
    }
  }

  DEBUG_PRINT("RokiPhysics::Load() : enter\n");

  // Elements for Roki settings
  if (_sdf->HasElement("roki") == true) {
    DEBUG_PRINT("RokiPhysics::Load() : <roki> tag found in sdf...name=%s\n", _sdf->GetName().c_str());

    e_roki = _sdf->GetElement("roki");

    if (e_roki->HasElement("contact_info")) {

      e_ci = e_roki->GetElement("contact_info");

      if (e_ci->HasElement("compensation")) {
        contact_info_compensation_ = e_ci->Get<double>("compensation");
      }
      if (e_ci->HasElement("relaxation")) {
        contact_info_relaxation_ = e_ci->Get<double>("relaxation");
      }
      if (e_ci->HasElement("static_friction")) {
        contact_info_static_friction_ = e_ci->Get<double>("static_friction");
      }
      if (e_ci->HasElement("friction")) {
        contact_info_friction_ = e_ci->Get<double>("friction");
      }
    }

    if (e_roki->HasElement("solver_type")) {
      std::string solver_type_str = e_roki->Get<std::string>("solver_type");
      std::transform(solver_type_str.begin(), solver_type_str.end(), solver_type_str.begin(), ::tolower);
      if (solver_type_str == "vert" || solver_type_str == "vertex") {
        solver_type_ = SOLVER_VERT;
      }
      else {
        solver_type_ = SOLVER_VOLUME;
      }
    }
  }
  else {
    DEBUG_PRINT("RokiPhysics::Load() : <roki> tag not found in sdf...name=%s\n", this->sdf->GetName().c_str());
  }

  DEBUG_PRINT("RokiPhysics::Load() : contact_info_compensation_=%f, contact_info_relaxation_=%f, contact_info_static_friction_=%f, contact_info_friction_=%f, solver_type=%s\n", contact_info_compensation_, contact_info_relaxation_, contact_info_static_friction_, contact_info_friction_, (solver_type_ == SOLVER_VERT?"VERT":"VOLUME"));

  InitFD_();
  PhysicsEngine::Load(_sdf);

  DEBUG_PRINT("RokiPhysics::Load() : leave\n");
}

void RokiPhysics::Init()
{
  printf("RokiPhysics::Init()\n");
  DEBUG_PRINT("RokiPhysics::Init() : rkFDUpdateInit(fd_)\n");
  rkFDUpdateInit(fd_);
  DEBUG_PRINT("RokiPhysics::Init() : leave\n");
}

void RokiPhysics::Reset()
{
  DEBUG_PRINT("RokiPhysics::Reset()\n");
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);
}

void RokiPhysics::InitForThread()
{
  DEBUG_PRINT("RokiPhysics::InitForThread()\n");
}

void RokiPhysics::UpdateCollision()
{
  //DEBUG_PRINT("RokiPhysics::UpdateCollision()\n");
  rkCDPair *cdp;
  rkCDVert *cdv;
  rkCDVert *v;
  rkCDCellDat *celld[2];
  zVec3D f0, f1, t0, t1;

  RokiLinkPtr      link0, link1;
  RokiCollisionPtr collision0, collision1;
  math::Vector3    pos, norm, pro;
  math::Vector3    force0, force1;
  math::Vector3    torque0, torque1;

  zListForEach(&fd_->cd.plist, cdp){
    if(!cdp->data.is_col) continue;

    link0 = GetRokiLinkByPathName(zName(cdp->data.cell[0]->data.shape));
    link1 = GetRokiLinkByPathName(zName(cdp->data.cell[1]->data.shape));

    if (!link0 || !link1) continue;

    collision0 = link0->GetRokiCollision();
    collision1 = link1->GetRokiCollision();

    if (!collision0 || !collision1) continue;

    DEBUG_PRINT("RokiPhysics::UpdateCollision() : cdv_num=%d\n", cdp->data.vlist.num);
    zListForEach(&cdp->data.vlist, cdv) {
      celld[0] = &cdv->data.cell->data;
      celld[1] = cdv->data.cell != cdp->data.cell[0] ? &cdp->data.cell[0]->data : &cdp->data.cell[1]->data;

      if (celld[0]->type == RK_CD_CELL_STAT) continue;
      if (celld[1]->type == RK_CD_CELL_STAT) continue;

      zListForEach(&cdp->data.vlist, v){
        zXfer3DInv(rkLinkWldFrame(celld[0]->link), cdv->data.vert, &f0);
        zXfer3DInv(rkLinkWldFrame(celld[1]->link), cdv->data.vert, &f1);
        zMulMatTVec3D(rkLinkWldAtt(celld[0]->link), &cdv->data.axis[0], &t0);
        zMulMatTVec3D(rkLinkWldAtt(celld[1]->link), &cdv->data.axis[1], &t1);
        zVec3DMulDRC(&t1, -1.0);

        pos = math::Vector3(
                zVec3DElem(v->data.vert, 0),
                zVec3DElem(v->data.vert, 1),
                zVec3DElem(v->data.vert, 2));

        norm = math::Vector3(
                zVec3DElem(&v->data.norm, 0),
                zVec3DElem(&v->data.norm, 1),
                zVec3DElem(&v->data.norm, 2));

        pro = math::Vector3(
                zVec3DElem(&v->data.pro, 0),
                zVec3DElem(&v->data.pro, 1),
                zVec3DElem(&v->data.pro, 2));

        force0 = math::Vector3(
                zVec3DElem(&f0, 0),
                zVec3DElem(&f0, 1),
                zVec3DElem(&f0, 2));

        force1 = math::Vector3(
                zVec3DElem(&f1, 0),
                zVec3DElem(&f1, 1),
                zVec3DElem(&f1, 2));
        
        torque0 = math::Vector3(
                zVec3DElem(&t0, 0),
                zVec3DElem(&t0, 1),
                zVec3DElem(&t0, 2));
        
        torque1 = math::Vector3(
                zVec3DElem(&t1, 0),
                zVec3DElem(&t1, 1),
                zVec3DElem(&t1, 2));

        Contact *contactFeedback = this->GetContactManager()->NewContact(
            collision0.get(), collision1.get(),
            this->world->GetSimTime());

        if (contactFeedback == nullptr) continue;

        contactFeedback->positions[0] = pos;
        contactFeedback->normals[0] = norm;
        contactFeedback->depths[0] = (pro - pos).GetLength();

        if (!link0->IsStatic()) { 
          contactFeedback->wrench[0].body1Force  = force0;
          contactFeedback->wrench[0].body1Torque = torque0;
        }
        if (!link1->IsStatic()) {
          contactFeedback->wrench[0].body2Force  = force1;
          contactFeedback->wrench[0].body2Torque = torque1;
        }
        ++contactFeedback->count;
      }
    }
  }
}

void RokiPhysics::UpdatePhysics()
{
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

  DEBUG_PRINT("RokiPhysics::UpdatePhysics() enter : maxStepSize=%f\n", this->GetMaxStepSize());
  DEBUG_PRINT("RokiPhysics::UpdatePhysics() enter : realTimeUpdateRate=%f\n", this->GetRealTimeUpdateRate());
  DEBUG_PRINT("RokiPhysics::UpdatePhysics() enter : targetRealTimeFactor=%f\n", this->GetTargetRealTimeFactor());
  DEBUG_PRINT("RokiPhysics::UpdatePhysics() enter : updatePeriod=%f\n", this->GetUpdatePeriod());
  DEBUG_PRINT("RokiPhysics::UpdatePhysics() enter : fd_->dt=%f\n", fd_->dt);
  DEBUG_PRINT("RokiPhysics::UpdatePhysics() enter : fd_->t=%f\n", fd_->t);

  rkFDSetDT(fd_, this->GetMaxStepSize());
  rkFDUpdate(fd_);

  Model_V models = this->world->GetModels();
  for (Model_V::iterator mi = models.begin(); mi != models.end(); ++mi) {
    Link_V links = (*mi)->GetLinks();
    for (Link_V::iterator li = links.begin(); li != links.end(); ++li) { 
      RokiLinkPtr link = boost::dynamic_pointer_cast<RokiLink>(*li);
      link->SetStateFromRoki();
    }
  }

  DEBUG_PRINT("RokiPhysics::UpdatePhysics() leave : \n");
}

void RokiPhysics::Fini()
{
  printf("RokiPhysics::Fini()\n");
  PhysicsEngine::Fini();
  ReleaseFD_();
}

void RokiPhysics::SetSeed(uint32_t _seed)
{
  DEBUG_PRINT("RokiPhysics::SetSeed() : seed=%d\n", _seed);
}

ModelPtr RokiPhysics::CreateModel(BasePtr _base)
{
  DEBUG_PRINT("RokiPhysics::CreateModel()\n");
  ModelPtr model(new RokiModel(_base));
  return model;
}

LinkPtr RokiPhysics::CreateLink(ModelPtr _parent)
{
  DEBUG_PRINT("RokiPhysics::CreateLink() : parent_model=%s\n", _parent->GetName().c_str());

  if (_parent == NULL)
    gzthrow("Link must have a parent\n");

  LinkPtr link(new RokiLink(_parent));
  link->SetWorld(_parent->GetWorld());
  return link;
}

CollisionPtr RokiPhysics::CreateCollision(const std::string &_type,
                                         LinkPtr _body)
{
  DEBUG_PRINT("RokiPhysics::CreateCollision() : type=%s\n", _type.c_str());

  CollisionPtr collision(new RokiCollision(_body));

  ShapePtr shape = CreateShape(_type, collision);
  collision->SetShape(shape);
  shape->SetWorld(_body->GetWorld());
  return collision;
}

ShapePtr RokiPhysics::CreateShape(const std::string &_type,
                                 CollisionPtr _collision)
{
  DEBUG_PRINT("RokiPhysics::CreateShape() : type=%s\n", _type.c_str());

  ShapePtr shape;
  RokiCollisionPtr collision = boost::dynamic_pointer_cast<RokiCollision>(_collision);

  if (_type == "sphere") {
    shape.reset(new RokiSphereShape(collision));
  }
  else if (_type == "plane") {
    shape.reset(new RokiPlaneShape(collision));
  }
  else if (_type == "box") {
    shape.reset(new RokiBoxShape(collision));
  }
  else if (_type == "cylinder") {
    shape.reset(new RokiCylinderShape(collision));
  }
  else if (_type == "mesh" || _type == "trimesh") {
    shape.reset(new RokiMeshShape(collision));
  }
  else if (_type == "map" || _type == "image") {
    shape.reset(new MapShape(collision)); 
  }
  else {
    gzerr << "Unable to create collision of type[" << _type << "]\n";
  }

  return shape;
}

JointPtr RokiPhysics::CreateJoint(const std::string &_type,
                                  ModelPtr _parent)
{
  DEBUG_PRINT("RokiPhysics::CreateJoint() : parent_model=%s, type=%s\n", _parent->GetName().c_str(), _type.c_str());

  JointPtr joint;

  if (_type == "prismatic") {
    joint.reset(new RokiSliderJoint(_parent));
  } else if (_type == "fixed") {
    joint.reset(new RokiFixedJoint(_parent));
  } else if (_type == "revolute") {
    joint.reset(new RokiHingeJoint(_parent));
  }
  else {
    gzerr << "Unable to create joint of type[" << _type << "]";
  }

  return joint; 
}

void RokiPhysics::SetGravity(const gazebo::math::Vector3 &_gravity)
{
  DEBUG_PRINT("RokiPhysics::SetGravity() : _gravity=%f, %f, %f)\n", _gravity.x, _gravity.y, _gravity.z);
  this->sdf->GetElement("gravity")->Set(_gravity);
}

void RokiPhysics::DebugPrint() const
{
  DEBUG_PRINT("RokiPhysics::DebugPrint()\n");
}

bool RokiPhysics::SetParam(const std::string &_key, const boost::any &_value)
{
  DEBUG_PRINT("RokiPhysics::SetParam() : key=%s, value~%s\n", _key.c_str(),any2str(_value));
  return PhysicsEngine::SetParam(_key, _value);
}

boost::any RokiPhysics::GetParam(const std::string &_key) const
{
  boost::any value;
  this->GetParam(_key, value);

  DEBUG_PRINT("RokiPhysics::GetParam() : key=%s, value~%s\n", _key.c_str(), any2str(value));
  return value;
}

bool RokiPhysics::GetParam(const std::string &_key, boost::any &_value) const
{
  DEBUG_PRINT("RokiPhysics::GetParam() : key=%s, value~%s\n", _key.c_str(), any2str(_value));
  sdf::ElementPtr rokiElem = this->sdf->GetElement("roki");
  GZ_ASSERT(rokiElem != nullptr, "ROKI SDF element does not exist");

  if (_key == "solver_type")
  {
    _value = rokiElem->Get<std::string>("solver_type");
  }
  else if (_key == "compensation")
  {
    _value = rokiElem->GetElement("constraints")->Get<double>("compensation");
  }
  else if (_key == "relaxation")
  {
    _value = rokiElem->GetElement("constraints")->Get<double>("relaxation");
  }
  else if (_key == "static_friction")
  {
    _value = rokiElem->GetElement("constraints")->Get<double>("static_friction");
  }
  else if (_key == "friction")
  {
    _value = rokiElem->GetElement("constraints")->Get<double>("friction");
  }
  else
  {
    return PhysicsEngine::GetParam(_key, _value);
  }
  return true;
}
void RokiPhysics::OnRequest(ConstRequestPtr &_msg)
{
  DEBUG_PRINT("RokiPhysics::OnRequest() : msg=(%s, %s, %f)\n", _msg->request().c_str(), "_msg->data().c_str()", _msg->dbl_data());
msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");
  std::string *serializedData = response.mutable_serialized_data();

  if (_msg->request() == "physics_info")
  {
    msgs::Physics physicsMsg;
    physicsMsg.set_type(msgs::Physics::ROKI);
    physicsMsg.set_solver_type(&current_solver_type_);
    // min_step_size is defined but not yet used
    boost::any min_step_size;
    try
    {
      if (this->GetParam("min_step_size", min_step_size))
        physicsMsg.set_min_step_size(boost::any_cast<double>(min_step_size));
    }
    catch(boost::bad_any_cast &_e)
    {
      gzerr << "Failed boost::any_cast in RokiPhysics.cc: " << _e.what();
    }
    physicsMsg.set_enable_physics(this->world->PhysicsEnabled());;
    physicsMsg.mutable_gravity()->CopyFrom(
      msgs::Convert(this->world->Gravity()));
    physicsMsg.mutable_magnetic_field()->CopyFrom(
      msgs::Convert(this->world->MagneticField()));
    physicsMsg.set_real_time_update_rate(this->realTimeUpdateRate);
    physicsMsg.set_real_time_factor(this->targetRealTimeFactor);
    physicsMsg.set_max_step_size(this->maxStepSize);

    response.set_type(physicsMsg.GetTypeName());
    physicsMsg.SerializeToString(serializedData);
    this->responsePub->Publish(response);
  }
}

void RokiPhysics::OnPhysicsMsg(ConstPhysicsPtr &_msg)
{
  DEBUG_PRINT("RokiPhysics::OnPhysicsMsg() : msg=()\n");
  PhysicsEngine::OnPhysicsMsg(_msg);

  if (_msg->has_enable_physics())
    this->world->EnablePhysicsEngine(_msg->enable_physics());

  if (_msg->has_gravity())
    this->SetGravity(msgs::ConvertIgn(_msg->gravity()));

  if (_msg->has_real_time_factor())
    this->SetTargetRealTimeFactor(_msg->real_time_factor());

  if (_msg->has_real_time_update_rate())
  {
    this->SetRealTimeUpdateRate(_msg->real_time_update_rate());
  }

  if (_msg->has_max_step_size())
  {
    this->SetMaxStepSize(_msg->max_step_size());
  }

  this->world->EnableAllModels();
}

RokiLinkPtr RokiPhysics::GetRokiLinkByPathName(const char *path_name)
{
  RokiLinkPtr result_link;

  Model_V models = this->world->GetModels();
  for (Model_V::iterator mi = models.begin(); mi != models.end(); ++mi) {
    Link_V links = (*mi)->GetLinks();
    for (Link_V::iterator li = links.begin(); li != links.end(); ++li) { 
      RokiLinkPtr roki_link = boost::dynamic_pointer_cast<RokiLink>(*li);
      if (roki_link->GetPathName() == path_name) {
        result_link = roki_link;
        break;
      }
    }
  }

  return result_link;
}
