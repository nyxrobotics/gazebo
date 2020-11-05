#include <sstream>

#include "gazebo/common/Console.hh"
#include "ignition/math/Box.hh"

#include "gazebo/physics/roki/RokiModel.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/physics/roki/RokiCollision.hh"
#include "gazebo/physics/roki/RokiSurfaceParams.hh"
#include "gazebo/physics/roki/RokiUtils.hh"

using namespace gazebo;
using namespace physics;

RokiCollision::RokiCollision(LinkPtr _link) : Collision(_link), shape_(nullptr)
{
  DEBUG_PRINT("RokiCollision::RokiCollision()\n");
  this->SetName("Roki_Collision");
  this->surface.reset(new RokiSurfaceParams());
}

RokiCollision::~RokiCollision()
{
  DEBUG_PRINT("RokiCollision::~RokiCollision()\n");
}

void RokiCollision::Load(sdf::ElementPtr _sdf)
{
  DEBUG_PRINT("RokiCollision::Load()\n");
  Collision::Load(_sdf);

  if (this->IsStatic())
  {
    this->SetCategoryBits(GZ_FIXED_COLLIDE);
    this->SetCollideBits(~GZ_FIXED_COLLIDE);
  }
}

void RokiCollision::Init()
{
  DEBUG_PRINT("RokiCollision::Init() : name=%s\n", GetPathName().c_str());
  Collision::Init();
}

void RokiCollision::Fini()
{
  DEBUG_PRINT("RokiCollision::Fini() : name=%s\n", GetPathName().c_str());
  Collision::Fini();
}

void RokiCollision::OnPoseChange()
{
  DEBUG_PRINT("RokiCollision::OnPoseChange() : name=%s\n", GetPathName().c_str());
}

void RokiCollision::SetCategoryBits(unsigned int _bits)
{
  DEBUG_PRINT("RokiCollision::SetCategoryBits() : name=%s bits=%08x\n", GetPathName().c_str(), _bits);
  gzerr << "Roki does not provide RokiCollision::SetCategoryBits()";
}

void RokiCollision::SetCollideBits(unsigned int _bits)
{
  DEBUG_PRINT("RokiCollision::SetCollideBits() : name=%s bits=%08x\n", GetPathName().c_str(), _bits);
  gzerr << "Roki does not provide RokiCollision::SetCollideBits()\n";
}

unsigned int RokiCollision::GetCategoryBits() const
{
  DEBUG_PRINT("RokiCollision::GetCategoryBits() : name=%s\n", GetPathName().c_str());
  return 0;
}

unsigned int RokiCollision::GetCollideBits() const
{
  DEBUG_PRINT("RokiCollision::GetCollideBits() : name=%s\n", GetPathName().c_str());
  return 0;
}

ignition::math::Box RokiCollision::BoundingBox() const
{
  DEBUG_PRINT("RokiCollision::BoundingBox() : name=%s\n", GetPathName().c_str());

  ignition::math::Box result;
  gzerr << "Roki does not provide bounding box info.\n";
  return result;
}

std::string RokiCollision::GetPathName() const
{ 
  RokiLinkPtr l = boost::dynamic_pointer_cast<RokiLink>(GetLink());
  return l->GetPathName();
}
