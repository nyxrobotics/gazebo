#include <sstream>

#include "gazebo/common/Console.hh"
#include "ignition/math/Box.hh"

#include "gazebo/physics/roki/RokiSurfaceParams.hh"
//#include "gazebo/physics/roki/RokiPhysics.hh"
#include "gazebo/physics/roki/RokiModel.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/physics/roki/RokiCollision.hh"
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
  this->Fini();
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

  // Force max correcting velocity to zero for certain collision entities
  if (
	this->IsStatic()
	|| this->shape->HasType(Base::HEIGHTMAP_SHAPE)
	|| this->shape->HasType(Base::MAP_SHAPE)
	){
    this->GetRokiSurface()->maxVel = 0.0;
  }

}

void RokiCollision::OnPoseChange()
{
  DEBUG_PRINT("RokiCollision::OnPoseChange() : name=%s\n", GetPathName().c_str());
}

void RokiCollision::SetCategoryBits(unsigned int _bits)
{

  gzwarn << "[Roki]SetCategoryBits is not implemented" << std::endl;
  DEBUG_PRINT("RokiCollision::SetCategoryBits() : name=%s bits=%08x\n", GetPathName().c_str(), _bits);
  this->categoryBits = _bits;
}

void RokiCollision::SetCollideBits(unsigned int _bits)
{
  DEBUG_PRINT("RokiCollision::SetCollideBits() : name=%s bits=%08x\n", GetPathName().c_str(), _bits);
  gzwarn << "Roki does not provide RokiCollision::SetCollideBits()\n";
  this->collideBits = _bits;
}

unsigned int RokiCollision::GetCategoryBits() const
{
  gzwarn << "[Roki]GetCategoryBits is not implemented" << std::endl;
  DEBUG_PRINT("RokiCollision::GetCategoryBits() : name=%s\n", GetPathName().c_str());
  return this->categoryBits;
}

unsigned int RokiCollision::GetCollideBits() const
{
  gzwarn << "[Roki]GetCollideBits is not implemented" << std::endl;
  DEBUG_PRINT("RokiCollision::GetCollideBits() : name=%s\n", GetPathName().c_str());
  return this->collideBits;
}

//copied from ODECollision.cpp
ignition::math::Box RokiCollision::BoundingBox() const
{
	  gzwarn << "Roki does not provide bounding box info.\n";
	  dReal aabb[6];
	  memset(aabb, 0, 6 * sizeof(dReal));

//	  dGeomGetAABB(this->collisionId, aabb);

	  ignition::math::Box box(
	      ignition::math::Vector3d(aabb[0], aabb[2], aabb[4]),
	      ignition::math::Vector3d(aabb[1], aabb[3], aabb[5]));

	  return box;
}

std::string RokiCollision::GetPathName() const
{
  DEBUG_PRINT("RokiCollision::GetPathName() \n");
  RokiLinkPtr l = boost::dynamic_pointer_cast<RokiLink>(GetLink());
  return l->GetPathName();
}

RokiSurfaceParamsPtr RokiCollision::GetRokiSurface() const
{
  return boost::dynamic_pointer_cast<RokiSurfaceParams>(this->surface);
}
