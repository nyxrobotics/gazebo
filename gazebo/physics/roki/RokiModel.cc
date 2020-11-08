#include "gazebo/physics/World.hh"
#include "gazebo/physics/roki/RokiPhysics.hh"
#include "gazebo/physics/roki/RokiModel.hh"
#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/physics/roki/RokiUtils.hh"

#include "roki/rk_link.h"

using namespace gazebo;
using namespace physics;

extern "C" {
    rkFDCell* _rkFDCellPush(rkFD* fd, rkFDCell* lc);
}

RokiModel::RokiModel(BasePtr _parent)
  : Model(_parent), cell_(nullptr), chain_(nullptr), link_id_counter_(0), self_collide_(true)
{
  DEBUG_PRINT("RokiModel::RokiModel()\n");
}

RokiModel::~RokiModel()
{
  DEBUG_PRINT("RokiModel::~RokiModel()\n");
}

void RokiModel::Init()
{
  int link_size = GetLinks().size();
  DEBUG_PRINT("RokiModel::Init() enter : name=%s, link_size=%d\n", GetName().c_str(), link_size);

  std::string modelName = this->GetName();

  cell_  = zAlloc(rkFDCell, 1);
  chain_ = &(cell_->data.chain);
  rkChainInit(chain_);
  zNameSet(chain_, const_cast<char*>(modelName.c_str()));

  Link_V ls = GetLinks();

  zArrayAlloc(&(chain_->link), rkLink, link_size * 2);

  // travers link->InitRoki()
  DEBUG_PRINT("RokiModel::Init() call Model::Init(): name=%s\n", GetName().c_str());
  Model::Init();
  DEBUG_PRINT("RokiModel::Init() return Model::Init(): name=%s\n", GetName().c_str());

  double mass = 0.0;
  for (Link_V::iterator li = ls.begin(); li != ls.end(); ++li) { 
    RokiLinkPtr link = boost::dynamic_pointer_cast<RokiLink>(*li);
    mass += link->GetInertial()->Mass();
  }
  rkChainSetMass(chain_, mass); 

  rkChainSetOffset(chain_);
  rkChainUpdateFK(chain_);
  rkChainUpdateID(chain_);

  rkFD *fd = GetRkFD();

  _rkFDCellPush(fd, cell_);
  rkFDUpdateInit(fd);//MESSAGE:warning: cannot divid by zero value(zVec3DDiv).

  if (!self_collide_) {
    DEBUG_PRINT("RokiModel::Init() rkCDPairChainUnreg() : name=%s\n", GetName().c_str());
    rkCDPairChainUnreg(&fd->cd, &cell_->data.chain);
  }

  // update displacement
  DEBUG_PRINT("RokiModel::InitRoki() force links->OnPoseChange() : name=%s\n", GetName().c_str());
  for (Link_V::iterator li = ls.begin(); li != ls.end(); ++li) { 
    RokiLinkPtr link = boost::dynamic_pointer_cast<RokiLink>(*li);
    link->OnPoseChange();
  }

  DEBUG_PRINT("RokiModel::Init() leave : name=%s\n", GetName().c_str());
}

void RokiModel::Fini() 
{
  DEBUG_PRINT("RokiModel::Fini() : name=%s\n", GetName().c_str());

  Model::Fini();
}

void RokiModel::DisableSelfCollide()
{
  self_collide_ = false;
}

rkFD* RokiModel::GetRkFD() const
{
  return (boost::dynamic_pointer_cast<RokiPhysics>(this->GetWorld()->Physics()))->fd_;
}
