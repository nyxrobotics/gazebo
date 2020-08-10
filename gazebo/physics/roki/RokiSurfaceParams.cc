#include "gazebo/common/Console.hh"
#include "gazebo/physics/roki/RokiSurfaceParams.hh"
#include "gazebo/physics/roki/RokiUtils.hh"

using namespace gazebo;
using namespace physics;

RokiSurfaceParams::RokiSurfaceParams() : SurfaceParams()
{
  DEBUG_PRINT("RokiSurfaceParams::RokiSurfaceParams()\n");
}

RokiSurfaceParams::~RokiSurfaceParams()
{
  DEBUG_PRINT("RokiSurfaceParams::~RokiSurfaceParams()\n");
}

void RokiSurfaceParams::Load(sdf::ElementPtr _sdf)
{
  DEBUG_PRINT("RokiSurfaceParams::Load()\n");

  SurfaceParams::Load(_sdf);

  if (!_sdf) {
    gzerr << "Surface _sdf is NULL" << std::endl;
  }

  sdf::ElementPtr e_friction = _sdf->GetElement("friction");
  if (!e_friction) {
    gzerr << "Surface friction sdf member is NULL" << std::endl;
    return;
  }

  sdf::ElementPtr e_friction_ode = e_friction->GetElement("ode");
  if (!e_friction_ode) {
    gzerr << "Surface friction ode sdf member is NULL" << std::endl;
    return;
  }

  e_friction_ode->Get<double>("mu");
  e_friction_ode->Get<double>("mu2");
}

void RokiSurfaceParams::FillMsg(msgs::Surface &_msg)
{
  DEBUG_PRINT("RokiSurfaceParams::FillMsg()\n");
  SurfaceParams::FillMsg(_msg);
  _msg.mutable_friction()->set_mu(1.0);
  _msg.mutable_friction()->set_mu2(1.0);
}

void RokiSurfaceParams::ProcessMsg(const msgs::Surface &_msg)
{
  DEBUG_PRINT("RokiSurfaceParams::ProcessMsg()\n");
  SurfaceParams::ProcessMsg(_msg);
  if (_msg.has_friction()) { 
    if (_msg.friction().has_mu()) {
      DEBUG_PRINT("RokiSurfaceParams::ProcessMsg() : mu=%f\n", _msg.friction().mu());
    }
    if (_msg.friction().has_mu2()) {
      DEBUG_PRINT("RokiSurfaceParams::ProcessMsg() : mu=%f\n", _msg.friction().mu2());
    }
  }
}
