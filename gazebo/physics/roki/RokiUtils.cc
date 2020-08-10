#include <iostream>
#include "RokiUtils.hh"
#include <stdio.h>
#include <stdarg.h>

#include <sstream>


extern "C" {
  rkFDCell* _rkFDCellPush(rkFD* fd, rkFDCell* lc);
}

using namespace gazebo;
using namespace physics;

///////////////////////////////////////////////////////////////

class RokiUtilsStaticObject {
  public:
    RokiUtilsStaticObject() {
      setvbuf(stdout, 0, _IONBF, 0);
    }
};
RokiUtilsStaticObject roki_util_static_object__;

///////////////////////////////////////////////////////////////

bool enable_roki_debug_print__ = false;

void gazebo::physics::set_roki_debug_print_flag_(const bool flag)
{
    enable_roki_debug_print__ = flag;
}

void gazebo::physics::roki_debug_print_(const char *format, ...)
{

  if (enable_roki_debug_print__ == false) return;

  va_list arg;
  va_start(arg, format);
  vprintf(format, arg);
  va_end(arg);
}

///////////////////////////////////////////////////////////////
std::string gazebo::physics::any2str_(const boost::any &a)
{
  std::stringstream ss;
  if(a.type() == typeid(int)) {
    ss << boost::any_cast<int>(a);
  }
  else if(a.type() == typeid(unsigned int)) {
    ss << boost::any_cast<unsigned int>(a);
  }
  else if(a.type() == typeid(short)) {
    ss << boost::any_cast<short>(a);
  }
  else if(a.type() == typeid(unsigned short)) {
    ss << boost::any_cast<unsigned short>(a);
  }
  else if(a.type() == typeid(char)) {
    ss << boost::any_cast<char>(a);
  }
  else if(a.type() == typeid(unsigned char)) {
    ss << boost::any_cast<unsigned char>(a);
  }
  else if(a.type() == typeid(bool)) {
    ss << boost::any_cast<bool>(a);
  }
  else if(a.type() == typeid(float)) {
    ss << boost::any_cast<float>(a);
  }
  else if(a.type() == typeid(double)) {
    ss << boost::any_cast<double>(a);
  }
  else if(a.type() == typeid(long)) {
    ss << boost::any_cast<long>(a);
  }
  else if(a.type() == typeid(long long)) {
    ss << boost::any_cast<long long>(a);
  }
  else if(a.type() == typeid(gazebo::math::Vector3)) {
    ss << boost::any_cast<gazebo::math::Vector3>(a);
  }
  else if(a.type() == typeid(ignition::math::Vector3d)) {
    ss << boost::any_cast<ignition::math::Vector3d>(a);
  }
  else {
    ss << "cannot convert " << a.type().name() << " to std::string...";
    std::cerr << ss.str() << std::endl;
  }

  return ss.str();
}

std::string gazebo::physics::conv2str_(const math::Angle &a)
{
  std::stringstream ss;
  ss.precision(5);
  ss << a;
  return ss.str();
}

std::string gazebo::physics::conv2str_(const math::Vector3 &v)
{
  std::stringstream ss;
  ss.precision(5);
  ss << v;
  return ss.str();
}

std::string gazebo::physics::conv2str_(const math::Quaternion &q)
{
  std::stringstream ss;
  ss.precision(5);
  ss << q;
  return ss.str();
}

std::string gazebo::physics::conv2str_(const math::Matrix3 &m)
{
  std::stringstream ss;
  ss.precision(5);
  ss << m;
  return ss.str();
}

std::string gazebo::physics::conv2str_(const math::Matrix4 &m)
{
  std::stringstream ss;
  ss.precision(5);
  ss << m;
  return ss.str();
}

std::string gazebo::physics::conv2str_(const math::Pose &p)
{
  std::stringstream ss;
  ss.precision(5);
  ss << p;
  return ss.str();
}

std::string gazebo::physics::conv2str_(const zFrame3D* f)
{
  std::stringstream ss;
  ss.precision(5);

  ss << "zFrame3D[pos=(";
  ss << zVec3DElem(zFrame3DPos(f), 0) << " ";
  ss << zVec3DElem(zFrame3DPos(f), 1) << " ";
  ss << zVec3DElem(zFrame3DPos(f), 2) << " ";
  ss << "),att=(";
  ss << zMat3DElem(zFrame3DAtt(f), 0, 0) << " ";
  ss << zMat3DElem(zFrame3DAtt(f), 1, 0) << " ";
  ss << zMat3DElem(zFrame3DAtt(f), 2, 0) << " ";
  ss << zMat3DElem(zFrame3DAtt(f), 0, 1) << " ";
  ss << zMat3DElem(zFrame3DAtt(f), 1, 1) << " ";
  ss << zMat3DElem(zFrame3DAtt(f), 2, 1) << " ";
  ss << zMat3DElem(zFrame3DAtt(f), 0, 2) << " ";
  ss << zMat3DElem(zFrame3DAtt(f), 1, 2) << " ";
  ss << zMat3DElem(zFrame3DAtt(f), 2, 2) << " ";
  ss << ")]";

  return ss.str();
}

math::Pose gazebo::physics::conv2pose(const zVec dis, const int chain_offset_idx)
{
  math::Pose pose;

  math::Vector3 pos;
  math::Quaternion q;

  pos.x   = zVecElem(dis, chain_offset_idx + 0);
  pos.y   = zVecElem(dis, chain_offset_idx + 1);
  pos.z   = zVecElem(dis, chain_offset_idx + 2);

  math::Vector3 aa(
    zVecElem(dis, chain_offset_idx + 3),
    zVecElem(dis, chain_offset_idx + 4),
    zVecElem(dis, chain_offset_idx + 5));

  double angle = aa.GetLength(); // norm
  printf("angle=%f\n", angle);

  if (angle > 0.00001) {
    q.SetFromAxis(aa.x/angle, aa.y/angle, aa.z/angle, angle);
  }

  pose.pos = pos;
  pose.rot = q;

  return pose;
}

math::Pose gazebo::physics::conv2pose(const zFrame3D *f)
{
  // // zeo/zeo_frame.h
  // typedef struct{
  //   zVec3D pos;
  //   zMat3D att;
  // } zFrame3D;
  // 
  // #define zFrame3DPos(f) ( &(f)->pos )
  // #define zFrame3DAtt(f) ( &(f)->att )
  //
  // // zeo/zeo_vec3d.h
  // typedef struct{
  //     double e[3];
  // } zVec3D;
  //
  // /* for backward compatibility */
  // #define zVec3DElem(v,i)      ( (v)->e[i] )
  // #define zVec3DSetElem(v,i,x) ( zVec3DElem(v,i) = (x) )
  //
  // // zeo/zeo_mat3d.h
  // typedef union{
  //   double e[3][3]; /*!< 3x3 matrix */
  //   zVec3D v[3];    /*!< 3 column vectors */
  //   double c[9];    /*!< 9 components */
  // } zMat3D;
  //
  // /* for backward compatibility */
  // #define zMat3DElem(m,r,c)      (m)->e[(c)][(r)]
  // #define zMat3DSetElem(m,r,c,x) ( zMat3DElem(m,r,c) = (x) )
  // #define zMat3DElem9(m,i)       ( (m)->c[i] )
  // #define zMat3DSetElem9(m,i,c)  ( zMat3DElem9(m,i) = (c) )
  // #define zMat3DVec(m,i)         ( &(m)->v[(i)] )
  // #define zMat3DSetVec(m,i,v)    zVec3DCopy(v,zMat3DVec(m,i)

  math::Pose pose;
  math::Vector3 pos;
  math::Matrix4 m;

  pos.x = zVec3DElem(zFrame3DPos(f), 0);
  pos.y = zVec3DElem(zFrame3DPos(f), 1);
  pos.z = zVec3DElem(zFrame3DPos(f), 2);

  m.Set(
    zMat3DElem(zFrame3DAtt(f), 0, 0),
    zMat3DElem(zFrame3DAtt(f), 1, 0),
    zMat3DElem(zFrame3DAtt(f), 2, 0),
    0,

    zMat3DElem(zFrame3DAtt(f), 0, 1),
    zMat3DElem(zFrame3DAtt(f), 1, 1),
    zMat3DElem(zFrame3DAtt(f), 2, 1),
    0,

    zMat3DElem(zFrame3DAtt(f), 0, 2),
    zMat3DElem(zFrame3DAtt(f), 1, 2),
    zMat3DElem(zFrame3DAtt(f), 2, 2),
    0,

    0,
    0,
    0,
    1
  );

  pose.pos = pos;
  pose.rot = m.Inverse().GetRotation();

  return pose;
}

void gazebo::physics::pose2zFrame3D(const math::Pose &pose, zFrame3D *f)
{
    zVec3DCreate(zFrame3DPos(f), pose.pos.x, pose.pos.y, pose.pos.z);

    math::Matrix3 m = pose.rot.GetAsMatrix3();

    zMat3DCreate(
        zFrame3DAtt(f),
        m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2]);
}

void gazebo::physics::pose2rklinkdis(const math::Pose &pose, rkFDCell *lc, rkChain *chain, const int &rklink_idx)
{
    zVec dis = zVecAlloc(rkChainJointSize(chain));
    rkChainGetJointDisAll(chain, dis);

    int chain_offset_idx = rkChainLinkOffset(chain, rklink_idx);

    math::Vector3 angle = pose.rot.GetAsEuler();

    zVecElem(dis, chain_offset_idx + 0) = pose.pos.x;
    zVecElem(dis, chain_offset_idx + 1) = pose.pos.y;
    zVecElem(dis, chain_offset_idx + 2) = pose.pos.z;
    zVecElem(dis, chain_offset_idx + 3) = angle.x;
    zVecElem(dis, chain_offset_idx + 4) = angle.y;
    zVecElem(dis, chain_offset_idx + 5) = angle.z;

    // apply chain displacementj
    rkFDChainSetDis(lc, dis);

    zVecFree(dis);
}

///////////////////////////////////////////////////////////////
DummyRokiObject::DummyRokiObject(rkFD *fd_)
{
  // cell
  lc = zAlloc(rkFDCell, 1);

  // chain
  chain = &(lc->data.chain);
  rkChainInit(chain);
  zArrayAlloc(&(chain->link), rkLink, 1);

  // link
  rklink = rkChainLink(chain, 0);
  rkLinkInit(rklink);

  zVec3DCreate(rkLinkOrgPos(rklink), 0, 0, 0);
  zMat3DCreate(
      rkLinkOrgAtt(rklink),
      1,  0,  0,
      0,  1,  0,
      0,  0,  1);

  // mass
  rkLinkSetMass(rklink, 1.0);
  zVec3DCreate(rkLinkCOM(rklink), 0, 0, 0);
  zMat3DCreate(
      rkLinkInertia(rklink),
      0.01,    0,    0,
      0,    0.01,    0,
      0,       0, 0.01);

  // joint
  joint = rkLinkJoint(rklink);
  rkJointCreate(joint, RK_JOINT_FLOAT);

  rkChainSetMass(chain, 1.0);
  rkChainSetOffset(chain);
  rkChainUpdateFK(chain);
  rkChainUpdateID(chain);

  _rkFDCellPush(fd_, lc);
}

DummyRokiObject::~DummyRokiObject()
{
}
