#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Mesh.hh"

#include "gazebo/physics/roki/RokiLink.hh"
#include "gazebo/physics/roki/RokiCollision.hh"
#include "gazebo/physics/roki/RokiMeshShape.hh"
#include "gazebo/physics/roki/RokiPhysics.hh"
#include "gazebo/physics/roki/RokiUtils.hh"

using namespace gazebo;
using namespace physics;

RokiMeshShape::RokiMeshShape(CollisionPtr _parent) : MeshShape(_parent)
{
  DEBUG_PRINT("RokiMeshShape::RokiMeshShape()\n");
}

RokiMeshShape::~RokiMeshShape()
{
  DEBUG_PRINT("RokiMeshShape::~RokiMeshShape()\n");
}

void RokiMeshShape::Update()
{
  DEBUG_PRINT("RokiMeshShape::Update()\n");
  MeshShape::Update();
}

void RokiMeshShape::Load(sdf::ElementPtr _sdf)
{
  DEBUG_PRINT("RokiMeshShape::Load()\n");
  MeshShape::Load(_sdf);
}

void RokiMeshShape::Init()
{
  DEBUG_PRINT("RokiMeshShape::Init()\n");
  MeshShape::Init();

  if (this->submesh)
  {
     InitMesh_(this->submesh, this->sdf->Get<math::Vector3>("scale"));
  }
  else
  {
     InitMesh_(this->mesh, this->sdf->Get<math::Vector3>("scale"));
  }
}

void RokiMeshShape::InitMesh_(const common::SubMesh *_subMesh,  const math::Vector3 &_scale)
{
  RokiCollisionPtr collision = boost::dynamic_pointer_cast<RokiCollision>(this->collisionParent);
  RokiLinkPtr link = boost::dynamic_pointer_cast<RokiLink>(this->collisionParent->GetLink());

  DEBUG_PRINT("RokiMeshShape::InitMesh_() link=%s, name=%s, type=%s\n", link->GetName().c_str(), GetName().c_str(), "_subMesh");

  float *vertices = NULL;
  int   *indices  = NULL;

  _subMesh->FillArrays(&vertices, &indices);

  this->CreateMesh_(
    vertices, _subMesh->GetVertexCount(),
    indices, _subMesh->GetIndexCount(),
    _scale);

  delete [] vertices;
  delete [] indices;
}

void RokiMeshShape::InitMesh_(const common::Mesh *_mesh, const math::Vector3 &_scale)
{
  RokiCollisionPtr collision = boost::dynamic_pointer_cast<RokiCollision>(this->collisionParent);
  RokiLinkPtr link = boost::dynamic_pointer_cast<RokiLink>(this->collisionParent->GetLink());

  DEBUG_PRINT("RokiMeshShape::InitMesh_() link=%s, name=%s, type=%s\n", link->GetName().c_str(), GetName().c_str(),"_mesh");

  float *vertices = NULL;
  int   *indices  = NULL;

  _mesh->FillArrays(&vertices, &indices);

  this->CreateMesh_(
    vertices, _mesh->GetVertexCount(),
    indices, _mesh->GetIndexCount(),
    _scale);

  delete [] vertices;
  delete [] indices;
}

void RokiMeshShape::CreateMesh_(float *_vertices, unsigned int _numVertices, int *_indices, unsigned int _numIndices, const math::Vector3 &_scale)
{
  RokiCollisionPtr collision = boost::dynamic_pointer_cast<RokiCollision>(this->collisionParent);
  RokiLinkPtr link = boost::dynamic_pointer_cast<RokiLink>(this->collisionParent->GetLink());

  DEBUG_PRINT("RokiMeshShape:CreatetMesh_() link=%s, name=%s, _numVertices=%d, _numIndices,=%d\n", link->GetName().c_str(), GetName().c_str(), _numVertices, _numIndices);

  zShape3D *shape = zAlloc(zShape3D, 1);
  zShape3DInit(shape);
  zShape3DType(shape) = ZSHAPE_PH; // polyhedron

  shape->com = &zprim_ph3d_com;    // function valiable

  zPH3D* ph = (zPH3D*)&shape->body;
  zPH3DInit(ph);

  zPH3DAlloc(ph, _numVertices, _numIndices/3);

  for (unsigned int i = 0; i < _numVertices; ++i) {
    zVec3D* v;
    v = zPH3DVert(ph, i);
    zVec3DSetElem(v, zX, _vertices[i * 3 + 0] * _scale.x);
    zVec3DSetElem(v, zY, _vertices[i * 3 + 1] * _scale.y);
    zVec3DSetElem(v, zZ, _vertices[i * 3 + 2] * _scale.z);
  }

  for (unsigned int i = 0; i < _numIndices/3; ++i) {
    zTri3DCreate(
      zPH3DFace(ph, i),
      zPH3DVert(ph, _indices[i * 3 + 0]),
      zPH3DVert(ph, _indices[i * 3 + 1]),
      zPH3DVert(ph, _indices[i * 3 + 2]));
  }

  zAABox3D aabb;
  zAABB(&aabb, zShape3DVertBuf(shape), zShape3DVertNum(shape), NULL);
  zAABox3DToBox3D(&aabb, zShape3DBB(shape));

  zNameSet(shape, const_cast<char*>(link->GetPathName().c_str()));

  collision->shape_ = shape;
}
