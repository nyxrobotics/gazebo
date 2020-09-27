#ifndef _ROKIMESHSHAPE_HH_
#define _ROKIMESHSHAPE_HH_

#include "gazebo/physics/MeshShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    class GZ_PHYSICS_VISIBLE RokiMeshShape : public MeshShape
    {
      public: explicit RokiMeshShape(CollisionPtr _parent);
      public: virtual ~RokiMeshShape();
      public: virtual void Load(sdf::ElementPtr _sdf);
      public: virtual void Init();
      public: virtual void Update();
      private:
        void InitMesh_(const common::SubMesh *_subMesh,  const ignition::math::Vector3d &_scale);
        void InitMesh_(const common::Mesh *_mesh, const ignition::math::Vector3d &_scale);
        void CreateMesh_(float *_vertices, unsigned int _numVertices, int *_indices, unsigned int _numIndices, const ignition::math::Vector3d &_scale);
    };
  }
}
#endif
