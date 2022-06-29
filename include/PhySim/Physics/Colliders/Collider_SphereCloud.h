//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>

#include <PhySim/Geometry/Partition/BVHTree.h>
#include <PhySim/Geometry/Samplers/SurfaceSampler.h>
#include <PhySim/Physics/Colliders/Collider.h>

namespace PhySim {
class Particle;
class Geometry;
class Embedding;

class Collider_SphereCloud : public Collider {
 public:
  struct Point {
    Point(Embedding* Position, Real Radius);
    Embedding* Position;
    Real Radius;
  };

  struct Object : public BVHTree::Object {
    size_t NumElements() const override;
    AlignedBox3d BBox(size_t index) const override;
    Collider_SphereCloud* pInstance;
  };

 public:
  Collider_SphereCloud() = default;
  Collider_SphereCloud(Geometry* pMesh, const vector<SurfaceSample>& vSamples);
  Collider_SphereCloud(Simulable* pModel,
                       const vector<SurfaceSample>& vSamples);

 public:
  string Name() const override;
  ColliderType Type() const override;

 public:
  void Init(Geometry* pMesh, const vector<SurfaceSample>& vSamples);
  void Init(Simulable* pModel, const vector<SurfaceSample>& vSamples);

 public:
  void Update() override;

 public:
  const BVHTree& Tree() const;
  const vector<Point>& Points() const;

 protected:
  BVHTree m_BVHTree;
  Object m_BVHObject;
  vector<PtrS<Embedding>> m_vEmbeddings;
  vector<Point> m_vPoints;
};
}  // namespace PhySim