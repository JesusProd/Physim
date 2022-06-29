//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Colliders/Collider_SphereCloud.h>
#include <PhySim/Physics/Simulables/Simulable.h>
#include <PhySim/Physics/Simulables/Simulable_Composite.h>
#include <PhySim/Physics/Simulables/Simulable_Mesh.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
Collider_SphereCloud::Point::Point(Embedding* Position, Real Radius)
    : Position(Position), Radius(Radius) {}

size_t Collider_SphereCloud::Object::NumElements() const {
  return pInstance->m_vPoints.size();
}

AlignedBox3d Collider_SphereCloud::Object::BBox(size_t index) const {
  const Point& rPoint = pInstance->m_vPoints[index];
  const Vector3d vPosition = rPoint.Position->InterpolateValue(Tag::Tag_Position_X);
  const Real Radius = rPoint.Radius;

  const Vector3d vMinAABB = vPosition - Vector3d::Constant(Radius);
  const Vector3d vMaxAABB = vPosition + Vector3d::Constant(Radius);

  return AlignedBox3d(vMinAABB, vMaxAABB);
};

Collider_SphereCloud::Collider_SphereCloud(
    Geometry* pGeom,
    const vector<SurfaceSample>& vSamples) {
  Init(pGeom, vSamples);
  this->m_pModel = NULL;
}

Collider_SphereCloud::Collider_SphereCloud(
    Simulable* pModel,
    const vector<SurfaceSample>& vSamples) {
  Init(pModel, vSamples);
}

void Collider_SphereCloud::Init(Simulable* pModel,
                                const vector<SurfaceSample>& vSamples) {
  this->Init(pModel->Geometries()[0].get(), vSamples);
  this->m_pModel = dynamic_cast<Simulable*>(pModel);
}

void Collider_SphereCloud::Init(Geometry* pGeom,
                                const vector<SurfaceSample>& vSamples) {
  // TODO: Make this generic
  Mesh* pMesh = dynamic_cast<Mesh*>(pGeom);
  assert(pMesh && "Expected a mesh pointer");

  // Build statistics object to obtain the sampling points in matrix format.
  // Choose the radius based on the mean distance between adjacent points.
  SurfaceSamplingStatistics Stats(vSamples);
  MatrixXd mPoints = Stats.Points();
  Real Radius = 0.5 * Stats.MeanDistance();

  IOUtils::logTrace(Verbosity::V1_Default,
                    "\n[TRACE] Collider radius set to %f", Radius);

  // Compute the embeddings with respect to the simulable objects.
  pMesh->ComputeEmbedding(mPoints, m_vEmbeddings, Tag::Tag_Position_0);

  // Build point cloud based upon this information.
  m_vPoints.reserve(m_vEmbeddings.size());

  for (size_t i = 0; i < m_vEmbeddings.size(); ++i)
    m_vPoints.emplace_back(m_vEmbeddings[i].get(), Radius);

  // Build BVH-Tree.
  m_BVHObject.pInstance = this;
  m_BVHTree.Build(&m_BVHObject, {});
}

string Collider_SphereCloud::Name() const {
  return "Collider_SphereCloud";
}

ColliderType Collider_SphereCloud::Type() const {
  return ColliderType_SphereCloud;
}

void Collider_SphereCloud::Update() {
  m_BVHTree.Refit();
}

const BVHTree& Collider_SphereCloud::Tree() const {
  return m_BVHTree;
}

const vector<Collider_SphereCloud::Point>& Collider_SphereCloud::Points()
    const {
  return m_vPoints;
}
}  // namespace PhySim