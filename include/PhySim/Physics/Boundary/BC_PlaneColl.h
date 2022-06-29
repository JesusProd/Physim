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

#include <PhySim/Physics/Boundary/BCondition.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class BC_PlaneColl : public BCondition {
 protected:
  Vector3d m_vpoints;
  Vector3d m_vn;
  Real m_tol;
  bool m_isSoft;

  PtrS<GeometryFilter_PlaneDistNode> m_pFilter;

 public:
  BC_PlaneColl(Simulable* pModel,
               const Vector3d& vp = Vector3d(0.0, 0.0, 0.0),
               const Vector3d& vn = Vector3d(0.0, 1.0, 0.0),
               Real tol = 0,
               bool isSoft = true);
  virtual ~BC_PlaneColl(void);

  inline virtual string Name() const { return "CollPlane"; };

  virtual void Init() override;
  virtual void Update() override;
};
}  // namespace PhySim
