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

#include <PhySim/Physics/Elements/EnergyElement.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Face_Tri;
class Simulable;

class EnergyElement_AlignFace : public EnergyElement {
 protected:
  Face_Tri* m_pFace;
  Vector3d m_vnt;
  Real m_kA;

 public:
  EnergyElement_AlignFace(Simulable* pModel, Face_Tri* pFace);
  virtual ~EnergyElement_AlignFace(void);

  virtual void Init();

  virtual Real GetStiffness() const { return this->m_kA; }
  virtual void SetStiffness(Real kA) { this->m_kA = kA; }

  virtual const Vector3d& GetTargetNormal() const { return this->m_vnt; }
  virtual void SetTargetNormal(const Vector3d& vnt) { this->m_vnt = vnt; }

  virtual void ComputeAndStore_Energy_Internal();
  virtual void ComputeAndStore_Gradient_Internal();
  virtual void ComputeAndStore_Hessian_Internal();
};

}  // namespace PhySim
