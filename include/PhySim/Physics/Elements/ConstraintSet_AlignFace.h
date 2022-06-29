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

#include <PhySim/Physics/Elements/ConstraintSet.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Face;
class Simulable;

class ConstraintSet_AlignFace : public ConstraintSet {
 protected:
  Vector3d m_vn;

 public:
  ConstraintSet_AlignFace(Simulable* pModel,
                          Face* pFace,
                          const Vector3d& vn,
                          bool isSoft = false);

  virtual ~ConstraintSet_AlignFace(void);

  inline virtual Vector3d& Normal() { return this->m_vn; }

  virtual void Init();

  virtual void ProjectConstraint();
  virtual void ComputeAndStore_Values();
  virtual void ComputeAndStore_Jacobian();
};
}  // namespace PhySim