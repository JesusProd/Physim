//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/ConstraintSet_NodeInCurve.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

ConstraintSet_NodeInCurve::ConstraintSet_NodeInCurve(Simulable* pModel,
                                                     bool isSoft,
                                                     KEleParticle3D* pDOF,
                                                     PtrS<Curve> pCurve)
    : ConstraintSet(pModel, isSoft) {
  this->m_vDoFs.resize(1);
  this->m_vDoFs[0] = pDOF;

  this->m_pCurve = pCurve;

  this->m_size = 3;
  this->m_vvalues.resize(3);
  this->m_mJacobian.resize(3, 3);
  this->m_vHessian.resize(1);
  this->m_vHessian.clear();

  this->m_type = Type::EQ;
}

ConstraintSet_NodeInCurve::~ConstraintSet_NodeInCurve(void) {
  // Nothing to do here...
}

void ConstraintSet_NodeInCurve::Init() {
  // Nothing to do here...
}

void ConstraintSet_NodeInCurve::ComputeAndStore_Values() {
  Vector3d vx = this->m_vDoFs[0]->GetValue();

  // Update cached

  vector<PtrS<Embedding>> ve;

  this->m_pCurve->ComputeProjection(vx.transpose(), ve, Tag::Tag_Position_0);

  const Edge* pEdge = static_cast<const Edge*>(ve[0]->Master());
  this->m_v0 = pEdge->GetTail()->Traits().Vector3d(Tag::Tag_Position_0);
  this->m_v1 = pEdge->GetHead()->Traits().Vector3d(Tag::Tag_Position_0);
  this->m_vt = pEdge->Tangent(Tag::Tag_Position_0);
  this->m_mP = Matrix3d::Identity() - m_vt * m_vt.transpose();

  // Compute constraint

  this->m_vvalues = this->m_mP * (vx - m_v0);
}

void ConstraintSet_NodeInCurve::ComputeAndStore_Jacobian() {
  this->m_mJacobian.block(0, 0, 3, 3) = this->m_mP;
}

void ConstraintSet_NodeInCurve::ComputeAndStore_Hessian() {
  // Zero: nothing to do here
}

void ConstraintSet_NodeInCurve::ProjectConstraint() {
  Vector3d vx = this->m_vDoFs[0]->GetValue();

  vx = vx - m_mP * (vx - this->m_v0);
}

}  // namespace PhySim