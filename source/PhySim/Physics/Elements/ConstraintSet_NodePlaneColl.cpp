//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/ConstraintSet_NodePlaneColl.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Physics/DoFSet.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

ConstraintSet_NodePlaneColl::ConstraintSet_NodePlaneColl(Simulable* pModel,
                                                         bool isSoft,
                                                         KEleParticle3D* pDOF,
                                                         const Vector3d& vp,
                                                         const Vector3d& vn,
                                                         Real tol)
    : ConstraintSet(pModel, isSoft) {
  this->m_vDoFs.resize(1);
  this->m_vDoFs[0] = pDOF;

  this->m_size = 1;
  this->m_vvalues.resize(1);
  this->m_mJacobian.resize(1, 3);
  this->m_vHessian.resize(1);
  this->m_vHessian.clear();

  this->m_type = Type::LEQ;

  this->m_vpoints = vp;
  this->m_vn = vn;
  this->m_tol = tol;
}

ConstraintSet_NodePlaneColl::~ConstraintSet_NodePlaneColl(void) {
  // Nothing to do here...
}

void ConstraintSet_NodePlaneColl::Init() {
  // Nothing to do here...
}

void ConstraintSet_NodePlaneColl::ProjectConstraint() {
  Vector3d vx = this->m_vDoFs[0]->GetValue();
  this->m_vDoFs[0]->SetValue(vx - (m_vn * m_vn.transpose()) * (vx - m_vpoints));
}

void ConstraintSet_NodePlaneColl::ComputeAndStore_Values() {
  const Vector3d& vx = this->m_vDoFs[0]->GetValue();

  Vector3d vv = vx - m_vpoints;
  this->m_vvalues[0] = -(m_vn.dot(vx - m_vpoints) - m_tol);
}

void ConstraintSet_NodePlaneColl::ComputeAndStore_Jacobian() {
  this->m_mJacobian.block(0, 0, 1, 3) = -m_vn.transpose();
}

void ConstraintSet_NodePlaneColl::ComputeAndStore_Hessian() {
  // Zero: nothing to do here
}
}  // namespace PhySim