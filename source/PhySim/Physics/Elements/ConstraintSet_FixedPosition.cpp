//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/ConstraintSet_FixedPosition.h>

#include <PhySim/Kinematics/KinematicsEle.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

ConstraintSet_FixedPosition::ConstraintSet_FixedPosition(Simulable* pModel,
                                                         KinematicsEle* pDoF,
                                                         const VectorXd& vt)
    : ConstraintSet(pModel, true) {
  int N = pDoF->NumDim();

  assert((int)vt.size() == N);

  this->m_vDoFs.resize(1);
  this->m_vDoFs[0] = pDoF;

  this->m_vvalues.resize(N);
  this->m_mJacobian.resize(N, N);
  this->m_vHessian.resize(0);

  this->m_type = Type::EQ;

  this->m_vt = vt;
}

ConstraintSet_FixedPosition::~ConstraintSet_FixedPosition(void) {
  // Nothing to do here...
}

void ConstraintSet_FixedPosition::Init() {
  int N = this->m_vDoFs[0]->NumDim();
  this->m_mJacobian.setIdentity(N, N);
}

void ConstraintSet_FixedPosition::ProjectConstraint() {
  this->m_vDoFs[0]->SetPositionX(this->m_vt);
}

void ConstraintSet_FixedPosition::ComputeAndStore_Values() {
  this->m_vvalues = m_vDoFs[0]->GetPositionX() - m_vt;
}

void ConstraintSet_FixedPosition::ComputeAndStore_Jacobian() {
  // Constant: nothing to do here
}
}  // namespace PhySim