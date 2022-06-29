//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Solvers/OptimProblem.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

OptimProblem::OptimProblem() {
  this->m_timerComputeObj = CustomTimer(10, "CAL_OBJ");
  this->m_timerComputeGrad = CustomTimer(10, "CAL_GRA");
  this->m_timerComputeHess = CustomTimer(10, "CAL_HES");
}

void OptimProblem::GetUpperBound(VectorXd& vub) const {
  vub.setConstant(this->m_N, HUGE_VAL);
}

void OptimProblem::GetLowerBound(VectorXd& vlb) const {
  vlb.setConstant(this->m_N, -HUGE_VAL);
}

bool OptimProblem::GetEnergy(Real& e) {
  e = HUGE_VAL;

  return false;
}

bool OptimProblem::GetGradient(AVectorXd& vg) {
  vg.setConstant(m_N, 0);

  return false;
}

bool OptimProblem::GetHessian(AMatrixSd& mH) {
  mH.Zero();

  return false;
}

bool OptimProblem::GetConstraint(VectorXd& vc) {
  vc.setConstant(m_N, 0);

  return false;
}

bool OptimProblem::GetJacobian(MatrixSd& mJ) {
  mJ = MatrixSd(m_M, m_N);

  return false;
}

}  // namespace PhySim
