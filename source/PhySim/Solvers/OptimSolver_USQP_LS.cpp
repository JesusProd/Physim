//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Solvers/OptimSolver_USQP_LS.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

const OptimState& OptimSolver_USQP_LS::SolveStepInternal() {
  if (!this->m_isInitialized) {
    this->m_state.m_result = OSResult::OR_FAILURE;
    return this->m_state;
  }

  this->m_pProblem->OnIterStart(this->m_state);

  VectorXd dxTry;
  VectorXd dxFin;
  Real objNew;
  int numBis;
  AVectorXd vgNew;

  this->m_pProblem->GetEnergy(this->m_state.m_obj);
  this->m_pProblem->GetGradient(this->m_state.m_vg);
  this->m_pProblem->GetVariables(this->m_state.m_vx);

  this->m_state.m_opt = this->m_state.m_vg.norm();

  IOUtils::logTrace(
      Verbosity::V1_Default,
      "\n--\nStarting problem %s STEP. Objective: %.9e. Optimality: %.9e",
      this->m_pProblem->GetName().c_str(), this->m_state.m_obj,
      this->m_state.m_opt);

  // Check if the gradient is near zero

  if (m_state.m_opt < this->m_options.tolMaxError) {
    this->m_state.m_result = OSResult::OR_SUCCESS;
    return this->m_state;
  }

  // Compute the step

  bool success = false;
  for (int i = 0; i < this->m_options.lsNumRegTrials && !success; ++i)
    success = this->ComputeStep(dxTry, i);  // Try regularizations

  // Line-search in dx

  bool improved = this->LineSearch(dxTry, dxFin, objNew, vgNew, numBis);

  // Update state result

  this->UpdateStepResult(dxFin, objNew, vgNew);

  this->m_pProblem->OnIterFinish(this->m_state);

  return this->m_state;
}

}  // namespace PhySim
