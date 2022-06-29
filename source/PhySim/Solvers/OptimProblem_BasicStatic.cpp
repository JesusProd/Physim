//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Solvers/OptimProblem_BasicStatic.h>

#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Physics/Simulables/Simulable_Mesh.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

OptimProblem_BasicStatic::OptimProblem_BasicStatic(ISimulable* pM) {
  this->Init(pM);
}

OptimProblem_BasicStatic::~OptimProblem_BasicStatic() {
  // Nothing to do here...
}

void OptimProblem_BasicStatic::Init(ISimulable* pM) {
  this->m_pM = pM;

  m_pM->GetState(this->m_pS);
  this->m_N = m_pM->GetNumFullDOF();
  this->m_M = 0;

  this->m_name = "[Basic Static]";
}

void OptimProblem_BasicStatic::GetVariables(VectorXd& vx) const {
  m_pM->GetDOFVector(vx, Tag_Position_X);
}

void OptimProblem_BasicStatic::IncVariables(const VectorXd& vx) {
  VectorXd vx0;
  m_pM->GetDOFVector(vx0, Tag_Position_X);
  m_pM->SetDOFVector(vx0 + vx, Tag_Position_X);
}

void OptimProblem_BasicStatic::SetVariables(const VectorXd& vx) {
  m_pM->SetDOFVector(vx, Tag_Position_X);
}

bool OptimProblem_BasicStatic::GetEnergy(Real& e) {
  this->m_timerComputeObj.Start();

  e = 0;
  this->m_pM->AddEnergy(e);

  this->m_timerComputeObj.StopStoreLog();

  return true;
}

bool OptimProblem_BasicStatic::GetGradient(AVectorXd& vg) {
  this->m_timerComputeGrad.Start();

  m_pM->GetGradient(vg);
  m_pM->FixVectorDoFs(vg);

  this->m_timerComputeGrad.StopStoreLog();

  return true;
}

bool OptimProblem_BasicStatic::GetHessian(AMatrixSd& mH) {
  this->m_timerComputeHess.Start();

  m_pM->GetHessian(mH);
  m_pM->FixMatrixDoFs(mH);

  this->m_timerComputeHess.StopStoreLog();

  return true;
}

bool OptimProblem_BasicStatic::OnIterStart(const OptimState& state) {
  m_pM->StepBoundaryConditions();

  m_pM->GetState(m_pS);

  return true;
}

bool OptimProblem_BasicStatic::OnIterFinish(const OptimState& state) {
  m_pM->UpdateKinematics();

  return true;
}

bool OptimProblem_BasicStatic::OnSolveStart(const OptimState& state) {
  return true;
}

bool OptimProblem_BasicStatic::OnSolveFinish(const OptimState& state) {
  return true;
}

bool OptimProblem_BasicStatic::PrePerformStep(const OptimState& state) {
  m_pM->SetState(this->m_pS);

  return true;
}

bool OptimProblem_BasicStatic::PosPerformStep(const OptimState& state) {
  // m_pM->UpdateKinematics();

  return true;
}

bool OptimProblem_BasicStatic::IsFullyConstrained() {
  return this->m_pM->BoundaryConditionsLoaded();
}

}  // namespace PhySim
