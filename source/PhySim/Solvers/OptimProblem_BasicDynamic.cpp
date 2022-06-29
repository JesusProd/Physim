//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Solvers/OptimProblem_BasicDynamic.h>

#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

OptimProblem_BasicDynamic::OptimProblem_BasicDynamic(ISimulable* pM, Real dt) {
  this->Init(pM, dt);
}

OptimProblem_BasicDynamic::~OptimProblem_BasicDynamic() {
  // Nothing to do here..
}

void OptimProblem_BasicDynamic::Init(ISimulable* pM, Real dt) {
  this->m_dt = dt;
  this->m_pM = pM;

  m_pM->GetState(m_pS);
  this->m_N = m_pM->GetNumFullDOF();
  this->m_M = 0;

  m_pM->GetDOFVector(this->m_vx0, Tag::Tag_Position_X);
  m_pM->GetDOFVector(this->m_vv0, Tag::Tag_Velocity);

  this->m_name = "[Basic Dynamic]";

  OptimProblem::Init();
}

void OptimProblem_BasicDynamic::GetVariables(VectorXd& vx) const {
  m_pM->GetDOFVector(vx, Tag::Tag_Position_X);
}

void OptimProblem_BasicDynamic::IncVariables(const VectorXd& vx) {
  VectorXd vx0;
  m_pM->GetDOFVector(vx0, Tag::Tag_Position_X);
  m_pM->SetDOFVector(vx0 + vx, Tag::Tag_Position_X);
}

void OptimProblem_BasicDynamic::SetVariables(const VectorXd& vx) {
  m_pM->SetDOFVector(vx, Tag::Tag_Position_X);
}

bool OptimProblem_BasicDynamic::GetEnergy(Real& e) {
  this->m_timerComputeObj.Start();

  VectorXd vx;
  VectorXd vv;
  this->m_pM->GetDOFVector(vx, Tag::Tag_Position_X);
  this->m_pM->GetDOFVector(vv, Tag::Tag_Velocity);

  m_pM->GetEnergy(e);
  Real dt2 = m_dt * m_dt;
  m_pM->GetInertia(m_mMass);

  VectorXd va = (vx - m_vx0) / dt2 - m_vv0 / m_dt;

  e += (dt2 / 2.0) * va.dot(m_mMass * va);

  this->m_timerComputeObj.StopStoreLog();

  return true;
}

bool OptimProblem_BasicDynamic::GetGradient(AVectorXd& vg) {
  this->m_timerComputeGrad.Start();

  VectorXd vx;
  VectorXd vv;
  this->m_pM->GetDOFVector(vx, Tag::Tag_Position_X);
  this->m_pM->GetDOFVector(vv, Tag::Tag_Velocity);

  m_pM->GetGradient(vg);
  m_pM->GetInertia(m_mMass);
  m_pM->GetDMvDt(m_vDMvDt);
  Real dt2 = m_dt * m_dt;

  VectorXd va = (vx - m_vx0) / dt2 - m_vv0 / m_dt;

  vg += m_mMass * va + m_vDMvDt;

  m_pM->FixVectorDoFs(vg);

  this->m_timerComputeGrad.StopStoreLog();

  return true;
}

bool OptimProblem_BasicDynamic::GetHessian(AMatrixSd& mH) {
  this->m_timerComputeHess.Start();

  m_pM->GetHessian(m_mHess);
  m_pM->GetInertia(m_mMass);
  mH = m_mHess;
  mH += m_mMass * (1.0 / (m_dt * m_dt));
  m_pM->FixMatrixDoFs(mH);

  this->m_timerComputeHess.StopStoreLog();

  return true;
}

bool OptimProblem_BasicDynamic::IsFullyConstrained() {
  return true;
}

bool OptimProblem_BasicDynamic::OnSolveStart(const OptimState& state) {
  IOUtils::logTrace(Verbosity::V1_Default,
                    "\n[TRACE] Starting step solve with dt = %f", this->m_dt);

  m_pM->GetDOFVector(this->m_vx0, Tag::Tag_Position_X);
  m_pM->GetDOFVector(this->m_vv0, Tag::Tag_Velocity);

  m_pM->StepBoundaryConditions();

  m_pM->SetDOFVector(this->m_vx0 + this->m_dt * this->m_vv0, Tag::Tag_Position_X);

  m_pM->UpdateKinematics();

  return true;
}

bool OptimProblem_BasicDynamic::OnSolveFinish(const OptimState& state) {
  return true;
}

bool OptimProblem_BasicDynamic::OnIterStart(const OptimState& state) {
  m_pM->GetState(m_pS);

  return true;
}

bool OptimProblem_BasicDynamic::OnIterFinish(const OptimState& state) {
  return true;
}

bool OptimProblem_BasicDynamic::PrePerformStep(const OptimState& state) {
  m_pM->SetState(m_pS);

  return true;
}

bool OptimProblem_BasicDynamic::PosPerformStep(const OptimState& state) {
  VectorXd vv;

  m_pM->GetDOFVector(vv, Tag::Tag_Position_X);
  vv -= this->m_vx0;
  vv /= this->m_dt;
  m_pM->SetDOFVector(vv, Tag::Tag_Velocity);

  m_pM->UpdateKinematics();

  return true;
}

}  // namespace PhySim
