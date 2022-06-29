//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Kinematics/KinematicsEle.h>

#include <PhySim/Physics/Elements/ConstraintSet.h>
#include <PhySim/Physics/Elements/EnergyElement.h>
#include <PhySim/Physics/Elements/MassElement_Lumped.h>

#include <PhySim/Utils/IOUtils.h>
#include <PhySim/Utils/ParameterSet.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

int Simulable::IDCount = 0;

Simulable::Simulable() {
  this->m_isProfiling = true;
  this->m_timerComputeKine = CustomTimer(10, "COM_KIN");
  this->m_timerComputeMech = CustomTimer(10, "COM_MEC");

  this->m_timerComputeEner = CustomTimer(10, "CAL_ENE");
  this->m_timerComputeGrad = CustomTimer(10, "CAL_GRA");
  this->m_timerComputeHess = CustomTimer(10, "CAL_HES");

  this->m_timerPropagaGrad = CustomTimer(10, "PRO_GRA");
  this->m_timerPropagaHess = CustomTimer(10, "PRO_HES");

  this->m_timerAssembleEner = CustomTimer(10, "ASS_ENE");
  this->m_timerAssembleGrad = CustomTimer(10, "ASS_GRA");
  this->m_timerAssembleHess = CustomTimer(10, "ASS_HES");

  FreeInternal();

  m_ID = Simulable::IDCount;
  Simulable::IDCount += 1;
}

Simulable::~Simulable() {
  FreeInternal();

#ifndef NDEBUG
  IOUtils::logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Simulable");
#endif
}

void Simulable::FreeInternal() {
  this->m_numFreeDoF = 0;
  this->m_numFixedDoF = 0;

  this->m_dirtyFlags = DirtyFlags::All;

  m_venerEle.clear();
  m_vmassEle.clear();
  m_vconsSet.clear();
  m_vBC.clear();
}

void Simulable::InitInternal() {
  // Nothing to do here...
}

void Simulable::Setup() {
  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s setup",
                    this->GetName().c_str());

  this->FreeInternal();
  this->InitInternal();

  this->ComputeAndStore_Rest();
  this->ComputeAndStore_Mass();
  this->UpdateMechanics();

  this->ComputeAndStore_Energy();
  this->ComputeAndStore_Gradient();
  this->ComputeAndStore_Hessian();
  this->ComputeAndStore_Fixed();

  IOUtils::logTrace(Verbosity::V1_Default, "\n--");
  IOUtils::logTrace(Verbosity::V1_Default, "\nINITIALIZED %s",
                    this->GetName().c_str());
  IOUtils::logTrace(Verbosity::V1_Default, "\nTotal full DOF: %i",
                    this->GetNumFullDOF());
  IOUtils::logTrace(Verbosity::V1_Default, "\nTotal free DOF: %i",
                    this->m_numFreeDoF);
  IOUtils::logTrace(Verbosity::V1_Default, "\nTotal fixed DOF: %i",
                    this->m_numFixedDoF);
  IOUtils::logTrace(Verbosity::V1_Default, "\nBoundary conditions: %i",
                    (int)this->m_vBC.size());
  IOUtils::logTrace(Verbosity::V1_Default, "\n--");

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s setup",
                    this->GetName().c_str());
}

inline const PtrS<KinematicsTree>& Simulable::GetKinematicsTree() const {
  return this->m_pTree;
}

inline const vector<PtrS<KinematicsEle>>& Simulable::GetKinematics() const {
  return this->m_pTree->RootElements();
}

void Simulable::GetState(VectorXd& vs) const {
  this->m_pTree->GetFullState(vs);
}

void Simulable::SetState(const VectorXd& vs) {
  this->m_pTree->SetFullState(vs);

  this->DirtyKinematics();
}

inline int Simulable::GetNumFullDOF() const {
  return this->m_pTree->GetNumActiveDoF();
}

inline int Simulable::GetNumFreeDOF() const {
  return this->m_numFreeDoF;
}

inline int Simulable::GetNumFixedDOF() const {
  return this->m_numFixedDoF;
}

void Simulable::GetDOFVector(VectorXd& vx, Tag s) const {
  vx.resize(this->GetNumFullDOF());

  if (s == Tag::Position_X) {
    this->m_pTree->GetPositionX(vx);
    return;
  }

  if (s == Tag::Position_0) {
    this->m_pTree->GetPosition0(vx);
    return;
  }

  if (s == Tag::Velocity) {
    this->m_pTree->GetVelocity(vx);
    return;
  }
}

void Simulable::SetDOFVector(const VectorXd& vx, Tag s) {
  assert(vx.size() == this->GetNumFullDOF());

  if (s == Tag::Position_X) {
    this->m_pTree->SetPositionX(vx);
    this->DirtyKinematics();
    return;
  }

  if (s == Tag::Velocity) {
    this->m_pTree->SetVelocity(vx);
    this->DirtyKinematics();
    return;
  }

  if (s == Tag::Position_0) {
    this->m_pTree->SetPosition0(vx);
    this->DirtyRest();
    return;
  }
}

void Simulable::UpdateKinematics() {
  if (!IsDirty_Kinematics())
    return;  // Updated

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s kinematics update",
                    this->GetName().c_str());

  if (m_isProfiling)
    this->m_timerComputeKine.Start();

  this->m_pTree->UpdateKinematics();

  for (PtrS<IEnergyElement> pEle : m_venerEle)
    pEle->UpdateKinematics();

  for (PtrS<IMassElement> pEle : m_vmassEle)
    pEle->UpdateKinematics();

  for (PtrS<IConstraintSet> pEle : m_vconsSet)
    pEle->UpdateKinematics();

  if (m_isProfiling)
    this->m_timerComputeKine.StopStoreLog();

  this->m_dirtyFlags =
      (DirtyFlags)(this->m_dirtyFlags & ~DirtyFlags::Kinematics);

  this->DirtyMechanics();

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s kinematics update",
                    this->GetName().c_str());
}

void Simulable::UpdateMechanics() {
  if (!IsDirty_Mechanics())
    return;  // Updated

  if (m_isProfiling)
    this->m_timerComputeMech.Start();

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s mechanics precomputation",
                    this->GetName().c_str());

  // this->m_pTree->UpdateMapPartials();

  for (PtrS<IEnergyElement> pEle : m_venerEle)
    pEle->UpdateMechanics();

  for (PtrS<IMassElement> pEle : m_vmassEle)
    pEle->UpdateMechanics();

  for (PtrS<IConstraintSet> pEle : m_vconsSet)
    pEle->UpdateMechanics();

  if (m_isProfiling)
    this->m_timerComputeMech.StopStoreLog();

  this->m_dirtyFlags =
      (DirtyFlags)(this->m_dirtyFlags & ~DirtyFlags::Mechanics);

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s mechanics precomputation",
                    this->GetName().c_str());
}

void Simulable::SetUseFDGradientAtElements(bool u) {
  for (auto& pEle : this->m_venerEle)
    pEle->SetUseFDGradient(u);
}

void Simulable::SetUseFDHessianAtElements(bool u) {
  for (auto& pEle : this->m_venerEle)
    pEle->SetUseFDHessian(u);
}

inline const VectorXd& Simulable::GetConstraint() {
  if (this->IsDirty_Constraint())
    this->ComputeAndStore_Constraint();

  throw new PhySim::exception("Not implemented: deprecated");
}

inline const MatrixSd& Simulable::GetJacobian() {
  if (this->IsDirty_Jacobian())
    this->ComputeAndStore_Jacobian();

  throw new PhySim::exception("Not implemented: deprecated");
}

void Simulable::GetEnergy(Real& energy, bool addBC) {
  energy = 0;

  this->AddEnergy(energy, addBC);
}

void Simulable::GetGradient(AVectorXd& vgradient, bool addBC) {
  vgradient.Layer() = m_ID;
  int N = this->GetNumFullDOF();
  vgradient.setZero(N);

  this->AddGradient(vgradient, addBC);
}

void Simulable::GetHessian(AMatrixSd& mHessian, bool addBC) {
  int N = this->GetNumFullDOF();

  if (mHessian.IsInit()) {
    assert(mHessian.rows() == N);
    assert(mHessian.cols() == N);
  } else {
    mHessian = AMatrixSd(N, N, -1, true);
  }

  mHessian.Layer() = m_ID;

  mHessian.StartAssembly();
  this->AddHessian(mHessian, addBC);
  mHessian.EndAssembly();
}

void Simulable::GetInertia(AMatrixSd& mInertia) {
  int N = this->GetNumFullDOF();
  mInertia = AMatrixSd(N, N);

  mInertia.Layer() = m_ID;

  mInertia.StartAssembly();
  this->AddInertia(mInertia);
  mInertia.EndAssembly();
}

void Simulable::GetDMvDt(AVectorXd& vDMvDt) {
  vDMvDt.Layer() = m_ID;
  int N = this->GetNumFullDOF();
  vDMvDt.setZero(N);

  this->AddDMvDt(vDMvDt);
}

void Simulable::AddEnergy(Real& energy, bool addBC) {
  if (this->IsDirty_Energy())
    this->ComputeAndStore_Energy();

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s energy assembly",
                    this->GetName().c_str());

  if (m_isProfiling)
    this->m_timerAssembleEner.Start();

  if (this->m_vselEles.empty()) {
    for (size_t i = 0; i < this->m_venerEle.size(); ++i)
      energy += this->m_venerEle[i]->GetLocalEnergy();
  } else {
    for (size_t i = 0; i < this->m_vselEles.size(); ++i)
      energy += this->m_venerEle[m_vselEles[i]]->GetLocalEnergy();
  }

  for (size_t i = 0; i < this->m_vmassEle.size(); ++i)
    energy += this->m_vmassEle[i]->GetLocalEnergy();

  if (addBC) {
    for (size_t i = 0; i < this->m_vBC.size(); ++i)
      for (size_t j = 0; j < this->m_vBC[i]->Energies().size(); ++j)
        energy += this->m_vBC[i]->Energies()[j]->GetLocalEnergy();
  }

  if (m_isProfiling)
    this->m_timerAssembleEner.StopStoreLog();

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s energy assembly",
                    this->GetName().c_str());
}

void Simulable::AddGradient(AVectorXd& vgradient, bool addBC) {
  if (this->IsDirty_Gradient())
    this->ComputeAndStore_Gradient();

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s gradient assembly",
                    this->GetName().c_str());

  // Internal

  if (this->m_vselEles.empty()) {
    // Propagation

    if (m_isProfiling)
      this->m_timerPropagaGrad.Start();

#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)this->m_venerEle.size(); ++i)
      this->m_venerEle[i]->Propagate_Gradient(vgradient);

    //#ifdef NDEBUG
    //#pragma omp parallel for
    //#endif
    // for (int i = 0; i < (int) this->m_venerEle.size(); ++i)
    //	this->m_venerEle[i]->PreprocessAssembly(vgradient.Layer());

    if (m_isProfiling)
      this->m_timerPropagaGrad.StopStoreLog();

    // Assembly

    if (m_isProfiling)
      this->m_timerAssembleGrad.Start();

    // for (size_t i = 0; i < this->m_vselEles.size(); ++i)
    //	this->m_venerEle[m_vselEles[i]]->AssemblePreprocessedGradient(vgradient);

    for (size_t i = 0; i < this->m_venerEle.size(); ++i)
      this->m_venerEle[i]->Assemble_Gradient(vgradient);

    // for (size_t i = 0; i < this->m_venerEle.size(); ++i)
    //	this->m_venerEle[i]->PropagateAndAssemble_Gradient(vgradient);

    if (m_isProfiling)
      this->m_timerAssembleGrad.StopStoreLog();
  } else {
    // Propagation

    if (m_isProfiling)
      this->m_timerPropagaGrad.Start();

      //#ifdef NDEBUG
      //#pragma omp parallel for
      //#endif
      //			for (int i = 0; i <
      //(int)this->m_venerEle.size(); ++i)
      //				this->m_venerEle[i]->PreprocessAssembly(vgradient.Layer());

#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)this->m_vselEles.size(); ++i)
      this->m_venerEle[m_vselEles[i]]->Propagate_Gradient(vgradient);

    if (m_isProfiling)
      this->m_timerPropagaGrad.StopStoreLog();

    // Assembly

    if (m_isProfiling)
      this->m_timerAssembleGrad.Start();

    // for (size_t i = 0; i < this->m_vselEles.size(); ++i)
    //	this->m_venerEle[m_vselEles[i]]->AssemblePreprocessedGradient(vgradient);

    for (size_t i = 0; i < this->m_vselEles.size(); ++i)
      this->m_venerEle[m_vselEles[i]]->Assemble_Gradient(vgradient);

    // for (size_t i = 0; i < this->m_vselEles.size(); ++i)
    //	this->m_venerEle[m_vselEles[i]]->PropagateAndAssemble_Gradient(vgradient);

    if (m_isProfiling)
      this->m_timerAssembleGrad.StopStoreLog();
  }

  // Gravity

  // if (m_isProfiling) this->m_timerPropagaGrad.Resume();

  //#ifdef NDEBUG
  //#pragma omp parallel for
  //#endif
  // for (int i = 0; i < (int)this->m_vmassEle.size(); ++i)
  //	this->m_vmassEle[i]->PreprocessAssembly(vgradient.Layer());

#ifdef NDEBUG
#pragma omp parallel for
#endif
  for (int i = 0; i < (int)this->m_vmassEle.size(); ++i)
    this->m_vmassEle[i]->Propagate_Gradient(vgradient);

  // if (m_isProfiling) this->m_timerPropagaGrad.Pause();

  // if (m_isProfiling) this->m_timerAssembleGrad.Resume();

  // for (size_t i = 0; i < this->m_vmassEle.size(); ++i)
  //	this->m_vmassEle[i]->AssemblePreprocessedGradient(vgradient);

  for (size_t i = 0; i < this->m_vmassEle.size(); ++i)
    this->m_vmassEle[i]->Assemble_Gradient(vgradient);

  // for (size_t i = 0; i < this->m_vmassEle.size(); ++i)
  //	this->m_vmassEle[i]->PropagateAndAssemble_Gradient(vgradient);

  // if (m_isProfiling) this->m_timerAssembleGrad.Pause();

  // BC

  if (addBC) {
    // if (m_isProfiling) this->m_timerPropagaGrad.Resume();

    //#ifdef NDEBUG
    //#pragma omp parallel for
    //#endif
    //			for (int i = 0; i < (int)m_vBC.size(); ++i)
    //				for (int j = 0; j <
    //(int)m_vBC[i]->Energies().size();
    //++j)
    // m_vBC[i]->Energies()[j]->PreprocessAssembly(vgradient.Layer());

#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)m_vBC.size(); ++i)
      for (int j = 0; j < (int)m_vBC[i]->Energies().size(); ++j)
        m_vBC[i]->Energies()[j]->Propagate_Gradient(vgradient);

    // if (m_isProfiling) this->m_timerPropagaGrad.StopStoreLog();

    // if (m_isProfiling) this->m_timerAssembleGrad.Resume();

    // for (size_t i = 0; i < m_vBC.size(); ++i)
    //	for (size_t j = 0; j < m_vBC[i]->Energies().size(); ++j)
    //		m_vBC[i]->Energies()[j]->AssemblePreprocessedGradient(vgradient);

    for (size_t i = 0; i < m_vBC.size(); ++i)
      for (size_t j = 0; j < m_vBC[i]->Energies().size(); ++j)
        m_vBC[i]->Energies()[j]->Assemble_Gradient(vgradient);

    // for (size_t i = 0; i < m_vBC.size(); ++i)
    //	for (size_t j = 0; j < m_vBC[i]->Energies().size(); ++j)
    //		m_vBC[i]->Energies()[j]->PropagateAndAssemble_Gradient(vgradient);

    // if (m_isProfiling) this->m_timerAssembleGrad.StopStoreLog();
  } else {
    // if (m_isProfiling) this->m_timerPropagaGrad.StopStoreLog();
    // if (m_isProfiling) this->m_timerAssembleGrad.StopStoreLog();
  }

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s gradient assembly",
                    this->GetName().c_str());
}

void Simulable::AddHessian(AMatrixSd& mHessian, bool addBC) {
  if (this->IsDirty_Hessian())
    this->ComputeAndStore_Hessian();

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s Hessian assembly",
                    this->GetName().c_str());

  if (!mHessian.IsFast()) {
    IOUtils::logTrace(Verbosity::V4_DeepShit,
                      "\n[TRACE] Fast assembly Hessian: OFF");

    int numTriplets = 0;

    for (size_t i = 0; i < this->m_venerEle.size(); ++i)
      numTriplets += m_venerEle[i]->GetHessianSize();

    for (size_t i = 0; i < this->m_vmassEle.size(); ++i)
      numTriplets += m_vmassEle[i]->GetHessianSize();

    mHessian.SetNumNZStatic(mHessian.GetNumNZStatic() + numTriplets);

    IOUtils::logTrace(Verbosity::V4_DeepShit, "\n[TRACE] Allocated %d triplets",
                      mHessian.GetNumNZStatic());
  } else {
    IOUtils::logTrace(Verbosity::V4_DeepShit,
                      "\n[TRACE] Fast assembly Hessian: ON");
  }

  // Internal

  if (this->m_vselEles.empty()) {
    if (m_isProfiling)
      this->m_timerPropagaHess.Start();

      //#ifdef NDEBUG
      //#pragma omp parallel for
      //#endif
      //			for (int i = 0; i <
      //(int)this->m_venerEle.size(); ++i)
      //				this->m_venerEle[i]->PreprocessAssembly(mHessian.Layer());

#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)this->m_venerEle.size(); ++i)
      this->m_venerEle[i]->Propagate_Hessian(mHessian);

    if (m_isProfiling)
      this->m_timerPropagaHess.StopStoreLog();

    if (m_isProfiling)
      this->m_timerAssembleHess.Start();

    // for (int i = 0; i < (int)this->m_venerEle.size(); ++i)
    //	this->m_venerEle[i]->AssemblePreprocessedHessian(mHessian);

    for (size_t i = 0; i < this->m_venerEle.size(); ++i)
      this->m_venerEle[i]->Assemble_Hessian(mHessian);

    // for (size_t i = 0; i < this->m_venerEle.size(); ++i)
    //	this->m_venerEle[i]->PropagateAndAssemble_Hessian(mHessian);

    if (m_isProfiling)
      this->m_timerAssembleHess.StopStoreLog();
  } else {
    if (m_isProfiling)
      this->m_timerPropagaHess.Start();

      //#ifdef NDEBUG
      //#pragma omp parallel for
      //#endif
      //			for (int i = 0; i <
      //(int)this->m_venerEle.size(); ++i)
      //				this->m_venerEle[i]->PreprocessAssembly(mHessian.Layer());

#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)this->m_venerEle.size(); ++i)
      this->m_venerEle[m_vselEles[i]]->Propagate_Hessian(mHessian);

    if (m_isProfiling)
      this->m_timerPropagaHess.StopStoreLog();

    if (m_isProfiling)
      this->m_timerAssembleHess.Start();

    // for (int i = 0; i < (int)this->m_venerEle.size(); ++i)
    //	this->m_venerEle[i]->AssemblePreprocessedHessian(mHessian);

    for (size_t i = 0; i < this->m_venerEle.size(); ++i)
      this->m_venerEle[m_vselEles[i]]->Assemble_Hessian(mHessian);

    // for (size_t i = 0; i < this->m_venerEle.size(); ++i)
    //	this->m_venerEle[m_vselEles[i]]->PropagateAndAssemble_Hessian(mHessian);

    if (m_isProfiling)
      this->m_timerAssembleHess.StopStoreLog();
  }

  // Gravity

  // if (m_isProfiling) this->m_timerPropagaHess.Resume();

  //#ifdef NDEBUG
  //#pragma omp parallel for
  //#endif
  //		for (int i = 0; i < (int)this->m_vmassEle.size(); ++i)
  //			this->m_vmassEle[i]->PreprocessAssembly(mHessian.Layer());

#ifdef NDEBUG
#pragma omp parallel for
#endif
  for (int i = 0; i < (int)this->m_vmassEle.size(); ++i)
    this->m_vmassEle[i]->Propagate_Hessian(mHessian);

  // if (m_isProfiling) this->m_timerPropagaHess.Pause();

  // if (m_isProfiling) this->m_timerAssembleHess.Resume();

  // for (size_t i = 0; i < this->m_vmassEle.size(); ++i)
  //	this->m_vmassEle[i]->AssemblePreprocessedHessian(mHessian);

  for (size_t i = 0; i < this->m_vmassEle.size(); ++i)
    this->m_vmassEle[i]->Assemble_Hessian(mHessian);

  // for (size_t i = 0; i < this->m_vmassEle.size(); ++i)
  //	this->m_vmassEle[i]->PropagateAndAssemble_Hessian(mHessian);

  // if (m_isProfiling) this->m_timerAssembleHess.Pause();

  // BC

  if (addBC) {
    // if (m_isProfiling) this->m_timerPropagaHess.Resume();

    //#ifdef NDEBUG
    //#pragma omp parallel for
    //#endif
    //			for (int i = 0; i < (int)m_vBC.size(); ++i)
    //				for (int j = 0; j <
    //(int)m_vBC[i]->Energies().size();
    //++j)
    // m_vBC[i]->Energies()[j]->PreprocessAssembly(mHessian.Layer());

#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)m_vBC.size(); ++i)
      for (int j = 0; j < (int)m_vBC[i]->Energies().size(); ++j)
        m_vBC[i]->Energies()[j]->Propagate_Hessian(mHessian);

    // if (m_isProfiling) this->m_timerPropagaHess.StopStoreLog();

    // if (m_isProfiling) this->m_timerAssembleHess.Resume();

    // for (size_t i = 0; i < m_vBC.size(); ++i)
    //	for (size_t j = 0; j < m_vBC[i]->Energies().size(); ++j)
    //		m_vBC[i]->Energies()[j]->AssemblePreprocessedHessian(mHessian);

    for (size_t i = 0; i < m_vBC.size(); ++i)
      for (size_t j = 0; j < m_vBC[i]->Energies().size(); ++j)
        m_vBC[i]->Energies()[j]->Assemble_Hessian(mHessian);

    // for (size_t i = 0; i < m_vBC.size(); ++i)
    //	for (size_t j = 0; j < m_vBC[i]->Energies().size(); ++j)
    //		m_vBC[i]->Energies()[j]->PropagateAndAssemble_Hessian(mHessian);

    // if (m_isProfiling) this->m_timerAssembleHess.StopStoreLog();
  } else {
    // if (m_isProfiling) this->m_timerPropagaHess.StopStoreLog();
    // if (m_isProfiling) this->m_timerAssembleHess.StopStoreLog();
  }

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s Hessian assembly",
                    this->GetName().c_str());
}

void Simulable::AddInertia(AMatrixSd& mInertia) {
  if (this->IsDirty_Mass())
    this->ComputeAndStore_Mass();

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s inertia assembly",
                    this->GetName().c_str());

  for (size_t i = 0; i < this->m_vmassEle.size(); ++i)
    this->m_vmassEle[i]->AssembleGlobal_Mass(mInertia);

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s inertia assembly",
                    this->GetName().c_str());
}

void Simulable::AddDMvDt(AVectorXd& vDMvDt) {
  if (this->IsDirty_Mass())
    this->ComputeAndStore_Mass();

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s DMvDt assembly",
                    this->GetName().c_str());

  for (size_t i = 0; i < this->m_vmassEle.size(); ++i)
    this->m_vmassEle[i]->AssembleGlobal_DMDtv(vDMvDt);

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s DMvDt assembly",
                    this->GetName().c_str());
}

void Simulable::FixVectorDoFs(VectorXd& vv) {
  if (this->IsDirty_Fixed())
    this->ComputeAndStore_Fixed();

  IOUtils::logTrace(Verbosity::V4_DeepShit, "\n[TRACE] Fixing DoFs in vector");

  for (int i = 0; i < this->m_vFixIndices.size(); ++i) {
    vv[this->m_vFixIndices[i]] = 0;
  }
}

void Simulable::FixMatrixDoFs(MatrixSd& mM) {
  if (this->IsDirty_Fixed())
    this->ComputeAndStore_Fixed();

  IOUtils::logTrace(Verbosity::V4_DeepShit, "\n[TRACE] Fixing DoFs in matrix");

  for (int k = 0; k < mM.outerSize(); ++k) {
    for (MatrixSd::InnerIterator it(mM, k); it; ++it)
      if (m_vFixStencil[it.row()] || m_vFixStencil[it.col()])
        if (it.row() == it.col())
          it.valueRef() = 1.0;
        else
          it.valueRef() = 0.0;
  }
}

const MatrixSd& Simulable::GetFreeSelection() {
  if (this->IsDirty_Fixed())
    this->ComputeAndStore_Fixed();

  return this->m_mFreeSele;
}

const MatrixSd& Simulable::GetFreePermutation() {
  if (this->IsDirty_Fixed())
    this->ComputeAndStore_Fixed();

  return this->m_mFreePerm;
}

void Simulable::ComputeAndStore_Fixed() {
  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s fixed precomputation",
                    this->GetName().c_str());

  // Propagate fixation from the leaves to the roots
  // If a pasive node is fixed, all parents should be

  this->m_pTree->PropagateFixation();

  const vector<PtrS<KinematicsEle>>& vDoFs = this->m_pTree->RootElements();

  // Allocate free DOF

  int numFullDoF = this->GetNumFullDOF();

  size_t numDoFSet = vDoFs.size();
  iVector vfreeOffset(numFullDoF, -1);
  m_vFixStencil.resize(numFullDoF, false);
  m_vFixIndices.reserve(numFullDoF);

  int identCount = 0;
  m_numFixedDoF = 0;
  m_numFreeDoF = 0;

  for (size_t i = 0; i < numDoFSet; ++i) {
    if (!vDoFs[i]->Active())
      continue;  // Ignore

    const bVector& vfix = vDoFs[i]->GetFixed();
    for (int j = 0; j < vDoFs[i]->NumDim(); ++j) {
      int idx = vDoFs[i]->Offset() + j;
      if (vfix[j]) {
        m_vFixStencil[idx] = true;
        m_vFixIndices.push_back(idx);
        vfreeOffset[idx] = -1;
        m_numFixedDoF++;
      } else {
        m_vFixStencil[idx] = false;
        vfreeOffset[idx] = m_numFreeDoF;
        m_numFreeDoF++;
      }
    }
  }

  assert(numFullDoF == m_numFixedDoF + m_numFreeDoF);

  // Build free DoF selection

  VectorTd vS;
  vS.reserve(m_numFreeDoF);
  for (size_t i = 0; i < numFullDoF; ++i) {
    if (!m_vFixStencil[i]) {
      assert(vfreeOffset[i] >= 0 && vfreeOffset[i] < m_numFreeDoF &&
             "Free DoF not in range");
      vS.push_back(Triplet<Real>(vfreeOffset[i], i, 1.0));
    }
  }
  m_mFreeSele = MatrixSd(m_numFreeDoF, numFullDoF);
  m_mFreeSele.setFromTriplets(vS.begin(), vS.end());
  m_mFreeSele.makeCompressed();

  // Build free/fixed permutation

  int offsetUnfix = 0;
  int offsetFixed = this->m_numFreeDoF;

  VectorTd vP;
  vP.reserve(numFullDoF);
  for (size_t i = 0; i < numFullDoF; ++i) {
    if (m_vFixStencil[i]) {
      assert(offsetFixed >= 0 && offsetFixed < numFullDoF &&
             "Fixed DoF not in range");
      vP.push_back(Triplet<Real>(offsetFixed, i, 1.0));
      offsetFixed++;
    } else {
      assert(offsetUnfix >= 0 && offsetUnfix < numFullDoF &&
             "Free DoF not in range");
      vP.push_back(Triplet<Real>(offsetUnfix, i, 1.0));
      offsetUnfix++;
    }
  }
  m_mFreePerm = MatrixSd(numFullDoF, numFullDoF);
  m_mFreePerm.setFromTriplets(vP.begin(), vP.end());
  m_mFreePerm.makeCompressed();

  this->CleanFlags_Fixed();

  IOUtils::logTrace(Verbosity::V1_Default,
                    "\n[TRACE] Precomputed fixation (Fixed: %d / Free: %d)",
                    m_numFixedDoF, m_numFreeDoF);

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s fixed precomputation",
                    this->GetName().c_str());
}

void Simulable::CleanFlags_Fixed() {
  this->m_dirtyFlags = (DirtyFlags)(this->m_dirtyFlags & ~DirtyFlags::Fixed);
}

void Simulable::DirtyMass() {
  this->m_dirtyFlags = this->m_dirtyFlags | DirtyFlags::Mass;

  this->DirtyMechanics();
}

void Simulable::DirtyRest() {
  this->m_dirtyFlags = this->m_dirtyFlags | DirtyFlags::Rest;
  this->m_dirtyFlags = this->m_dirtyFlags | DirtyFlags::Mass;

  this->DirtyKinematics();
}

void Simulable::DirtyKinematics() {
  this->m_dirtyFlags = this->m_dirtyFlags | DirtyFlags::Kinematics;

  this->DirtyMechanics();
}

void Simulable::DirtyMechanics() {
  this->m_dirtyFlags = this->m_dirtyFlags | DirtyFlags::Mechanics;
  this->m_dirtyFlags = this->m_dirtyFlags | DirtyFlags::Energy;
  this->m_dirtyFlags = this->m_dirtyFlags | DirtyFlags::Gradient;
  this->m_dirtyFlags = this->m_dirtyFlags | DirtyFlags::Hessian;
  this->m_dirtyFlags = this->m_dirtyFlags | DirtyFlags::Constraint;
  this->m_dirtyFlags = this->m_dirtyFlags | DirtyFlags::Jacobian;
}

void Simulable::DirtyFixed() {
  this->m_dirtyFlags = this->m_dirtyFlags | DirtyFlags::Fixed;
}

bool Simulable::IsDirty_Energy() const {
  return (this->m_dirtyFlags & DirtyFlags::Energy) != DirtyFlags::None;
}

bool Simulable::IsDirty_Gradient() const {
  return (this->m_dirtyFlags & DirtyFlags::Gradient) != DirtyFlags::None;
}

bool Simulable::IsDirty_Hessian() const {
  return (this->m_dirtyFlags & DirtyFlags::Hessian) != DirtyFlags::None;
}

bool Simulable::IsDirty_Mass() const {
  return (this->m_dirtyFlags & DirtyFlags::Mass) != DirtyFlags::None;
}

bool Simulable::IsDirty_Rest() const {
  return (this->m_dirtyFlags & DirtyFlags::Rest) != DirtyFlags::None;
}

bool Simulable::IsDirty_Constraint() const {
  return (this->m_dirtyFlags & DirtyFlags::Constraint) != DirtyFlags::None;
}

bool Simulable::IsDirty_Jacobian() const {
  return (this->m_dirtyFlags & DirtyFlags::Jacobian) != DirtyFlags::None;
}

bool Simulable::IsDirty_Fixed() const {
  return (this->m_dirtyFlags & DirtyFlags::Fixed) != DirtyFlags::None;
}

bool Simulable::IsDirty_Kinematics() const {
  return (this->m_dirtyFlags & DirtyFlags::Kinematics) != DirtyFlags::None;
}

bool Simulable::IsDirty_Mechanics() const {
  return (this->m_dirtyFlags & DirtyFlags::Mechanics) != DirtyFlags::None;
}

void Simulable::ComputeAndStore_Rest() {
  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s rest computation",
                    this->GetName().c_str());

  // Initialize kinematics

  this->m_pTree->Init();

  // Initialize energy elements

  int numEle = (int)this->m_venerEle.size();

#ifdef NDEBUG
#pragma omp parallel for
#endif
  for (int i = 0; i < numEle; ++i)
    this->m_venerEle[i]->Init();

  // Intialize constraint elements

  int numCon = (int)this->m_vconsSet.size();

#ifdef NDEBUG
#pragma omp parallel for
#endif
  for (int i = 0; i < numCon; ++i)
    this->m_vconsSet[i]->Init();

  // Initialize mass elements

  int numMass = (int)this->m_vmassEle.size();

#ifdef NDEBUG
#pragma omp parallel for
#endif
  for (int i = 0; i < numMass; ++i)
    this->m_vmassEle[i]->Init();

  this->m_dirtyFlags = (DirtyFlags)(this->m_dirtyFlags & ~DirtyFlags::Rest);

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s rest computation",
                    this->GetName().c_str());
}

void Simulable::ComputeAndStore_Mass() {
  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s mass computation",
                    this->GetName().c_str());

  // Compute

#ifdef NDEBUG
#pragma omp parallel for
#endif
  for (int i = 0; i < (int)this->m_vmassEle.size(); ++i)
    this->m_vmassEle[i]->ComputeAndStore_Mass();

  this->m_dirtyFlags = (DirtyFlags)(this->m_dirtyFlags & ~DirtyFlags::Mass);

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s mass computation",
                    this->GetName().c_str());
}

void Simulable::ComputeAndStore_Energy(bool addBC) {
  if (!IsDirty_Energy())
    return;  // Updated

  if (IsDirty_Rest())
    this->ComputeAndStore_Rest();

  if (IsDirty_Mass())
    this->ComputeAndStore_Mass();

  if (IsDirty_Mechanics())
    this->UpdateMechanics();

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s energy computation",
                    this->GetName().c_str());

  // Compute

  if (m_isProfiling)
    this->m_timerComputeEner.Start();
  if (this->m_vselEles.empty()) {
#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)this->m_venerEle.size(); ++i)
      this->m_venerEle[i]->ComputeAndStore_Energy();
  } else {
#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)this->m_vselEles.size(); ++i)
      this->m_venerEle[m_vselEles[i]]->ComputeAndStore_Energy();
  }

#ifdef NDEBUG
#pragma omp parallel for
#endif
  for (int i = 0; i < (int)this->m_vmassEle.size(); ++i)
    this->m_vmassEle[i]->ComputeAndStore_Energy();

  if (addBC) {
#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)m_vBC.size(); ++i)
      for (int j = 0; j < (int)m_vBC[i]->Energies().size(); ++j)
        m_vBC[i]->Energies()[j]->ComputeAndStore_Energy();
  }
  if (m_isProfiling)
    this->m_timerComputeEner.StopStoreLog();

  // Clean

  this->m_dirtyFlags = (DirtyFlags)(this->m_dirtyFlags & ~DirtyFlags::Energy);

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s energy computation",
                    this->GetName().c_str());
}

void Simulable::ComputeAndStore_Gradient(bool addBC) {
  if (!IsDirty_Gradient())
    return;  // Updated

  if (IsDirty_Rest())
    this->ComputeAndStore_Rest();

  if (IsDirty_Mass())
    this->ComputeAndStore_Mass();

  if (IsDirty_Mechanics())
    this->UpdateMechanics();

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s gradient computation",
                    this->GetName().c_str());

  // Compute

  if (m_isProfiling)
    this->m_timerComputeGrad.Start();
  if (this->m_vselEles.empty()) {
#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)this->m_venerEle.size(); ++i)
      this->m_venerEle[i]->ComputeAndStore_Gradient();
  } else {
#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)this->m_vselEles.size(); ++i)
      this->m_venerEle[m_vselEles[i]]->ComputeAndStore_Gradient();
  }

#ifdef NDEBUG
#pragma omp parallel for
#endif
  for (int i = 0; i < (int)this->m_vmassEle.size(); ++i)
    this->m_vmassEle[i]->ComputeAndStore_Gradient();

  if (addBC) {
#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)m_vBC.size(); ++i)
      for (int j = 0; j < (int)m_vBC[i]->Energies().size(); ++j)
        m_vBC[i]->Energies()[j]->ComputeAndStore_Gradient();
  }
  if (m_isProfiling)
    this->m_timerComputeGrad.StopStoreLog();

  // Clean

  this->m_dirtyFlags = (DirtyFlags)(this->m_dirtyFlags & ~DirtyFlags::Gradient);

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s gradient computation",
                    this->GetName().c_str());
}

void Simulable::ComputeAndStore_Hessian(bool addBC) {
  if (!IsDirty_Hessian())
    return;  // Updated

  if (IsDirty_Rest())
    this->ComputeAndStore_Rest();

  if (IsDirty_Mass())
    this->ComputeAndStore_Mass();

  if (IsDirty_Mechanics())
    this->UpdateMechanics();

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Starting Simulable %s Hessian computation",
                    this->GetName().c_str());

  // Compute

  if (m_isProfiling)
    this->m_timerComputeHess.Start();
  if (this->m_vselEles.empty()) {
#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)this->m_venerEle.size(); ++i)
      this->m_venerEle[i]->ComputeAndStore_Hessian();
  } else {
#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)this->m_vselEles.size(); ++i)
      this->m_venerEle[m_vselEles[i]]->ComputeAndStore_Hessian();
  }

#ifdef NDEBUG
#pragma omp parallel for
#endif
  for (int i = 0; i < (int)this->m_vmassEle.size(); ++i)
    this->m_vmassEle[i]->ComputeAndStore_Hessian();

  if (addBC) {
#ifdef NDEBUG
#pragma omp parallel for
#endif
    for (int i = 0; i < (int)m_vBC.size(); ++i)
      for (int j = 0; j < (int)m_vBC[i]->Energies().size(); ++j)
        m_vBC[i]->Energies()[j]->ComputeAndStore_Hessian();
  }
  if (m_isProfiling)
    this->m_timerComputeHess.StopStoreLog();

  // Clean

  this->m_dirtyFlags = (DirtyFlags)(this->m_dirtyFlags & ~DirtyFlags::Hessian);

  IOUtils::logTrace(Verbosity::V4_DeepShit,
                    "\n[TRACE] Finishing Simulable %s Hessian computation",
                    this->GetName().c_str());
}

void Simulable::ComputeAndStore_Constraint(bool addBC) {
  //		int numEle = (int) this->m_vconsSet.size();
  //
  //		int numActiveCon = 0;
  //
  //		// Compute
  //
  //		if (m_isProfiling) this->m_timerComputeCons.Start();
  //#pragma omp parallel for
  //		for (int i = 0; i < numEle; ++i)
  //		{
  //			this->m_vconsSet[i]->ComputeAndStore_Constraint();
  //
  //			if (this->m_vconsSet[i]->IsActive())
  //			{
  //				this->m_vconsSet[i]->Offset() = numActiveCon;
  //				numActiveCon += this->m_vconsSet[i]->GetSize();
  //			}
  //			else
  //			{
  //				this->m_vconsSet[i]->Offset() = -1;
  //			}
  //		}
  //		if (m_isProfiling) this->m_timerAssembleCons.StopStoreLog();
  //
  //		// Assemble
  //
  //		if (m_isProfiling) this->m_timerAssembleCons.Start();
  //		this->m_vcons.resize(numActiveCon);
  //		for (size_t i = 0; i < numEle; ++i)
  //			this->m_vconsSet[i]->AssembleGlobal_Values(this->m_vcons);
  //		if (m_isProfiling) this->m_timerAssembleCons.StopStoreLog();
  //
  //		this->m_dirtyFlags = (DirtyFlags)(this->m_dirtyFlags &
  //~DirtyFlags::Constraint);
}

void Simulable::ComputeAndStore_Jacobian(bool addBC) {
  //		int numEle = (int) this->m_vconsSet.size();
  //
  //		// Compute
  //
  //		if (m_isProfiling) this->m_timerComputeJaco.Start();
  //#pragma omp parallel for
  //		for (int i = 0; i < numEle; ++i)
  //			if (this->m_vconsSet[i]->Offset() != -1)
  //				this->m_vconsSet[i]->ComputeAndStore_Jacobian();
  //		if (m_isProfiling) this->m_timerComputeJaco.StopStoreLog();
  //
  //		// Assemble
  //
  //		if (m_isProfiling) this->m_timerAssembleJaco.Start();
  //
  //		// Reserve space
  //
  //		int numTriplets = 0;
  //		for (size_t i = 0; i < numEle; ++i)
  //			numTriplets += m_vconsSet[i]->GetJacobianSize();
  //		this->m_mJaco.m_vvalueTripletsStatic.reserve(numTriplets);
  //
  //		// Gather triplets
  //
  //		for (size_t i = 0; i < numEle; ++i)
  //			this->m_vconsSet[i]->AssembleGlobal_Jacobian(this->m_mJaco.m_vvalueTripletsStatic);
  //
  //		this->m_mJaco.BuildMatrixFromTriplets();
  //
  //		if (m_isProfiling) this->m_timerAssembleJaco.StopStoreLog();
  //
  //		this->m_dirtyFlags = (DirtyFlags)(this->m_dirtyFlags &
  //~DirtyFlags::Jacobian);
}

void Simulable::TestLocalGradients() {
  for (int i = 0; i < (int)this->m_venerEle.size(); ++i)
    this->m_venerEle[i]->TestLocalGradient();
}

void Simulable::TestLocalHessians() {
  for (int i = 0; i < (int)this->m_venerEle.size(); ++i)
    this->m_venerEle[i]->TestLocalHessian();
}

void Simulable::TestGlobalGradient() {
  Real eps = 1e-6;

  VectorXd vxF;
  this->GetDOFVector(vxF, Tag::Position_X);

  VectorXd vs0;
  this->GetState(vs0);

  // Compute estimation
  VectorXd vgF(this->GetNumFullDOF());

  Real e = 0;

  for (int i = 0; i < this->GetNumFullDOF(); ++i) {
    // +
    SetState(vs0);
    vxF(i) += eps;
    this->SetDOFVector(vxF, Tag::Position_X);
    // this->ComputeAndStore_Energy();
    Real ep;
    GetEnergy(ep);

    // -
    SetState(vs0);
    vxF(i) -= 2 * eps;
    this->SetDOFVector(vxF, Tag::Position_X);
    // this->ComputeAndStore_Energy();
    Real em;
    GetEnergy(em);

    vgF(i) = (ep - em) / (2 * eps);

    vxF(i) += eps;
  }

  // Compute analytic
  this->SetState(vs0);

  AVectorXd vgA;
  GetGradient(vgA);

  // Object
  VectorXd vgD = vgA - vgF;
  Real testNorm = vgA.norm();
  Real realNorm = vgF.norm();
  Real diffNorm = vgD.norm();
  if (realNorm < 1e-6) {
    IOUtils::logTrace(Verbosity::V1_Default,
                      "\n[INVALID] Finite difference gradient near zero: %f",
                      realNorm);
  } else {
    if (diffNorm / realNorm > 1e-6) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[FAILURE] Global gradient test error: %f",
                        diffNorm / realNorm);
      // logFile("csvGradientTest_F.csv", vectorToString_CSV(vgF));
      // logFile("csvGradientTest_A.csv", vectorToString_CSV(vgA));
    } else {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[SUCCESS] Global gradient test error: %f",
                        diffNorm / realNorm);
    }
  }
}

void Simulable::TestGlobalHessian() {
  Real eps = 1e-6;

  VectorXd vxF;
  this->GetDOFVector(vxF, Tag::Position_X);

  VectorXd vs0;
  this->GetState(vs0);

  // Compute estimation
  MatrixXd mHF(this->GetNumFullDOF(), this->GetNumFullDOF());

  for (int i = 0; i < this->GetNumFullDOF(); ++i) {
    // +
    SetState(vs0);
    vxF(i) += eps;
    this->SetDOFVector(vxF, Tag::Position_X);
    // this->ComputeAndStore_Gradient();
    AVectorXd vgp;
    GetGradient(vgp);

    // -
    SetState(vs0);
    vxF(i) -= 2 * eps;
    this->SetDOFVector(vxF, Tag::Position_X);
    // this->ComputeAndStore_Gradient();
    AVectorXd vgm;
    GetGradient(vgm);

    mHF.col(i) = (vgp - vgm) / (2 * eps);

    vxF(i) += eps;
  }

  // Compute analytic
  this->SetState(vs0);

  AMatrixSd mHS;
  GetHessian(mHS);

  MatrixSd mHAS = mHS.selfadjointView<Lower>();
  MatrixXd mHA = MatrixXd(mHAS);

  // Object
  MatrixXd mHD = mHA - mHF;
  Real realNorm = mHF.norm();
  Real diffNorm = mHD.norm();
  if (realNorm < 1e-6) {
    IOUtils::logTrace(Verbosity::V1_Default,
                      "\n[INVALID] Finite difference Hessian near zero: %f",
                      realNorm);
  } else {
    if (diffNorm / realNorm > 1e-6) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[FAILURE] Global Hessian test error: %f",
                        diffNorm / realNorm);
      IOUtils::logFile("csvHessianTest_F.csv",
                       IOUtils::matrixToString_CSV(mHF));
      IOUtils::logFile("csvHessianTest_A.csv",
                       IOUtils::matrixToString_CSV(mHA));
    } else {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[SUCCESS] Global Hessian test error: %f",
                        diffNorm / realNorm);
      IOUtils::logFile("csvHessianTest_F.csv",
                       IOUtils::matrixToString_CSV(mHF));
      IOUtils::logFile("csvHessianTest_A.csv",
                       IOUtils::matrixToString_CSV(mHA));
    }
  }
}

void Simulable::TestGlobalJacobian() {
  // TODO
}

void Simulable::AddEnergyElement(PtrS<IEnergyElement>& pEle) {}

void Simulable::AddConstraintSet(PtrS<IConstraintSet>& pCon) {}

void Simulable::AddMassElement(PtrS<IMassElement>& pMass) {}

void Simulable::AddBoundaryCondition(PtrS<IBCondition> pBC) {
  vector<PtrS<IBCondition>>::iterator it =
      find(this->m_vBC.begin(), this->m_vBC.end(), pBC);

  pBC->Init();

  assert(it == m_vBC.end());
  this->m_vBC.push_back(pBC);

  if (!pBC->Energies().empty()) {
    this->DirtyMechanics();
  }

  if (!pBC->Constraints().empty()) {
    this->DirtyMechanics();
  }
}

void Simulable::RemoveBoundaryCondition(PtrS<IBCondition> pBC) {
  vector<PtrS<IBCondition>>::iterator it =
      find(this->m_vBC.begin(), this->m_vBC.end(), pBC);

  assert(it != m_vBC.end());
  this->m_vBC.erase(it);

  if (!pBC->Energies().empty()) {
    this->DirtyMechanics();
  }

  if (!pBC->Constraints().empty()) {
    this->DirtyMechanics();
  }
}

void Simulable::ClearBoundaryConditions() {
  for (size_t i = 0; i < this->m_vBC.size(); ++i) {
    if (!this->m_vBC[i]->Energies().empty()) {
      this->DirtyMechanics();
    }

    if (!this->m_vBC[i]->Constraints().empty()) {
      this->DirtyMechanics();
    }
  }
  this->m_vBC.clear();
}

bool Simulable::BoundaryConditionsLoaded() {
  for (size_t i = 0; i < this->m_vBC.size(); ++i)
    if (!this->m_vBC[i]->IsLoaded())
      return false;

  return true;
}

void Simulable::ResetBoundaryConditions() {
  for (size_t i = 0; i < this->m_vBC.size(); ++i)
    if (this->m_vBC[i]->ResetLoading())
      this->m_vBC[i]->Update();
}

void Simulable::FullBoundaryConditions() {
  for (size_t i = 0; i < this->m_vBC.size(); ++i)
    if (this->m_vBC[i]->FullLoading())
      this->m_vBC[i]->Update();
}

void Simulable::StepBoundaryConditions() {
  for (size_t i = 0; i < this->m_vBC.size(); ++i)
    if (this->m_vBC[i]->StepLoading())
      this->m_vBC[i]->Update();
}

}  // namespace PhySim