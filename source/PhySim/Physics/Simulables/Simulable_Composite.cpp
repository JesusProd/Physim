//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Simulables/Simulable_Composite.h>

#include <PhySim/Kinematics/KEleParticle3D.h>
#include <PhySim/Kinematics/KEleRigidBody3D.h>
#include <PhySim/Kinematics/KMapIdentity.h>
#include <PhySim/Kinematics/KMapRB2Particle3D.h>
#include <PhySim/Kinematics/KMapSplit.h>

#include <PhySim/Geometry/Meshes/Mesh.h>
#include <PhySim/Geometry/Meshes/Mesh_Edge.h>
#include <PhySim/Geometry/Meshes/Mesh_Hexa.h>
#include <PhySim/Geometry/Meshes/Mesh_Quad.h>
#include <PhySim/Geometry/Meshes/Mesh_Tetra.h>
#include <PhySim/Geometry/Meshes/Mesh_Tri.h>

#include <PhySim/Geometry/Polytopes/Cell_Hexa.h>
#include <PhySim/Geometry/Polytopes/Cell_Tetra.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Embedding.h>
#include <PhySim/Geometry/Polytopes/Face_Quad.h>
#include <PhySim/Geometry/Polytopes/Face_Tri.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Physics/Elements/EnergyElement.h>
#include <PhySim/Physics/Elements/EnergyElement_Gravity.h>
#include <PhySim/Physics/Elements/MassElement_Lumped.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Simulable_Composite::Simulable_Composite() : Simulable() {
  this->m_pOptions = NULL;
}

Simulable_Composite::~Simulable_Composite() {
  this->FreeInternal();

  if (this->m_pOptions != NULL)
    delete this->m_pOptions;
  this->m_pOptions = NULL;

#ifndef NDEBUG
  IOUtils::logTrace(Verbosity::V1_Default,
                    "\n[DEBUG] Deleting Simulable_Composite");
#endif
}

Simulable_Composite::Options& Simulable_Composite::SetupOptions() {
  if (this->m_pOptions == NULL)  // Create if needed
    this->m_pOptions = new Simulable_Composite::Options();
  return *((Simulable_Composite::Options*)this->m_pOptions);
}

void Simulable_Composite::FreeInternal() {
  // for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i)
  //	this->m_pOptions->m_vpSimulables[i]->FreeInternal();
}

void Simulable_Composite::InitInternal() {
  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->Setup();  // Setup each model

  // Process couplings

  vector<PtrS<KinematicsEle>> vpRoots;
  vector<PtrS<KinematicsEle>> vpMapped;
  vector<PtrS<EnergyElement>> vpEnergy;

  for (int i = 0; i < (int)this->m_pOptions->m_vDoFCouplings.size(); ++i) {
    DoFCoupling::Type type = this->m_pOptions->m_vDoFCouplings[i].m_type;

    switch (type) {
      case DoFCoupling::Type::SoftFixed:
        addSoftFixedCoupling(this->m_pOptions->m_vDoFCouplings[i], vpMapped,
                             vpRoots, vpEnergy);
        break;
      case DoFCoupling::Type::HardFixed:
        addHardFixedCoupling(this->m_pOptions->m_vDoFCouplings[i], vpMapped,
                             vpRoots, vpEnergy);
        break;
      case DoFCoupling::Type::SoftEmbedding:
        addSoftEmbeddingCoupling(this->m_pOptions->m_vDoFCouplings[i], vpMapped,
                                 vpRoots, vpEnergy);
        break;
      case DoFCoupling::Type::HardEmbedding:
        addHardEmbeddingCoupling(this->m_pOptions->m_vDoFCouplings[i], vpMapped,
                                 vpRoots, vpEnergy);
        break;
      case DoFCoupling::Type::RigidBody:
        addRigidBodyCoupling(this->m_pOptions->m_vDoFCouplings[i], vpMapped,
                             vpRoots, vpEnergy);
        break;
    }
  }

  // Gather active kinematics

  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i) {
    PtrS<KinematicsTree> pTree =
        this->m_pOptions->m_vpSimulables[i]->GetKinematicsTree();

    for (int j = 0; j < (int)pTree->RootElements().size(); ++j) {
      PtrS<KinematicsEle> pEle = pTree->RootElements()[j];

      if (std::find(vpMapped.begin(), vpMapped.end(), pEle) != vpMapped.end())
        continue;  // This kinematic element has been already mapped -> Ignore

      PtrS<KinematicsEle> pNew = pEle->Clone();
      (new KMapIdentity())->Init(this, pNew, pEle);
      pNew->SetModel(this);
      pNew->Layer() = m_ID;
      pNew->Active() = true;
      vpRoots.push_back(pNew);
    }
  }

  // Create kinematic tree

  this->m_pTree.reset(new KinematicsTree(this, vpRoots));

  // Add energy elements

  this->m_venerEle.insert(this->m_venerEle.end(), vpEnergy.begin(),
                          vpEnergy.end());
}

bool Simulable_Composite::IsDirty_Energy() const {
  bool dirty = Simulable::IsDirty_Energy();
  for (size_t i = 0; i < m_pOptions->m_vpSimulables.size() && !dirty; ++i)
    dirty |= this->m_pOptions->m_vpSimulables[i]->IsDirty_Energy();
  return dirty;
}

bool Simulable_Composite::IsDirty_Gradient() const {
  bool dirty = Simulable::IsDirty_Gradient();
  for (size_t i = 0; i < m_pOptions->m_vpSimulables.size() && !dirty; ++i)
    dirty |= this->m_pOptions->m_vpSimulables[i]->IsDirty_Gradient();
  return dirty;
}

bool Simulable_Composite::IsDirty_Hessian() const {
  bool dirty = Simulable::IsDirty_Hessian();
  for (size_t i = 0; i < m_pOptions->m_vpSimulables.size() && !dirty; ++i)
    dirty |= this->m_pOptions->m_vpSimulables[i]->IsDirty_Hessian();
  return dirty;
}

bool Simulable_Composite::IsDirty_Mass() const {
  bool dirty = Simulable::IsDirty_Mass();
  for (size_t i = 0; i < m_pOptions->m_vpSimulables.size() && !dirty; ++i)
    dirty |= this->m_pOptions->m_vpSimulables[i]->IsDirty_Mass();
  return dirty;
}

bool Simulable_Composite::IsDirty_Rest() const {
  bool dirty = Simulable::IsDirty_Rest();
  for (size_t i = 0; i < m_pOptions->m_vpSimulables.size() && !dirty; ++i)
    dirty |= this->m_pOptions->m_vpSimulables[i]->IsDirty_Rest();
  return dirty;
}

bool Simulable_Composite::IsDirty_Constraint() const {
  bool dirty = Simulable::IsDirty_Constraint();
  for (size_t i = 0; i < m_pOptions->m_vpSimulables.size() && !dirty; ++i)
    dirty |= this->m_pOptions->m_vpSimulables[i]->IsDirty_Constraint();
  return dirty;
}

bool Simulable_Composite::IsDirty_Jacobian() const {
  bool dirty = Simulable::IsDirty_Jacobian();
  for (size_t i = 0; i < m_pOptions->m_vpSimulables.size() && !dirty; ++i)
    dirty |= this->m_pOptions->m_vpSimulables[i]->IsDirty_Jacobian();
  return dirty;
}

bool Simulable_Composite::IsDirty_Fixed() const {
  bool dirty = Simulable::IsDirty_Fixed();
  for (size_t i = 0; i < m_pOptions->m_vpSimulables.size() && !dirty; ++i)
    dirty |= this->m_pOptions->m_vpSimulables[i]->IsDirty_Fixed();
  return dirty;
}

bool Simulable_Composite::IsDirty_Kinematics() const {
  bool dirty = Simulable::IsDirty_Kinematics();
  for (size_t i = 0; i < m_pOptions->m_vpSimulables.size() && !dirty; ++i)
    dirty |= this->m_pOptions->m_vpSimulables[i]->IsDirty_Kinematics();
  return dirty;
}

bool Simulable_Composite::IsDirty_Mechanics() const {
  bool dirty = Simulable::IsDirty_Mechanics();
  for (size_t i = 0; i < m_pOptions->m_vpSimulables.size() && !dirty; ++i)
    dirty |= this->m_pOptions->m_vpSimulables[i]->IsDirty_Mechanics();
  return dirty;
}

void Simulable_Composite::DirtyMass() {
  Simulable::DirtyMass();

  for (auto pSim : this->m_pOptions->m_vpSimulables)
    pSim->DirtyMass();
}

void Simulable_Composite::DirtyRest() {
  Simulable::DirtyRest();

  for (auto pSim : this->m_pOptions->m_vpSimulables)
    pSim->DirtyRest();
}

void Simulable_Composite::DirtyMechanics() {
  Simulable::DirtyMechanics();

  for (auto pSim : this->m_pOptions->m_vpSimulables)
    pSim->DirtyMechanics();
}

void Simulable_Composite::DirtyFixed() {
  Simulable::DirtyFixed();

  for (auto pSim : this->m_pOptions->m_vpSimulables)
    pSim->DirtyFixed();
}

void Simulable_Composite::AddEnergy(Real& energy, bool addBC) {
  Simulable::AddEnergy(energy, addBC);

  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->AddEnergy(energy, addBC);
}

void Simulable_Composite::AddGradient(AVectorXd& vgradient, bool addBC) {
  Simulable::AddGradient(vgradient, addBC);

  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->AddGradient(vgradient, addBC);
}

void Simulable_Composite::AddHessian(AMatrixSd& mHessian, bool addBC) {
  Simulable::AddHessian(mHessian, addBC);

  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->AddHessian(mHessian, addBC);
}

void Simulable_Composite::SetUseFDGradientAtElements(bool u) {
  Simulable::SetUseFDGradientAtElements(u);

  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->SetUseFDGradientAtElements(u);
}

void Simulable_Composite::SetUseFDHessianAtElements(bool u) {
  Simulable::SetUseFDHessianAtElements(u);

  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->SetUseFDHessianAtElements(u);
}

void Simulable_Composite::UpdateMechanics() {
  Simulable::UpdateMechanics();

  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->UpdateMechanics();
}

void Simulable_Composite::ComputeAndStore_Fixed() {
  Simulable::ComputeAndStore_Fixed();

  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->CleanFlags_Fixed();
}

void Simulable_Composite::ComputeAndStore_Energy(bool addBC) {
  Simulable::ComputeAndStore_Energy(addBC);

  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->ComputeAndStore_Energy(addBC);
}

void Simulable_Composite::ComputeAndStore_Gradient(bool addBC) {
  Simulable::ComputeAndStore_Gradient(addBC);

  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->ComputeAndStore_Gradient(addBC);
}

void Simulable_Composite::ComputeAndStore_Hessian(bool addBC) {
  Simulable::ComputeAndStore_Hessian(addBC);

  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->ComputeAndStore_Hessian(addBC);
}

void Simulable_Composite::addSoftFixedCoupling(
    DoFCoupling& coupling,
    vector<PtrS<KinematicsEle>>& vpMapped,
    vector<PtrS<KinematicsEle>>& vpAdded,
    vector<PtrS<EnergyElement>>& vpEnergy) {
  // TODO
}

void Simulable_Composite::addHardFixedCoupling(
    DoFCoupling& coupling,
    vector<PtrS<KinematicsEle>>& vpMapped,
    vector<PtrS<KinematicsEle>>& vpAdded,
    vector<PtrS<EnergyElement>>& vpEnergy) {
  assert(coupling.m_type == DoFCoupling::Type::HardFixed);

  PtrS<KinematicsTree> pTree0 =
      this->m_pOptions->m_vpSimulables[coupling.m_mC(0, 0)]
          ->GetKinematicsTree();
  PtrS<KinematicsTree> pTree1 =
      this->m_pOptions->m_vpSimulables[coupling.m_mC(1, 0)]
          ->GetKinematicsTree();
  PtrS<KinematicsEle> pEle0 = pTree0->RootElements()[coupling.m_mC(0, 1)];
  PtrS<KinematicsEle> pEle1 = pTree1->RootElements()[coupling.m_mC(1, 1)];

  assert(pEle0->NumDim() == pEle1->NumDim());

  PtrS<KinematicsEle> pNew = pEle0->Clone();
  vector<PtrS<KinematicsEle>> vOutEle(2);
  vOutEle[0] = pEle0;
  vOutEle[1] = pEle1;
  pNew->SetModel(this);
  pNew->Layer() = m_ID;
  pNew->Active() = true;
  (new KMapSplit())->Init(this, pNew, vOutEle);
  vpAdded.push_back(pNew);
  vpMapped.push_back(pEle0);
  vpMapped.push_back(pEle1);
}

void Simulable_Composite::addRigidBodyCoupling(
    DoFCoupling& coupling,
    vector<PtrS<KinematicsEle>>& vpMapped,
    vector<PtrS<KinematicsEle>>& vpAdded,
    vector<PtrS<EnergyElement>>& vpEnergy) {
  assert(coupling.m_type == DoFCoupling::Type::RigidBody);

  PtrS<KinematicsTree> pTreeRigid =
      this->m_pOptions->m_vpSimulables[coupling.m_mC(0, 0)]
          ->GetKinematicsTree();
  PtrS<KinematicsTree> pTreeOther =
      this->m_pOptions->m_vpSimulables[coupling.m_mC(0, 2)]
          ->GetKinematicsTree();

  // Get rigid body
  PtrS<KinematicsEle> pEleRigid =
      pTreeRigid->RootElements()[coupling.m_mC(0, 1)];
  KEleRigidBody3D* pTest = dynamic_cast<KEleRigidBody3D*>(pEleRigid.get());
  assert(pTest != NULL);

  // Get particles
  vector<PtrS<KinematicsEle>> vpEleOther(coupling.m_mC.cols() - 3);
  for (int i = 0; i < coupling.m_mC.cols() - 3; ++i) {
    vpEleOther[i] = (pTreeOther->RootElements()[coupling.m_mC(0, i + 3)]);
    KEleParticle3D* pTest = dynamic_cast<KEleParticle3D*>(vpEleOther[i].get());
    assert(pTest != NULL);
  }

  // Create new rigid and map it (identity)

  PtrS<KinematicsEle> pNewRigid = pEleRigid->Clone();
  pNewRigid->SetModel(this);
  pNewRigid->Layer() = m_ID;
  pNewRigid->Active() = true;
  (new KMapIdentity())->Init(this, pNewRigid, pEleRigid);
  vpAdded.push_back(pNewRigid);
  vpMapped.push_back(pEleRigid);

  // Map the new rigid to the 3D particles

  (new KMapRB2Particle3D())
      ->Init(this, dynamic_pointer_cast<KEleRigidBody3D>(pNewRigid),
             vpEleOther);
  vpMapped.insert(vpMapped.end(), vpEleOther.begin(), vpEleOther.end());
}

void Simulable_Composite::addSoftEmbeddingCoupling(
    DoFCoupling& coupling,
    vector<PtrS<KinematicsEle>>& vpMapped,
    vector<PtrS<KinematicsEle>>& vpAdded,
    vector<PtrS<EnergyElement>>& vpEnergy) {
  // TODO
}

void Simulable_Composite::addHardEmbeddingCoupling(
    DoFCoupling& coupling,
    vector<PtrS<KinematicsEle>>& vpMapped,
    vector<PtrS<KinematicsEle>>& vpAdded,
    vector<PtrS<EnergyElement>>& vpEnergy) {
  // TODO
}

vector<IDoFSet*> Simulable_Composite::SelectDoF(const Vector3d& vboxMin,
                                                const Vector3d& vboxMax,
                                                Tag s) const {
  vector<IDoFSet*> vDoFs;

  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i) {
    vector<IDoFSet*> vDoFs_i =
        this->m_pOptions->m_vpSimulables[i]->SelectDoF(vboxMin, vboxMax, s);
    vDoFs.insert(vDoFs.end(), vDoFs_i.begin(), vDoFs_i.end());
  }

  return vDoFs;
}

vector<PtrS<Geometry>> Simulable_Composite::Geometries() const {
  vector<PtrS<Geometry>> vpGeometries;

  for (int i = 0; i < (int)this->m_pOptions->m_vpSimulables.size(); ++i) {
    vector<PtrS<Geometry>> vpGeometries_i =
        m_pOptions->m_vpSimulables[i]->Geometries();
    vpGeometries.insert(vpGeometries.end(), vpGeometries_i.begin(),
                        vpGeometries_i.end());
  }

  return vpGeometries;
}

bool Simulable_Composite::BoundaryConditionsLoaded() {
  bool loaded = true;

  for (size_t i = 0; i < this->m_pOptions->m_vpSimulables.size(); ++i)
    loaded &= this->m_pOptions->m_vpSimulables[i]->BoundaryConditionsLoaded();

  loaded &= Simulable::BoundaryConditionsLoaded();

  return loaded;
}

void Simulable_Composite::ResetBoundaryConditions() {
  for (size_t i = 0; i < this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->ResetBoundaryConditions();

  Simulable::ResetBoundaryConditions();
}

void Simulable_Composite::FullBoundaryConditions() {
  for (size_t i = 0; i < this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->FullBoundaryConditions();

  Simulable::FullBoundaryConditions();
}

void Simulable_Composite::StepBoundaryConditions() {
  for (size_t i = 0; i < this->m_pOptions->m_vpSimulables.size(); ++i)
    this->m_pOptions->m_vpSimulables[i]->StepBoundaryConditions();

  Simulable::StepBoundaryConditions();
}

}  // namespace PhySim