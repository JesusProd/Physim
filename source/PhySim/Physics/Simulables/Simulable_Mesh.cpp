//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Simulables/Simulable_Mesh.h>

#include <PhySim/Kinematics/KEleParticle3D.h>
#include <PhySim/Kinematics/KinematicsTree.h>

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

Simulable_Mesh::Simulable_Mesh() : Simulable() {
  this->m_pOptions = NULL;

  this->m_pMesh = NULL;
}

Simulable_Mesh::~Simulable_Mesh() {
  this->FreeInternal();

  if (this->m_pOptions != NULL)
    delete this->m_pOptions;
  this->m_pOptions = NULL;

#ifndef NDEBUG
  IOUtils::logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Simulable_Mesh");
#endif
}

Simulable_Mesh::Options& Simulable_Mesh::SetupOptions() {
  if (this->m_pOptions == NULL)  // Create if needed
    this->m_pOptions = new Simulable_Mesh::Options();
  return *((Simulable_Mesh::Options*)this->m_pOptions);
}

void Simulable_Mesh::FreeInternal() {
  Simulable::FreeInternal();
}

void Simulable_Mesh::InitInternal() {
  // Code before initialization

  this->PreInit();

  // Create simulation mesh

  this->CreateDiscretization(this->m_pMesh);

  // Create KinematicsEle

  vector<PtrS<KinematicsEle>> vpEle;
  this->CreateKinematicsEle(vpEle);
  this->m_pTree.reset(new KinematicsTree(this, vpEle));

  // Create energy elements

  this->m_venerEle.clear();
  vector<IEnergyElement*> venerEle;
  this->CreateEnergyElements(venerEle);
  this->m_venerEle.reserve((int)venerEle.size());
  for (int i = 0; i < (int)venerEle.size(); ++i)
    this->m_venerEle.push_back(PtrS<IEnergyElement>(venerEle[i]));

  // Create constraint sets

  this->m_vconsSet.clear();
  vector<IConstraintSet*> vconsSet;
  this->CreateConstraintSets(vconsSet);
  this->m_vconsSet.reserve((int)vconsSet.size());
  for (int i = 0; i < (int)vconsSet.size(); ++i)
    this->m_vconsSet.push_back(PtrS<IConstraintSet>(vconsSet[i]));

  // Create mass element

  this->m_vmassEle.clear();
  vector<IMassElement*> vmassEle;
  this->CreateMassElements(vmassEle);
  this->m_vmassEle.reserve((int)vmassEle.size());
  for (int i = 0; i < (int)vmassEle.size(); ++i)
    this->m_vmassEle.push_back(PtrS<IMassElement>(vmassEle[i]));

  // Code after initialization

  this->PostInit();
}

void Simulable_Mesh::PreInit() {
  //{
  //	// Setup default material parameters

  //	if (this->SetupOptions().m_pMatParams == NULL)
  //	{
  //		PtrS<ParameterSet> pMatParam(new ParameterSet());
  //		pMatParam->InitLinearFromYoungPoisson(1e9, 0.3, 1000);
  //		this->SetupOptions().m_pMatParams.reset(new
  //DomainDistribution_Constant<PtrS<ParameterSet>>(pMatParam));
  //	}
}

void Simulable_Mesh::PostInit() {
  // Nothing to do here...
}

void Simulable_Mesh::CreateDiscretization(PtrS<Mesh>& pMesh) {
  // Use the same mesh in the options

  pMesh = this->m_pOptions->m_pMesh;
}

void Simulable_Mesh::CreateKinematicsEle(vector<PtrS<KinematicsEle>>& vpEle) {
  vpEle.resize(this->m_pMesh->NumNodes());

  // Create particle DoF and add them to the mesh

  for (size_t i = 0; i < this->m_pMesh->NumNodes(); ++i) {
    vpEle[i].reset(new KEleParticle3D(this, this->m_pMesh->Nodes()[i]));
    m_pMesh->Nodes()[i]->Traits().AddTrait<IDoFSet*>(Tag::Tag_DOF_0, vpEle[i].get());
  }
}

void Simulable_Mesh::CreateEnergyElements(vector<IEnergyElement*>& vEnergies) {
  // Nothing to do here... to be implemented by derived classes
}

void Simulable_Mesh::CreateConstraintSets(
    vector<IConstraintSet*>& vConstraints) {
  // Nothing to do here... to be implemented by derived classes
}

void Simulable_Mesh::CreateMassElements(vector<IMassElement*>& vMasses) {
  vMasses.resize(this->m_pMesh->NumElems());

  // Default implementation: create lumping mass components for each of
  // the elements of the simulated mesh. The mass of the element will be
  // computed based on the volume of the element at rest configuration
  // and the material density.

  for (size_t i = 0; i < this->m_pMesh->NumElems(); ++i) {
    PtrS<ParameterSet> pPar =
        m_pOptions->m_pMatParams->GetValueAtDomainPoint(i);
    vMasses[i] = new MassElement_Lumped(this, this->m_pMesh->Elems()[i], pPar);
  }
}

vector<PtrS<Geometry>> Simulable_Mesh::Geometries() const {
  vector<PtrS<Geometry>> vpGeometries;
  vpGeometries.push_back(this->m_pMesh);
  return vpGeometries;
}

void Simulable_Mesh::InitSubelementPositions(const MatrixXd& mV, Tag s) {
  // assert(mV.rows() == m_pOptions->m_mNodes.rows());

  if (s == Tag::Tag_Position_0) {
    this->m_pMesh->SetNodesTrait(mV, Tag::Tag_Position_0);
    for (int i = 0; i < this->m_pMesh->NumElems(); ++i)
      this->m_pMesh->Elems()[i]->InitSubelementPositions(Tag::Tag_Position_0);

    this->DirtyRest();
  }

  if (s == Tag::Tag_Position_X) {
    this->m_pMesh->SetNodesTrait(mV, Tag::Tag_Position_X);
    for (int i = 0; i < this->m_pMesh->NumElems(); ++i)
      this->m_pMesh->Elems()[i]->InitSubelementPositions(Tag::Tag_Position_X);

    this->DirtyRest();
  }
}

vector<IDoFSet*> Simulable_Mesh::SelectDoF(const Vector3d& vboxMin,
                                           const Vector3d& vboxMax,
                                           Tag s) const {
  vector<IDoFSet*> vDoFs;

  Tag ns;
  if (s == Tag::Tag_Position_0)
    ns = Tag::Tag_Position_0;
  if (s == Tag::Tag_Position_X)
    ns = Tag::Tag_Position_X;

  for (size_t i = 0; i < this->m_pMesh->NumNodes(); ++i) {
    const Vector3d& pos = this->m_pMesh->Nodes()[i]->Traits().Vector3d(ns);
    if (pos.x() > vboxMin.x() && pos.x() < vboxMax.x() &&
        pos.y() > vboxMin.y() && pos.y() < vboxMax.y() &&
        pos.z() > vboxMin.z() && pos.z() < vboxMax.z())
      vDoFs.push_back(
          this->m_pMesh->Nodes()[i]->Traits().Kinematics(Tag::Tag_DOF_0));
  }

  return vDoFs;
}

}  // namespace PhySim