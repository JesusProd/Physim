//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Simulables/Simulable_MassSpring.h>

#include <PhySim/Physics/Elements/EnergyElement_Gravity.h>
#include <PhySim/Physics/Elements/EnergyElement_SpringCross.h>
#include <PhySim/Physics/Elements/EnergyElement_SpringHinge.h>
#include <PhySim/Physics/Elements/EnergyElement_SpringLinear.h>
#include <PhySim/Physics/Elements/MassElement_Lumped.h>

#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Node.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Simulable_MassSpring::Simulable_MassSpring() : Simulable_Mesh() {
  // Nothing to do here...
}

Simulable_MassSpring::~Simulable_MassSpring() {
  // Nothing to do here...
}

void Simulable_MassSpring::FreeInternal() {
  Simulable_Mesh::FreeInternal();

  this->m_venergyEle_linear.clear();
  this->m_venergyEle_hinge.clear();
  this->m_venergyEle_cross.clear();
  m_vmat_linear.clear();
  m_vmat_hinge.clear();
  m_vmat_cross.clear();
}

Simulable_MassSpring::Options& Simulable_MassSpring::SetupOptions() {
  if (this->m_pOptions == NULL)  // Create if needed
    this->m_pOptions = new Simulable_MassSpring::Options();
  return *((Simulable_MassSpring::Options*)this->m_pOptions);
}

void Simulable_MassSpring::CreateEnergyElements(
    vector<IEnergyElement*>& vEnergies) {
  Simulable_Mesh::CreateEnergyElements(vEnergies);

  Options* pOptions = &SetupOptions();

  const VectorXi& m_vlinear = pOptions->m_springs.m_vlinear;
  const MatrixXi& m_mhinges = pOptions->m_springs.m_mhinges;
  const MatrixXi& m_mcrosses = pOptions->m_springs.m_mcrosses;
  size_t numLinear = m_vlinear.size();
  size_t numCrosses = m_mcrosses.rows();
  size_t numHinges = m_mhinges.rows();

  this->m_venerEle.reserve(this->m_venerEle.size() + numLinear + numCrosses +
                           numHinges);
  this->m_venergyEle_linear.resize(numLinear);
  this->m_venergyEle_cross.resize(numCrosses);
  this->m_venergyEle_hinge.resize(numHinges);

  if (numLinear > 0) {
    ;
    assert(pOptions->m_material.HasParameter(ParameterSet::Param_StretchK));
  }
  if (numHinges > 0) {
    ;
    assert(pOptions->m_material.HasParameter(ParameterSet::Param_BendingK));
  }
  if (numCrosses > 0) {
    ;
    assert(pOptions->m_material.HasParameter(ParameterSet::Param_ShearK));
  }

  // Create energy elements (linear)

  m_vmat_linear.resize(numLinear);
  for (size_t i = 0; i < numLinear; ++i) {
    m_vmat_linear[i].AddParameter(
        ParameterSet::Param_StretchK,
        pOptions->m_material[ParameterSet::Param_StretchK]);
    m_vmat_linear[i].AddParameter(
        ParameterSet::Param_Density,
        pOptions->m_material[ParameterSet::Param_Density]);

    EnergyElement_SpringLinear* pEle = new EnergyElement_SpringLinear(
        this, (Edge*)this->m_pMesh->Elems()[m_vlinear(i)],
        &this->m_vmat_linear[i]);
    vEnergies.push_back(pEle);
    this->m_venergyEle_linear[i] = pEle;
  }

  // Create energy elements (hinges)

  m_vmat_hinge.resize(numHinges);
  for (size_t i = 0; i < numHinges; ++i) {
    m_vmat_hinge[i].AddParameter(
        ParameterSet::Param_BendingK,
        pOptions->m_material[ParameterSet::Param_BendingK]);

    EnergyElement_SpringHinge* pEle = new EnergyElement_SpringHinge(
        this, (Edge*)this->m_pMesh->Elems()[m_mhinges(i, 0)],
        (Edge*)this->m_pMesh->Elems()[m_mhinges(i, 1)], &this->m_vmat_hinge[i]);
    vEnergies.push_back(pEle);
    this->m_venergyEle_hinge[i] = pEle;
  }

  // Create energy elements (cross)

  m_vmat_cross.resize(numCrosses);
  for (size_t i = 0; i < numCrosses; ++i) {
    m_vmat_cross[i].AddParameter(
        ParameterSet::Param_ShearK,
        pOptions->m_material[ParameterSet::Param_ShearK]);

    EnergyElement_SpringCross* pEle = new EnergyElement_SpringCross(
        this, (Edge*)this->m_pMesh->Elems()[m_mcrosses(i, 0)],
        (Edge*)this->m_pMesh->Elems()[m_mcrosses(i, 1)],
        &this->m_vmat_cross[i]);
    vEnergies.push_back(pEle);
    this->m_venergyEle_cross[i] = pEle;
  }
}

VectorXd Simulable_MassSpring::GetRestLinearLengths() const {
  size_t numLinear = this->m_venergyEle_linear.size();

  VectorXd vl(numLinear);

  for (size_t i = 0; i < numLinear; ++i) {
    vl[i] = this->m_venergyEle_linear[i]->GetRestLength();
  }

  return vl;
}

void Simulable_MassSpring::SetRestLinearLengths(const VectorXd& vl) {
  size_t numLinear = this->m_venergyEle_linear.size();

  assert(vl.size() == numLinear);

  for (size_t i = 0; i < numLinear; ++i) {
    this->m_venergyEle_linear[i]->SetRestLength(vl[i]);
  }

  this->DirtyMechanics();
}

VectorXd Simulable_MassSpring::GetRestHingeAngles() const {
  size_t numHinges = this->m_venergyEle_hinge.size();

  VectorXd vl(numHinges);

  for (size_t i = 0; i < numHinges; ++i) {
    vl[i] = this->m_venergyEle_hinge[i]->GetRestAngle();
  }

  return vl;
}

void Simulable_MassSpring::SetRestHingeAngles(const VectorXd& va) {
  size_t numHinges = this->m_venergyEle_hinge.size();

  assert(va.size() == numHinges);

  for (size_t i = 0; i < numHinges; ++i) {
    this->m_venergyEle_hinge[i]->SetRestAngle(va[i]);
  }

  this->DirtyMechanics();
}

VectorXd Simulable_MassSpring::GetRestCrossStrain() const {
  size_t numCrosses = this->m_venergyEle_cross.size();

  VectorXd vr(numCrosses);

  for (size_t i = 0; i < numCrosses; ++i) {
    vr[i] = this->m_venergyEle_cross[i]->GetRestStrain();
  }

  return vr;
}

void Simulable_MassSpring::SetRestCrossStrain(const VectorXd& vr) {
  size_t numCrosses = this->m_venergyEle_cross.size();

  assert(vr.size() == numCrosses);

  for (size_t i = 0; i < numCrosses; ++i) {
    this->m_venergyEle_cross[i]->GetRestStrain(vr[i]);
  }

  this->DirtyMechanics();
}

VectorXd Simulable_MassSpring::GetStretchK() const {
  size_t numLinear = this->m_venergyEle_linear.size();

  VectorXd vs(numLinear);

  for (size_t i = 0; i < numLinear; ++i) {
    vs[i] = (*this->m_venergyEle_linear[i]
                  ->GetMaterial())[ParameterSet::Param_StretchK];
  }

  return vs;
}

VectorXd Simulable_MassSpring::GetBendingK() const {
  size_t numHinges = this->m_venergyEle_hinge.size();

  VectorXd vs(numHinges);

  for (size_t i = 0; i < numHinges; ++i) {
    vs[i] = (*this->m_venergyEle_hinge[i]
                  ->GetMaterial())[ParameterSet::Param_BendingK];
  }

  return vs;
}

VectorXd Simulable_MassSpring::GetShearK() const {
  size_t numCrosses = this->m_venergyEle_cross.size();

  VectorXd vs(numCrosses);

  for (size_t i = 0; i < numCrosses; ++i) {
    vs[i] = (*this->m_venergyEle_cross[i]
                  ->GetMaterial())[ParameterSet::Param_ShearK];
  }

  return vs;
}

void Simulable_MassSpring::SetStretchK(const VectorXd& vs) {
  size_t numLinear = this->m_venergyEle_linear.size();

  assert(vs.size() == numLinear);

  for (size_t i = 0; i < numLinear; ++i) {
    (*this->m_venergyEle_linear[i]
          ->GetMaterial())[ParameterSet::Param_StretchK] = vs[i];
  }

  this->DirtyMechanics();
}

void Simulable_MassSpring::SetBendingK(const VectorXd& vs) {
  size_t numHinges = this->m_venergyEle_hinge.size();

  assert(vs.size() == numHinges);

  for (size_t i = 0; i < numHinges; ++i) {
    (*this->m_venergyEle_hinge[i]
          ->GetMaterial())[ParameterSet::Param_BendingK] = vs[i];
  }

  this->DirtyMechanics();
}

void Simulable_MassSpring::SetShearK(const VectorXd& vs) {
  size_t numCrosses = this->m_venergyEle_cross.size();

  assert(vs.size() == numCrosses);

  for (size_t i = 0; i < numCrosses; ++i) {
    (*this->m_venergyEle_cross[i]->GetMaterial())[ParameterSet::Param_ShearK] =
        vs[i];
  }

  this->DirtyMechanics();
}
}  // namespace PhySim