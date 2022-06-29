//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/MassElement_Lumped.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Physics/DoFSet.h>
#include <PhySim/Physics/Simulables/Simulable.h>
#include <PhySim/Physics/Simulables/Simulable_FEM.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

MassElement_Lumped::MassElement_Lumped(Simulable* pModel,
                                       Poly* pPoly,
                                       PtrS<ParameterSet> pParams)
    : MassElement(pModel, pParams) {
  this->m_pPoly = pPoly;

  int N = 0;

  this->m_vDoF.resize(pPoly->NumNodes());
  for (int i = 0; i < pPoly->NumNodes(); ++i) {
    this->m_vDoF[i] = pPoly->Nodes()[i]->Traits().Kinematics(Tag::DOF_0);

    N += this->m_vDoF[i]->NumDim();
  }

  m_vDMDtv = VectorXd::Zero(0);
  m_mMass = MatrixXd::Zero(N, N);
  m_vgradient = VectorXd::Zero(N);
}

MassElement_Lumped::~MassElement_Lumped(void) {}

void MassElement_Lumped::Init() {
  // Nothing to do here...
}

void MassElement_Lumped::ComputeAndStore_Mass() {
  Real density = m_pParams->GetParameter(ParameterSet::Param_Density);

  int numDOF = this->GetSupportSize();
  double mass = density * m_pPoly->VolumeBasis(Tag::Position_0);
  this->m_mMass =
      MatrixXd::Identity(numDOF, numDOF) * mass / m_pPoly->NumNodes();
}

void MassElement_Lumped::ComputeAndStore_Energy_Internal() {
  MatrixXd mX;
  m_pPoly->GetNodesTrait(mX, Tag::Position_X);
  mX.transposeInPlace();
  mX.resize(mX.size(), 1);

  int numDOF = this->GetSupportSize();
  VectorXd vg = VectorXd::Zero(numDOF);
  for (int i = 0; i < (int)m_vDoF.size(); ++i)
    vg.segment(3 * i, 3) = -this->m_mMass(i, i) * this->m_vgravity;

  this->m_energy = VectorXd(mX).dot(vg);
}

void MassElement_Lumped::ComputeAndStore_Gradient_Internal() {
  int numDOF = this->GetSupportSize();
  m_vgradient = VectorXd::Zero(numDOF);
  for (int i = 0; i < (int)m_vDoF.size(); ++i)
    this->m_vgradient.segment(3 * i, 3) =
        -this->m_mMass(i, i) * this->m_vgravity;
}

}  // namespace PhySim