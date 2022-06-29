//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_Force.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Physics/Simulables/Simulable.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

EnergyElement_Force::EnergyElement_Force(Simulable* pModel,
                                         const vector<KinematicsEle*>& vDoF)
    : EnergyElement(pModel) {
  int countDoF = 0;
  this->m_vDoF = vDoF;
  this->m_vforces.resize(vDoF.size());
  for (size_t i = 0; i < vDoF.size(); ++i) {
    int numD = m_vDoF[i]->NumDim();
    this->m_vforces[i].resize(numD);
    this->m_vforces[i].setZero();
    countDoF = countDoF + numD;
  }

  this->m_vgradient.resize(countDoF);
}

EnergyElement_Force::~EnergyElement_Force() {
  // Nothing to do here...
}

void EnergyElement_Force::Init() {
  // Nothing to do here...
}

void EnergyElement_Force::ComputeAndStore_Energy_Internal() {
  this->m_energy = 0;
  for (size_t i = 0; i < m_vDoF.size(); ++i)
    this->m_energy += this->m_vDoF[i]->GetValue().dot(this->m_vforces[i]);
}

void EnergyElement_Force::ComputeAndStore_Gradient_Internal() {
  int dofOffset = 0;
  for (size_t i = 0; i < m_vDoF.size(); ++i) {
    m_vgradient.block(dofOffset, 0, m_vDoF[i]->NumDim(), 1) =
        this->m_vforces[i];
    dofOffset += m_vDoF[i]->NumDim();
  }
}
}  // namespace PhySim