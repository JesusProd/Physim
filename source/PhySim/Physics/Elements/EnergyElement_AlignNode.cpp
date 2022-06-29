//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_AlignNode.h>

#include <PhySim/Geometry/Polytopes/Face_Tri.h>
#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Physics/Simulables/Simulable.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

EnergyElement_AlignNode::EnergyElement_AlignNode(Simulable* pModel, Node* pNode)
    : EnergyElement(pModel) {
  this->m_vDoF.resize(1);
  this->m_vDoF[0] = pNode->Traits().Kinematics(Tag::DOF_0);

  this->m_vgradient.resize(3);
  this->m_mHessian.resize(3, 3);

  this->m_pEmbedding = pNode;
}

EnergyElement_AlignNode::~EnergyElement_AlignNode() {
  // Nothing to do here...
}

void EnergyElement_AlignNode::Init() {
  // Set target normal

  this->m_vxt = this->m_pEmbedding->Traits().Vector3d(Tag::Position_X);

  // Set align stiffness

  this->m_kA = 1e9;
}

void EnergyElement_AlignNode::ComputeAndStore_Energy_Internal() {
  this->m_energy = 0;

  VectorXd x0 = this->m_pEmbedding->Traits().Vector3d(Tag::Position_X);

  Vector3d xt = this->m_vxt;

  Real kA = this->m_kA;

  {
#include "../include/PhySim/Utils/Auto/PositionAlignment_Energy.mcg";

    this->m_energy = t9;
  }
}

void EnergyElement_AlignNode::ComputeAndStore_Gradient_Internal() {
  this->m_energy = 0;

  VectorXd x0 = this->m_pEmbedding->Traits().Vector3d(Tag::Position_X);

  Vector3d xt = this->m_vxt;

  Real kA = this->m_kA;

  {
    dVector vgx(3);

#include "../include/PhySim/Utils/Auto/PositionAlignment_Gradient.mcg";

    this->m_vgradient.resize(3);
    for (int i = 0; i < 3; ++i)
      this->m_vgradient[i] = vgx[i];
  }
}

void EnergyElement_AlignNode::ComputeAndStore_Hessian_Internal() {
  this->m_energy = 0;

  VectorXd x0 = this->m_pEmbedding->Traits().Vector3d(Tag::Position_X);

  Vector3d xt = this->m_vxt;

  Real kA = this->m_kA;

  {
    Real mHx[3][3];

#include "../include/PhySim/Utils/Auto/PositionAlignment_Hessian.mcg";

    this->m_mHessian.resize(3, 3);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        this->m_mHessian(i, j) = mHx[i][j];
  }
}

}  // namespace PhySim