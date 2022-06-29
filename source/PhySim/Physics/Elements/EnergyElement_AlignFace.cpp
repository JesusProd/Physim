//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_AlignFace.h>

#include <PhySim/Geometry/Polytopes/Face_Tri.h>
#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Physics/Simulables/Simulable.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

EnergyElement_AlignFace::EnergyElement_AlignFace(Simulable* pModel,
                                                 Face_Tri* pFace)
    : EnergyElement(pModel) {
  this->m_vDoF.resize(3);
  for (int i = 0; i < 3; ++i)
    this->m_vDoF[i] = pFace->Nodes()[i]->Traits().Kinematics(Tag::Tag_DOF_0);

  this->m_vgradient.resize(9);
  this->m_mHessian.resize(9, 9);

  this->m_pFace = pFace;
}

EnergyElement_AlignFace::~EnergyElement_AlignFace() {
  // Nothing to do here...
}

void EnergyElement_AlignFace::Init() {
  // Set target normal

  this->m_vnt = this->m_pFace->Normal(Tag::Tag_Position_X);

  // Set align stiffness

  this->m_kA = 1e9;
}

void EnergyElement_AlignFace::ComputeAndStore_Energy_Internal() {
  this->m_energy = 0;

  MatrixXd mX;
  this->m_pFace->GetNodesTrait(mX, Tag::Tag_Position_X);
  VectorXd x0 = mX.row(0);
  VectorXd x1 = mX.row(1);
  VectorXd x2 = mX.row(2);

  Vector3d nt = this->m_vnt;

  Real kA = this->m_kA;

  {
#include "../include/PhySim/Utils/Auto/NormalAlignment_Energy.mcg";

    this->m_energy = t33;
  }
}

void EnergyElement_AlignFace::ComputeAndStore_Gradient_Internal() {
  MatrixXd mX;
  this->m_pFace->GetNodesTrait(mX, Tag::Tag_Position_X);
  VectorXd x0 = mX.row(0);
  VectorXd x1 = mX.row(1);
  VectorXd x2 = mX.row(2);

  Vector3d nt = this->m_vnt;

  Real kA = this->m_kA;

  {
    dVector vgx(9);

#include "../include/PhySim/Utils/Auto/NormalAlignment_Gradient.mcg";

    this->m_vgradient.resize(9);
    for (int i = 0; i < 9; ++i)
      this->m_vgradient[i] = vgx[i];
  }
}

void EnergyElement_AlignFace::ComputeAndStore_Hessian_Internal() {
  MatrixXd mX;
  this->m_pFace->GetNodesTrait(mX, Tag::Tag_Position_X);
  VectorXd x0 = mX.row(0);
  VectorXd x1 = mX.row(1);
  VectorXd x2 = mX.row(2);

  Vector3d nt = this->m_vnt;

  Real kA = this->m_kA;

  {
    Real mHx[9][9];

#include "../include/PhySim/Utils/Auto/NormalAlignment_Hessian.mcg";

    this->m_mHessian.resize(9, 9);
    for (int i = 0; i < 9; ++i)
      for (int j = 0; j < 9; ++j)
        this->m_mHessian(i, j) = mHx[i][j];
  }
}

}  // namespace PhySim