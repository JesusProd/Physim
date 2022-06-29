
//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Christos Koutras, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_SpringBetwNodes.h>

#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Physics/Simulables/Simulable.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

EnergyElement_SpringBetwNodes::EnergyElement_SpringBetwNodes(
    Simulable* pModel,
    Node* node1,
    Node* node2,
    PtrS<ParameterSet> pMaterial)
    : EnergyElement(pModel) {
  this->m_node1 = node1;
  this->m_node2 = node2;

  this->m_vDoF.resize(2);
  this->m_vDoF[0] = node1->Traits().Kinematics(Tag::Tag_DOF_0);
  this->m_vDoF[1] = node2->Traits().Kinematics(Tag::Tag_DOF_0);

  this->m_vgradient.resize(6);
  this->m_mHessian.resize(6, 6);

  this->m_pMaterial = pMaterial;
  // this->m_restLength = 0.0;
}

EnergyElement_SpringBetwNodes::~EnergyElement_SpringBetwNodes() {
  // Nothing to do...
}

void EnergyElement_SpringBetwNodes::Init() {
  this->m_intVolume = this->m_restLength = this->ComputeRestLength();
}

Real EnergyElement_SpringBetwNodes::ComputeRestLength() {
  // return this->m_pEdge->VolumeBasis(Tag_Position_0);
  Vector3d n1 = m_node1->Traits().Vector3d(Tag_Position_0);
  Vector3d n2 = m_node2->Traits().Vector3d(Tag_Position_0);

  return (n2 - n1).norm();
}

void EnergyElement_SpringBetwNodes::ComputeAndStore_Energy_Internal() {
  // Get defo nodes

  Vector3d ax = this->m_vDoF[0]->GetValue();
  Vector3d bx = this->m_vDoF[1]->GetValue();

  // Strain

  // double s = ((bx - ax).norm() / this->m_restLength - 1);
  Vector3d s = bx - ax;

  // Stiffness

  double k = (*this->m_pMaterial)[ParameterSet::Param_StretchK];
  Matrix3d k_anisotropic;
  k_anisotropic.setZero();
  k_anisotropic(0, 0) = k;
  k_anisotropic(1, 1) = k;
  k_anisotropic(2, 2) = k;

  // this->m_energy = 0.5*k*this->m_restLength*s*s;
  this->m_energy = 0.5 * s.transpose() * k_anisotropic * s;
}
void EnergyElement_SpringBetwNodes::ComputeAndStore_Gradient_Internal() {
  // Get defo nodes

  Vector3d ax = this->m_vDoF[0]->GetValue();
  Vector3d bx = this->m_vDoF[1]->GetValue();

  // Strain

  Vector3d s = bx - ax;

  // Stiffness

  double k = (*this->m_pMaterial)[ParameterSet::Param_StretchK];
  Matrix3d k_anisotropic;
  k_anisotropic.setZero();
  k_anisotropic(0, 0) = k;
  k_anisotropic(1, 1) = k;
  k_anisotropic(2, 2) = k;

  // GradientFull

  this->m_vgradient.segment(0, 3) =
      Vector3d(-s[0] * k_anisotropic(0, 0), -s[1] * k_anisotropic(1, 1),
               -s[2] * k_anisotropic(2, 2));
  this->m_vgradient.segment(3, 3) =
      Vector3d(s[0] * k_anisotropic(0, 0), s[1] * k_anisotropic(1, 1),
               s[2] * k_anisotropic(2, 2));
}

void EnergyElement_SpringBetwNodes::ComputeAndStore_Hessian_Internal() {
  // Stiffness

  double k = (*this->m_pMaterial)[ParameterSet::Param_StretchK];
  Matrix3d k_anisotropic;
  k_anisotropic.setZero();
  k_anisotropic(0, 0) = k;
  k_anisotropic(1, 1) = k;
  k_anisotropic(2, 2) = k;

  this->m_mHessian.block<3, 3>(0, 0) = k_anisotropic;
  this->m_mHessian.block<3, 3>(3, 3) = k_anisotropic;
  this->m_mHessian.block<3, 3>(0, 3) = -k_anisotropic;
  this->m_mHessian.block<3, 3>(3, 0) = -k_anisotropic;
}
}  // namespace PhySim