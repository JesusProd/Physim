//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, IST Austria
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_DER_Connection.h>

#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Physics/Simulables/Simulable_DER.h>

#include <PhySim/Utils/Auto/getRBConEnergyOPT.h>
#include <PhySim/Utils/Auto/getRBConGradientOPT.h>
#include <PhySim/Utils/Auto/getRBConHessianOPT.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

EnergyElement_DER_Connection::EnergyElement_DER_Connection(Simulable* pModel,
                                                           Node* pCenter,
                                                           Edge* pEdge)
    : EnergyElement(pModel) {
  this->m_pModelDER = static_cast<Simulable_DER*>(pModel);
  this->m_pCenter = pCenter;
  this->m_pEdge = pEdge;

  this->m_vDoF.resize(4);
  this->m_vDoF[0] = m_pEdge->GetTail()->Traits().Kinematics(Tag::DOF_0);
  this->m_vDoF[1] = m_pEdge->GetHead()->Traits().Kinematics(Tag::DOF_0);
  this->m_vDoF[2] = m_pCenter->Traits().Kinematics(Tag::DOF_1);
  this->m_vDoF[3] = m_pEdge->Traits().Kinematics(Tag::DOF_0);

  this->m_vgradient.resize(10);
  this->m_mHessian.resize(10, 10);

  this->m_mDeDx.resize(7, 10);
  this->m_mDeDx.setZero();
  this->m_mDeDx.block(0, 0, 3, 3) = -Matrix3d::Identity();
  this->m_mDeDx.block(0, 3, 3, 3) = Matrix3d::Identity();
  this->m_mDeDx.block(3, 6, 3, 3) = Matrix3d::Identity();
  this->m_mDeDx(6, 9) = 1;

  this->Init();
}

EnergyElement_DER_Connection::~EnergyElement_DER_Connection(void) {
  // Nothing to do here...
}

void EnergyElement_DER_Connection::Init() {
  this->m_intVolume = 0.5 * this->m_pEdge->VolumeBasis(Tag::Position_0);
}

void EnergyElement_DER_Connection::ComputeAndStore_Energy_Internal() {
  // Get data

  Vector3d ve = this->m_pEdge->Vector(Tag::Position_X);
  const Frame3d& Fc0 = this->m_pEdge->Traits().Frame3d(Tag::Frame_0);
  Frame3d Fc = Fc0 * m_pCenter->Traits().Matrix3d(Tag::Rotat_X);
  const Frame3d& Fr = this->m_pEdge->Traits().Frame3d(Tag::Frame_X);
  const Real& matTwist = this->m_pEdge->Traits().Double(Tag::Angle_X);
  const Real& refTwist = this->m_pEdge->Traits().Double(Tag::Twist_X);
  const Vector3d& veul = m_pCenter->Traits().Vector3d(Tag::Euler_X);

  // Compute

  ParameterSet& parameters =
      *this->m_pModelDER->MaterialDistribution()->GetValueAtDomainPoint(
          this->m_pEdge->ID());

  this->m_energy = getRBConEnergyOPT(
      parameters[ParameterSet::Param_Young],
      parameters[ParameterSet::Param_Lame2],
      this->m_pEdge->Traits().Double(Tag::Size_0),
      this->m_pEdge->Traits().Double(Tag::Size_1), this->m_intVolume,
      Fc.tan.data(), Fc.nor.data(), Fr.tan.data(), Fr.nor.data(), refTwist,
      veul.data(), ve.data(), matTwist);
}

void EnergyElement_DER_Connection::ComputeAndStore_Gradient_Internal() {
  // Get data

  Vector3d ve = this->m_pEdge->Vector(Tag::Position_X);
  const Frame3d& Fc0 = this->m_pEdge->Traits().Frame3d(Tag::Frame_0);
  Frame3d Fc = Fc0 * m_pCenter->Traits().Matrix3d(Tag::Rotat_X);
  const Frame3d& Fr = this->m_pEdge->Traits().Frame3d(Tag::Frame_X);
  const Real& matTwist = this->m_pEdge->Traits().Double(Tag::Angle_X);
  const Real& refTwist = this->m_pEdge->Traits().Double(Tag::Twist_X);
  const Vector3d& veul = m_pCenter->Traits().Vector3d(Tag::Euler_X);

  // Compute

  ParameterSet& parameters =
      *this->m_pModelDER->MaterialDistribution()->GetValueAtDomainPoint(
          this->m_pEdge->ID());

  VectorXd vg(7);

  getRBConGradientOPT(parameters[ParameterSet::Param_Young],
                      parameters[ParameterSet::Param_Lame2],
                      this->m_pEdge->Traits().Double(Tag::Size_0),
                      this->m_pEdge->Traits().Double(Tag::Size_1),
                      this->m_intVolume, Fc.tan.data(), Fc.nor.data(),
                      Fr.tan.data(), Fr.nor.data(), refTwist, veul.data(),
                      ve.data(), matTwist, vg.data());

  this->m_vgradient = m_mDeDx.transpose() * vg;
}

void EnergyElement_DER_Connection::ComputeAndStore_Hessian_Internal() {
  // Get data

  Vector3d ve = this->m_pEdge->Vector(Tag::Position_X);
  const Frame3d& Fc0 = this->m_pEdge->Traits().Frame3d(Tag::Frame_0);
  Frame3d Fc = Fc0 * m_pCenter->Traits().Matrix3d(Tag::Rotat_X);
  const Frame3d& Fr = this->m_pEdge->Traits().Frame3d(Tag::Frame_X);
  const Real& matTwist = this->m_pEdge->Traits().Double(Tag::Angle_X);
  const Real& refTwist = this->m_pEdge->Traits().Double(Tag::Twist_X);
  const Vector3d& veul = m_pCenter->Traits().Vector3d(Tag::Euler_X);

  // Compute

  ParameterSet& parameters =
      *this->m_pModelDER->MaterialDistribution()->GetValueAtDomainPoint(
          this->m_pEdge->ID());

  MatrixXd mH(7, 7);

  getRBConHessianOPT(parameters[ParameterSet::Param_Young],
                     parameters[ParameterSet::Param_Lame2],
                     this->m_pEdge->Traits().Double(Tag::Size_0),
                     this->m_pEdge->Traits().Double(Tag::Size_1),
                     this->m_intVolume, Fc.tan.data(), Fc.nor.data(),
                     Fr.tan.data(), Fr.nor.data(), refTwist, veul.data(),
                     ve.data(), matTwist, mH.transpose().data());

  this->m_mHessian = m_mDeDx.transpose() * mH * m_mDeDx;
}

}  // namespace PhySim