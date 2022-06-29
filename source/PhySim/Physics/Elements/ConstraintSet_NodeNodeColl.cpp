//==========================================================
//
//    PhySim library. Generic library for physical simulation.
//
//    Authors:
//            Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/ConstraintSet_NodeNodeColl.h>

#include <PhySim/Geometry/Polytopes/Embedding.h>
#include <PhySim/Geometry/Polytopes/Poly.h>
#include <PhySim/Geometry/Polytopes/ShapeFunctions.h>

#include <PhySim/Kinematics/KinematicsEle.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

ConstraintSet_NodeNodeColl::ConstraintSet_NodeNodeColl(Simulable* pModel,
                                                       bool isSoft,
                                                       Embedding* pEmbedA,
                                                       Embedding* pEmbedB,
                                                       Real tol,
                                                       bool isTwoWay)
    : ConstraintSet(pModel, isSoft),
      m_tol(tol),
      m_pEmbedA(pEmbedA),
      m_pEmbedB(pEmbedB) {
  const Poly* pMasterA = pEmbedA->Master();
  const Poly* pMasterB = pEmbedB->Master();
  assert(pMasterA->DimSpace() == 3 && pMasterB->DimSpace() == 3 &&
         "Only 3-dimensional spaces are supported.");

  PtrS<ShapeFunction_LERP> pShapeFuncA =
      dynamic_pointer_cast<ShapeFunction_LERP>(pMasterA->GetShapeFunction());
  PtrS<ShapeFunction_LERP> pShapeFuncB =
      dynamic_pointer_cast<ShapeFunction_LERP>(pMasterB->GetShapeFunction());
  assert(pShapeFuncA && pShapeFuncB &&
         "Only LERP-based shape functions are supported.");
  pShapeFuncA->Compute_Values(pEmbedA->Parameter(), m_vParamsA);
  pShapeFuncB->Compute_Values(pEmbedB->Parameter(), m_vParamsB);

  this->m_NumNodesA = pMasterA->GetShapeFunction()->NumPoints();
  this->m_NumNodesB = pMasterB->GetShapeFunction()->NumPoints();
  this->m_NumDoFsA = 3 * this->m_NumNodesA;
  this->m_NumDoFsB = 3 * this->m_NumNodesB;

  this->m_isTwoWay = isTwoWay;

  if (this->m_isTwoWay) {
    this->m_vDoFs.reserve(m_NumNodesA + m_NumNodesB);
    for (auto pNode : pMasterA->Nodes())
      this->m_vDoFs.push_back(pNode->Traits().Kinematics(Tag::DOF_0));
    for (auto pNode : pMasterB->Nodes())
      this->m_vDoFs.push_back(pNode->Traits().Kinematics(Tag::DOF_0));

    this->m_size = 1;
    this->m_vvalues.resize(1);
    this->m_mJacobian.resize(1, this->m_NumDoFsA + this->m_NumDoFsB);
    this->m_vHessian.resize(1);
    this->m_vHessian[0].resize(this->m_NumDoFsA + this->m_NumDoFsB,
                               this->m_NumDoFsA + this->m_NumDoFsB);
  } else {
    this->m_vDoFs.reserve(m_NumNodesA);
    for (auto pNode : pMasterA->Nodes())
      this->m_vDoFs.push_back(pNode->Traits().Kinematics(Tag::DOF_0));

    this->m_size = 1;
    this->m_vvalues.resize(1);
    this->m_mJacobian.resize(1, this->m_NumDoFsA);
    this->m_vHessian.resize(1);
    this->m_vHessian[0].resize(this->m_NumDoFsA, this->m_NumDoFsA);
  }

  this->m_type = Type::LEQ;
}

void ConstraintSet_NodeNodeColl::Init() {
  // Nothing to do here...
}

void ConstraintSet_NodeNodeColl::ProjectConstraint() {
  /*
  // Consider if nodes are fixed for the constraint projection.
  const Vector3d& vxA = this->m_vDoFs[0]->GetValue();
  const Vector3d& vxB = this->m_vDoFs[1]->GetValue();
  const Real wA = Real(this->m_vDoFs[0]->Active() &&
  !this->m_vDoFs[0]->GetFixed()); const Real wB =
  Real(this->m_vDoFs[1]->Active() && !this->m_vDoFs[1]->GetFixed()); if ((wA +
  wB) == 0.0) return;

  const Vector3d vD = vxA - vxB;
  const Real L = vD.norm();

  const Vector3d vdxA = -(wA / (wA + wB)) * (L - m_tol) * (vD / L);
  const Vector3d vdxB =  (wB / (wA + wB)) * (L - m_tol) * (vD / L);

  this->m_vDoFs[0]->SetValue(vxA + vdxA);
  this->m_vDoFs[1]->SetValue(vxB + vdxB);
  */

  // Too much of a hassle to do it considering the isoparametric coordinates.
}

void ConstraintSet_NodeNodeColl::ComputeAndStore_Constraint() {
  // D = xa - xb
  // L = |D|
  // C = tol - L

  Vector3d vxA = m_pEmbedA->InterpolateValue(Tag::Position_X);
  Vector3d vxB = m_pEmbedB->InterpolateValue(Tag::Position_X);

  const Vector3d vD = vxA - vxB;
  const Real L = vD.norm();

  this->m_vvalues[0] = m_tol - L;

  /*
  for (size_t i = 0; i < m_vDoFs.size(); ++i)
  {
      Vector3d x = m_vDoFs[i]->Geometry()->Traits().Vector3d(Tag::Position_X);
      cout << i << " = " << x.transpose() << endl;
  }

  cout << "pA = " << m_vParamsA << endl;
  cout << "pB = " << m_vParamsB << endl;
  */
}

void ConstraintSet_NodeNodeColl::ComputeAndStore_Jacobian() {
  //       D = xa - xb
  //       L = |D|
  //   dC/da = -dL/da
  //   dL/da = (D^T / L) * dD/da

  // Compute Jacobian w.r.t xa and xb.
  Vector3d vxA = m_pEmbedA->InterpolateValue(Tag::Position_X);
  Vector3d vxB = m_pEmbedB->InterpolateValue(Tag::Position_X);

  const Vector3d vD = vxA - vxB;
  const Real L = vD.norm();

  const Vector3d dCdxA = vD / L;

  // Compute Jacobian w.r.t. DoFs.
  auto JA = this->m_mJacobian.block(0, 0, 1, m_NumDoFsA);
  for (size_t i = 0; i < m_NumNodesA; ++i)
    JA.block(0, i * 3, 1, 3) = -(m_vParamsA[i] * dCdxA).transpose();

  if (this->m_isTwoWay) {
    auto JB = this->m_mJacobian.block(0, m_NumDoFsA, 1, m_NumDoFsB);
    for (size_t i = 0; i < m_NumNodesB; ++i)
      JB.block(0, i * 3, 1, 3) = (m_vParamsB[i] * dCdxA).transpose();
  }
}

void ConstraintSet_NodeNodeColl::ComputeAndStore_Hessian() {
  //        D = xa - xb
  //        L = |D|
  //    dC/da = -dL/da
  //    dL/da = (D^T / L) * dD/da
  // d2C/dadb = -d2L/dadb
  // d2L/dadb = (1 / L) * [(dD/db)^T * dD/da + D^T * d2D/dadb - dL/da * dL/db^T]

  // Compute Hessian w.r.t xa and xb.
  Vector3d vxA = m_pEmbedA->InterpolateValue(Tag::Position_X);
  Vector3d vxB = m_pEmbedB->InterpolateValue(Tag::Position_X);
  const Vector3d vD = vxA - vxB;
  const Real L = vD.norm();

  const Vector3d dLdxA = vD / L;
  const Matrix3d d2LdxAxA =
      (1 / L) * (Matrix3d::Identity() - dLdxA * dLdxA.transpose());

  // Compute Hessian w.r.t. DoFs.
  auto HAA = this->m_vHessian[0].block(0, 0, m_NumDoFsA, m_NumDoFsA);
  for (size_t i = 0; i < m_NumNodesA; ++i)
    for (size_t j = 0; j < m_NumNodesA; ++j) {
      HAA.block(i * 3, j * 3, 3, 3) =
          -(m_vParamsA[i] * m_vParamsA[j]) * d2LdxAxA;
    }

  if (this->m_isTwoWay) {
    auto HAB = this->m_vHessian[0].block(0, m_NumDoFsA, m_NumDoFsA, m_NumDoFsB);
    auto HBA = this->m_vHessian[0].block(m_NumDoFsA, 0, m_NumDoFsB, m_NumDoFsA);
    auto HBB = this->m_vHessian[0].block(m_NumDoFsA, m_NumDoFsA, m_NumDoFsB,
                                         m_NumDoFsB);

    for (size_t i = 0; i < m_NumNodesB; ++i)
      for (size_t j = 0; j < m_NumNodesB; ++j) {
        HBB.block(i * 3, j * 3, 3, 3) =
            -(m_vParamsB[i] * m_vParamsB[j]) * d2LdxAxA;
      }

    for (size_t i = 0; i < m_NumNodesA; ++i)
      for (size_t j = 0; j < m_NumNodesB; ++j) {
        HAB.block(i * 3, j * 3, 3, 3) =
            (m_vParamsA[i] * m_vParamsB[j]) * d2LdxAxA;
        HBA.block(j * 3, i * 3, 3, 3) =
            (m_vParamsA[j] * m_vParamsB[i]) * d2LdxAxA;
      }
  }
}
}  // namespace PhySim