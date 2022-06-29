//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>

#include <PhySim/Physics/Elements/ConstraintSet.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Embedding;
class Simulable;

class ConstraintSet_NodeNodeColl : public ConstraintSet {
 protected:
  Real m_tol;
  Embedding* m_pEmbedA;
  Embedding* m_pEmbedB;
  VectorXd m_vParamsA;
  VectorXd m_vParamsB;
  size_t m_NumNodesA;
  size_t m_NumNodesB;
  size_t m_NumDoFsA;
  size_t m_NumDoFsB;
  bool m_isTwoWay;

 public:
  ConstraintSet_NodeNodeColl(Simulable* pModel,
                             bool isSoft,
                             Embedding* pEmbedA,
                             Embedding* pEmbedB,
                             Real tol,
                             bool twoWay = true);

 public:
  void Init() override;

  void ProjectConstraint() override;
  void ComputeAndStore_Constraint() override;
  void ComputeAndStore_Jacobian() override;
  void ComputeAndStore_Hessian() override;

  bool IsTwoWay() const { return this->m_isTwoWay; }
};
}  // namespace PhySim