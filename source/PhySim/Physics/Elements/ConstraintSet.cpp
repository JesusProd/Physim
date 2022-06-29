//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/ConstraintSet.h>

#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Utils/MatrixUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

ConstraintSet::ConstraintSet(Simulable* pModel, bool soft) {
  this->m_pModel = pModel;

  m_vvalues = VectorXd::Zero(0);
  m_mJacobian = MatrixXd::Zero(0, 0);
  m_vHessian.resize(0);

  this->m_vDoFs.clear();

  this->m_type = Type::EQ;
  this->m_size = -1;
  this->m_index = -1;
  this->m_offset = -1;
  this->m_soft = soft;
}

ConstraintSet::~ConstraintSet() {
  // Nothing to do...
}

int ConstraintSet::GetSupportSize() const {
  int N = 0;
  for (int i = 0; i < m_vDoFs.size(); ++i)
    N += m_vDoFs[i]->NumDim();
  return N;
}

bool ConstraintSet::IsActive() {
  this->ComputeAndStore_Constraint();

  for (int i = 0; i < m_size; ++i) {
    switch (this->m_type) {
      case Type::EQ:
        if (this->m_vvalues[i] != 0)
          return true;
      case Type::LE:
        if (this->m_vvalues[i] >= 0)
          return true;
      case Type::LEQ:
        if (this->m_vvalues[i] > 0)
          return true;
    }
  }

  return false;
}

void ConstraintSet::AssembleGlobal_Values(VectorXd& vtotalVector) {
  if (!this->IsActive() || this->m_vvalues.size() == 0)
    return;  // Ignore assembly

  vtotalVector.block(this->m_offset, 0, this->m_size, 1) += m_vvalues;
}

void ConstraintSet::AssembleGlobal_Jacobian(VectorTd& vtotalTriplets) {
  if (!this->IsActive() || this->m_mJacobian.size() == 0)
    return;  // Ignore assembly

  int localOffset_j = 0;

  size_t numDoFSet = this->m_vDoFs.size();
  for (size_t j = 0; j < numDoFSet; ++j) {
    assembleSparseMatrix(vtotalTriplets, this->m_offset,
                         this->m_vDoFs[j]->Offset(),
                         this->m_mJacobian.block(0, localOffset_j, this->m_size,
                                                 this->m_vDoFs[j]->NumDim()));

    localOffset_j += this->m_vDoFs[j]->NumDim();
  }
}
}  // namespace PhySim