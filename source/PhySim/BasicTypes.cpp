//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/BasicTypes.h>

#include <PhySim/Utils/MatrixUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

PtrS<AMatrixSd::Block> AMatrixSd::CreateBlock(int row, int col, int N, int M) {
  PtrS<Block> pBlock(new Block());

  if (row >= col) {
    pBlock->m_transpose = false;
    pBlock->m_row = row;
    pBlock->m_col = col;
    pBlock->m_N = N;
    pBlock->m_M = M;
  } else {
    pBlock->m_transpose = true;
    pBlock->m_row = col;
    pBlock->m_col = row;
    pBlock->m_N = M;
    pBlock->m_M = N;
  }

  pBlock->m_pMatrix = this;
  pBlock->m_mPointers.resize(pBlock->m_N, pBlock->m_M);
  for (int i = 0; i < pBlock->m_N; ++i)
    for (int j = 0; j < pBlock->m_M; ++j)
      pBlock->m_mPointers(i, j) =
          this->m_coeffMap[IntPair(pBlock->m_row + i, pBlock->m_col + j)];

  return pBlock;
}

bool AMatrixSd::AddStaticBlock(PtrS<AMatrixSd::Block> pBlock,
                               const MatrixXd& mBlock) {
  // The block is already created: assemble it!

  if (!pBlock->m_transpose) {
    assert(pBlock->m_N == mBlock.rows());
    assert(pBlock->m_M == mBlock.cols());

    pBlock->Add(mBlock);
  } else {
    assert(pBlock->m_N == mBlock.cols());
    assert(pBlock->m_M == mBlock.rows());

    pBlock->Add(mBlock.adjoint());
  }

  return true;
}

bool AMatrixSd::AddStaticBlock(int row, int col, const MatrixXd& mBlock) {
  if (row >= col) {
    for (int i = 0; i < mBlock.rows(); ++i)
      for (int j = 0; j < mBlock.cols(); ++j)
        this->m_vvalueTripletsStatic.push_back(
            Triplet<double>(row + i, col + j, mBlock(i, j)));
  } else {
    for (int i = 0; i < mBlock.rows(); ++i)
      for (int j = 0; j < mBlock.cols(); ++j)
        this->m_vvalueTripletsStatic.push_back(
            Triplet<double>(col + j, row + i, mBlock(i, j)));
  }

  return true;
}

bool AMatrixSd::AddDynamicBlock(int row, int col, const MatrixXd& mBlock) {
  // Regular assembly process

  if (row >= col) {
    for (int i = 0; i < mBlock.rows(); ++i)
      for (int j = 0; j < mBlock.cols(); ++j)
        this->m_vvalueTripletsDynamic.push_back(
            Triplet<double>(row + i, col + j, mBlock(i, j)));
  } else {
    for (int i = 0; i < mBlock.rows(); ++i)
      for (int j = 0; j < mBlock.cols(); ++j)
        this->m_vvalueTripletsDynamic.push_back(
            Triplet<double>(col + j, row + i, mBlock(i, j)));
  }

  return true;
}

void AMatrixSd::Clear() {
  this->Zero();

  m_coeffMap.clear();
  // m_vassemblyBlocks.clear();
  // this->m_assemblyCount = 0;
}

void AMatrixSd::Zero() {
  (*this) *= 0;
  this->m_vvalueTripletsStatic.clear();
  this->m_vvalueTripletsDynamic.clear();
}

void AMatrixSd::StartAssembly() {
  this->Zero();

  // this->m_assemblyCount = 0;
}

void AMatrixSd::EndAssembly() {
  // Am I using fast assembly?

  if (!this->IsFast()) {
    // Build static matrix from triplets

    this->setFromTriplets(this->m_vvalueTripletsStatic.begin(),
                          this->m_vvalueTripletsStatic.end());
    this->makeCompressed();

    if (this->m_isFast) {
      // Build the coefficients map for blocks

      m_coeffMap.clear();
      VectorTp vpointTriplets;

      eigenSparseMatrixToPointers((*this), vpointTriplets);
      size_t numCoeff = vpointTriplets.size();
      for (size_t i = 0; i < numCoeff; ++i) {
        const Triplet<Real*>& pointer = vpointTriplets[i];
        IntPair key = IntPair(pointer.row(), pointer.col());
        pair<IntPair, Real*> item(key, pointer.value());
        m_coeffMap.insert(item);
      }

      assert(m_coeffMap.size() == vpointTriplets.size());
    }
  }

  // Continue with regular assembly

  if (this->m_vvalueTripletsDynamic.size() != 0) {
    // Build dynamic matrix from triplets

    MatrixSd mdynamicPart(this->rows(), this->cols());
    mdynamicPart.setFromTriplets(this->m_vvalueTripletsDynamic.begin(),
                                 this->m_vvalueTripletsDynamic.end());
    mdynamicPart.makeCompressed();

    // Sum up dynamic part. This operation should be fast(ish) if
    // the sparsity pattern is the same. TODO: Run some tests and
    // consider summing the dynamic part by converting everything
    // to triplets and then rebuilding the matrix

    (*this) += mdynamicPart;
  }
}

}  // namespace PhySim