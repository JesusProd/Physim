//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Utils/MatrixUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

void eigenTripletsToSparseMatrix(const VectorTd& triplets, MatrixSd& M) {
  M.setFromTriplets(triplets.begin(), triplets.end());
}

void eigenSparseMatrixToPointers(const MatrixSd& M, VectorTp& triplets) {
  auto NZ = M.nonZeros();
  if (triplets.capacity() < NZ)
    triplets.reserve(NZ);
  triplets.clear();

  for (int k = 0; k < M.outerSize(); ++k)
    for (MatrixSd::InnerIterator it(M, k); it; ++it)
      triplets.push_back(Triplet<Real*>(it.row(), it.col(), &(it.valueRef())));
}

void eigenSparseMatrixToTriplets(const MatrixSd& M, VectorTd& triplets) {
  auto NZ = M.nonZeros();
  if (triplets.capacity() < NZ)
    triplets.reserve(NZ);
  triplets.clear();

  for (int k = 0; k < M.outerSize(); ++k)
    for (MatrixSd::InnerIterator it(M, k); it; ++it)
      triplets.push_back(Triplet<Real>(it.row(), it.col(), it.valueRef()));
}

void buildSelectionMatrix(const VectorXi& vselected,
                          int numIndex,
                          MatrixXd& mOut) {
  int numS = vselected.size();
  mOut.resize(numS, numIndex);
  mOut.setZero();
  for (int i = 0; i < numS; ++i)
    mOut(i, vselected(i)) = 1;
}

void buildBlockExpandedMatrix(const MatrixXd& mInput,
                              int expRate,
                              MatrixXd& mOut) {
  int Nin = mInput.rows();
  int Min = mInput.cols();
  mOut.resize(Nin * expRate, Min * expRate);
  mOut.setZero();
  for (int i = 0; i < Nin; ++i)
    for (int j = 0; j < Min; ++j)
      for (int k = 0; k < expRate; ++k)
        mOut(expRate * i + k, expRate * j + k) = mInput(i, j);
}

void buildBlockExpandedMatrix(const MatrixSd& mInput,
                              int expRate,
                              MatrixSd& mOut) {
  VectorTd vtripletsIn;
  VectorTd vtripletsOut;
  eigenSparseMatrixToTriplets(mInput, vtripletsIn);

  vtripletsOut.reserve(3 * vtripletsIn.size());
  for (size_t i = 0; i < vtripletsIn.size(); ++i) {
    int rowOffset = expRate * vtripletsIn[i].row();
    int colOffset = expRate * vtripletsIn[i].col();
    for (int j = 0; j < expRate; ++j) {
      vtripletsOut.push_back(
          Triplet<Real>(rowOffset + j, colOffset + j, vtripletsIn[i].value()));
    }
  }

  mOut.resize(mInput.rows() * expRate, mInput.cols() * expRate);
  mOut.setFromTriplets(vtripletsOut.begin(), vtripletsOut.end());
  mOut.makeCompressed();
}

void buildSelectionMatrix(const iVector& vselected,
                          int numIndex,
                          MatrixSd& mOut) {
  VectorTd vtripletsOut((int)vselected.size());
  for (auto i = 0; i < (int)vselected.size(); ++i)
    vtripletsOut[i] = Triplet<Real>(i, vselected[i], 1);

  mOut.resize(vselected.size(), numIndex);
  mOut.setFromTriplets(vtripletsOut.begin(), vtripletsOut.end());
  mOut.makeCompressed();
}

void buildDimensionRemovalMatrix(int numVert,
                                 int numDim,
                                 int remDim,
                                 MatrixXd& mOut) {
  mOut = MatrixXd::Zero(numVert * (numDim - 1), numVert * numDim);

  int offset = 0;
  for (int i = 0; i < numVert; ++i)
    for (int j = 0; j < numDim; ++j)
      if (j != remDim)
        mOut(offset++, i * numDim + j) = 1.0;
}

void assembleSparseMatrix(VectorTd& triplets,
                          unsigned int row,
                          unsigned int col,
                          const MatrixXd& M) {
  for (int i = 0; i < M.rows(); ++i)
    for (int j = 0; j < M.cols(); ++j) {
      triplets.push_back(Triplet<double>(row + i, col + j, M(i, j)));
    }
}

void getDiagonalIndices(const VectorTd& vT,
                        int rows,
                        int cols,
                        iVector& vDIdx) {
  vDIdx.reserve(rows);

  for (size_t i = 0; i < vT.size(); ++i)
    if (vT[i].row() == vT[i].col())
      vDIdx.push_back((int)i);
}

}  // namespace PhySim
