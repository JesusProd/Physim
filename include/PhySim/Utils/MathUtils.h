//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, FRL
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

namespace MathUtils {

void crossMatrix(const Vector3d& v, Matrix3d& M);
Matrix3d crossMatrix(const Vector3d& v);
int factorial(int n);
Real binomialCoefficient(int n, int i);
void computeBernsteinCoefficients(int n, Real x, VectorXd& vc);

Real degreesToRadians(Real degrees);
Real radiansToDegrees(Real radians);

void gaussQuadrature(int numPoints,
                     Real a,
                     Real b,
                     vector<VectorXd>& vpoints,
                     dVector& vweights);

template <typename T>
int sign(const T& x) {
  return x > 0 ? 1 : x < 0 ? -1 : 0;
}
template <typename T>
int looseSignP(const T& x) {
  return x >= 0 ? 1 : -1;
}
template <typename T>
int looseSignN(const T& x) {
  return x > 0 ? 1 : -1;
}

template <typename T>
T saturate(const T& x) {
  return x > 1 ? (T)1 : x < 0 ? (T)0 : x;
}

void computeCovarianceMatrix(const MatrixXd& mD, MatrixXd& mV);

void computePCA(const MatrixXd& mD, MatrixXd& mU, MatrixXd& mV, VectorXd& vs);

Real computePolarDecomposition(const Matrix3d& M,
                               Matrix3d& Q,
                               Matrix3d& S,
                               Real tol);

// Compute the cofactor matrix of the input 3x3 matrix
template <typename T>
void CofactorMatrix(const Matrix<T, 3, 3>& mIn, Matrix<T, 3, 3>& mOut) {
  mOut(0, 0) = mIn(1, 1) * mIn(2, 2) - mIn(1, 2) * mIn(2, 1);
  mOut(1, 0) = mIn(0, 2) * mIn(2, 1) - mIn(0, 1) * mIn(2, 2);
  mOut(2, 0) = mIn(0, 1) * mIn(1, 2) - mIn(0, 2) * mIn(1, 1);
  mOut(0, 1) = mIn(1, 2) * mIn(2, 0) - mIn(1, 0) * mIn(2, 2);
  mOut(1, 1) = mIn(0, 0) * mIn(2, 2) - mIn(0, 2) * mIn(2, 0);
  mOut(2, 1) = mIn(0, 2) * mIn(1, 0) - mIn(0, 0) * mIn(1, 2);
  mOut(0, 2) = mIn(1, 0) * mIn(2, 1) - mIn(1, 1) * mIn(2, 0);
  mOut(1, 2) = mIn(0, 1) * mIn(2, 0) - mIn(0, 0) * mIn(2, 1);
  mOut(2, 2) = mIn(0, 0) * mIn(1, 1) - mIn(0, 1) * mIn(1, 0);
}
template <typename T>
Matrix<T, 3, 3> CofactorMatrix(const Matrix<T, 3, 3>& mIn) {
  Matrix<T, 3, 3> mOut;
  CofactorMatrix(mIn, mOut);
  return mOut;
}

template <typename T, int N0, int N1, int N2>
using Tensor3 = Matrix<T, N1, N2>[N0];

template <int N0, int N1, int N2>
using Tensor3r = Tensor3<Real, N0, N1, N2>;

template <typename T, int N0, int N1, int N2, int N3>
using Tensor4 = Matrix<T, N2, N3>[N0][N1];

template <int N0, int N1, int N2, int N3>
using Tensor4r = Tensor4<Real, N0, N1, N2, N3>;

// Compute the vectorized representation of a matrix following the T.Kim 2020
// Siggraph Course convention
template <typename T, int N0, int N1>
void Vectorize(const Matrix<T, N0, N1>& inMat,
               Vector<T, -1>& outVec,
               bool colmajor = true) {
  outVec.resize(N0 * N1);
  int count = 0;
  if (colmajor) {
    for (int j = 0; j < N1; ++j)
      for (int i = 0; i < N0; ++i)
        outVec(count++) = inMat(i, j);
  } else {
    for (int i = 0; i < N0; ++i)
      for (int j = 0; j < N1; ++j)
        outVec(count++) = inMat(i, j);
  }
}

// Compute the matrix representation of a vector following the T.Kim 2020
// Siggraph Course convention
template <typename T, int N0, int N1>
void Unvectorize(const Vector<T, -1>& inVec,
                 Matrix<T, N0, N1>& outMat,
                 bool colmajor = true) {
  assert(inVec.size() == N0 * N1);
  int count = 0;
  if (colmajor) {
    for (int j = 0; j < N1; ++j)
      for (int i = 0; i < N0; ++i)
        outMat(i, j) = inVec(count++);
  } else {
    for (int i = 0; i < N0; ++i)
      for (int j = 0; j < N1; ++j)
        outMat(i, j) = inVec(count++);
  }
}

// Compute the matrix representation of a third-order tensor following the T.Kim
// 2020 Siggraph Course convention
template <typename T, int N0, int N1, int N2>
void Vectorize(const Tensor3<T, N0, N1, N2>& inTen,
               MatrixX<T>& outMat,
               bool colmajor = true) {
  if (colmajor) {
    outMat.resize(N1 * N2, N0);
    for (int i = 0; i < N0; ++i) {
      Vector<T, -1> col;
      Vectorize<T, N1, N2>(inTen[i], col, colmajor);
      outMat.col(i) = col;
    }
  } else {
    outMat.resize(N0, N1 * N2);
    for (int i = 0; i < N0; ++i) {
      Vector<T, -1> row;
      Vectorize<T, N1, N2>(inTen[i], row, colmajor);
      outMat.row(i) = row;
    }
  }
}

// Compute the third-order representation of a matrix following the T.Kim 2020
// Siggraph Course convention
template <typename T, int N0, int N1, int N2>
void Unvectorize(const MatrixX<T>& inMat,
                 Tensor3<T, N0, N1, N2>& outTen,
                 bool colmajor = true) {
  if (colmajor) {
    assert(inMat.rows() == constexpr(N1 * N2));
    assert(inMat.cols() == constexpr(N0));

    for (int i = 0; i < N0; ++i)
      Unvectorize<T, N1, N2>(inMat.col(i), outTen[i], colmajor);
  } else {
    assert(inMat.cols() == constexpr(N1 * N2));
    assert(inMat.rows() == constexpr(N0));

    for (int i = 0; i < N0; ++i)
      Unvectorize<T, N1, N2>(inMat.row(i), outTen[i], colmajor);
  }
}

// Compute the matrix representation of a fourth-order tensor following the
// T.Kim 2020 Siggraph Course convention
template <typename T, int N0, int N1, int N2, int N3>
void Vectorize(const Tensor4<T, N0, N1, N2, N3>& inTen,
               MatrixX<T>& outMat,
               bool colmajor = true) {
  int count = 0;
  if (colmajor) {
    outMat.resize(N2 * N3, N0 * N1);
    for (int j = 0; j < N1; ++j)
      for (int i = 0; i < N0; ++i) {
        VectorXr col;
        Vectorize<T, N2, N3>(inTen[i][j], col, colmajor);
        outMat.col(count++) = col;
      }
  } else {
    outMat.resize(N0 * N1, N2 * N3);
    for (int i = 0; i < N0; ++i)
      for (int j = 0; j < N1; ++j) {
        VectorXr row;
        Vectorize<T, N2, N3>(inTen[i][j], row, colmajor);
        outMat.row(count++) = row;
      }
  }
}

// Compute the fourth-order tensor representation of a matrix following the
// T.Kim 2020 Siggraph Course convention
template <typename T, int N0, int N1, int N2, int N3>
void Unvectorize(const MatrixX<T>& inMat,
                 Tensor4<T, N0, N1, N2, N3>& outTen,
                 bool colmajor = true) {
  int count = 0;
  if (colmajor) {
    assert(inMat.rows() == constexpr(N2 * N3));
    assert(inMat.cols() == constexpr(N0 * N1));
    for (int j = 0; j < N1; ++j)
      for (int i = 0; i < N0; ++i) {
        Unvectorize<T, N2, N3>(inMat.col(count++).eval(), outTen[i][j],
                               colmajor);
      }
  } else {
    assert(inMat.rows() == constexpr(N0 * N1));
    assert(inMat.cols() == constexpr(N2 * N3));
    for (int i = 0; i < N0; ++i)
      for (int j = 0; j < N1; ++j) {
        Unvectorize<T, N2, N3>(inMat.row(count++).eval(), outTen[i][j],
                               colmajor);
      }
  }
}

}  // namespace MathUtils
}  // namespace PhySim