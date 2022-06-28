//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, FRL
//
//==========================================================

#pragma once

#include <PhySim/Utils/MathUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

namespace MathUtils {
void crossMatrix(const Vector3d& v, Matrix3d& mC) {
  mC << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
}

Matrix3d crossMatrix(const Vector3d& v) {
  Matrix3d mA;
  crossMatrix(v, mA);
  return mA;
}

int factorial(int n) {
  assert(n >= 0);
  if (n == 0)
    return 1;
  if (n == 1)
    return 1;

  int fact = n;
  for (int i = n - 1; i > 1; i--)
    fact *= i;

  return fact;
}

Real binomialCoefficient(int n, int i) {
  assert(i <= n);
  return (Real)factorial(n) / (Real)(factorial(i) * factorial(n - i));
}

void computeBernsteinCoefficients(int n, Real x, VectorXd& vc) {
  vc.resize(n + 1);
  for (int i = 0; i <= n; ++i)
    vc[i] = pow(x, i) * pow(1 - x, n - i) * binomialCoefficient(n, i);
}

Real degreesToRadians(Real degrees) {
  return M_PI * (degrees / 180);
}

Real radiansToDegrees(Real radians) {
  return (radians / M_PI) * 180;
}

void gaussQuadrature(int nbGauss,
                     double a,
                     double b,
                     vector<VectorXd>& vpoints,
                     dVector& vweights) {
  vpoints.resize(nbGauss * nbGauss * nbGauss);
  vweights.resize(nbGauss * nbGauss * nbGauss);
  int N(nbGauss - 1), N1(N + 1), N2(N + 2);
  double eps(numeric_limits<double>::epsilon());
  Eigen::ArrayXd xu(N1), y(N1), y0(N1), var(N + 1), temp(N1), Lp(N1);
  Eigen::ArrayXXd L(N1, N2), var2(nbGauss, 2);
  xu = Eigen::ArrayXd::LinSpaced(N1, a, b);
  var = Eigen::ArrayXd::LinSpaced(N + 1, 0.0, N);
  y0 = Eigen::ArrayXd::Ones(N1);
  y = cos((2.0 * var + 1.0) * M_PI / (2.0 * double(N) + 2.0)) +
      (0.27 / double(N1)) * sin(M_PI * xu * double(N) / double(N2));
  L = Eigen::ArrayXXd::Zero(N1, N2);
  Lp = Eigen::ArrayXd::Zero(N1);
  y0 = y0 * 2.0;
  temp = abs(y - y0);
  while (temp.maxCoeff() > eps) {
    L.col(0) = Eigen::ArrayXd::Ones(N1);
    Lp = Eigen::ArrayXd::Zero(N1);
    L.col(1) = y;
    for (int k = 2; k < N1 + 1; ++k) {
      L.col(k) = ((2.0 * double(k) - 1.0) * y * L.col(k - 1) -
                  (double(k) - 1.0) * L.col(k - 2)) /
                 double(k);
    }
    Lp = double(N2) * (L.col(N1 - 1) - y * L.col(N2 - 1)) / (1.0 - y * y);
    y0 = y;
    y = y0 - L.col(N2 - 1) / Lp;
    temp = abs(y - y0);  // while test
  }
  var2.col(0) = (double(a) * (1.0 - y) + double(b) * (1.0 + y)) / 2.0;  // 1D x
  var2.col(1) = (double(b) - double(a)) / ((1.0 - y * y) * (Lp * Lp)) *
                pow(double(N2) / double(N1), 2);

  for (int i = 0; i < nbGauss; ++i)
    for (int j = 0; j < nbGauss; ++j)
      for (int k = 0; k < nbGauss; ++k) {
        int offset = i * nbGauss * nbGauss + j * nbGauss + k;
        vpoints[offset] = Vector3d();
        vpoints[offset].x() = var2(i, 0);
        vpoints[offset].y() = var2(j, 0);
        vpoints[offset].z() = var2(k, 0);
        vweights[offset] = var2(i, 1) * var2(j, 1) * var2(k, 1);
      }
}

void computeCovarianceMatrix(const MatrixXd& mD, MatrixXd& mC) {
  Vector3d mean = mD.colwise().mean();

  mC.resize(mD.cols(), mD.cols());

  for (int i = 0; i < mD.cols(); ++i) {
    for (int j = 0; j < mD.cols(); ++j) {
      Real meanVar = 0;

      for (int k = 0; k < mD.rows(); ++k)
        meanVar += (mD(k, i) - mean(i)) * (mD(k, j) - mean(j));

      meanVar /= mD.rows();

      mC(i, j) = meanVar;
    }
  }
}

void computePCA(const MatrixXd& mD, MatrixXd& mU, MatrixXd& mV, VectorXd& vs) {
  MatrixXd mC;
  computeCovarianceMatrix(mD, mC);
  JacobiSVD<MatrixXd> svd(mC, ComputeFullU | ComputeFullV);
  mU = svd.matrixU();
  mV = svd.matrixV();
  vs = svd.singularValues();
}

Real computePolarDecomposition(const Matrix3d& M,
                               Matrix3d& Q,
                               Matrix3d& S,
                               Real tol) {
  double det = M.determinant();
  if (abs(det) < 1e-9) {
    return -1.0;
  }

  Matrix3d Mk;
  Matrix3d Ek;
  double M_oneNorm, M_infNorm, E_oneNorm;

  // Initialize
  Mk = M.transpose();
  M_oneNorm = Mk.lpNorm<1>();
  M_infNorm = Mk.lpNorm<Eigen::Infinity>();

  // Iterate
  do {
    Matrix3d MadjTk;

    Vector3d row0 = Mk.transpose().col(0);
    Vector3d row1 = Mk.transpose().col(1);
    Vector3d row2 = Mk.transpose().col(2);

    MadjTk.row(0) = row1.cross(row2);
    MadjTk.row(1) = row2.cross(row0);
    MadjTk.row(2) = row0.cross(row1);

    det = Mk(0, 0) * MadjTk(0, 0) + Mk(0, 1) * MadjTk(0, 1) +
          Mk(0, 2) * MadjTk(0, 2);
    if (abs(det) < 1e-9) {
      return -1.0;
    }

    double MadjT_one = MadjTk.lpNorm<1>();
    double MadjT_inf = MadjTk.lpNorm<Eigen::Infinity>();

    double gamma = sqrt(
        sqrt((MadjT_one * MadjT_inf) / (M_oneNorm * M_infNorm)) / fabs(det));

    double g2 = 0.5 / (gamma * det);
    double g1 = gamma * 0.5;

    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        Ek(i, j) = Mk(i, j);  // Modify previous Mk
        Mk(i, j) = g1 * Mk(i, j) + g2 * MadjTk(i, j);
        Ek(i, j) -= Mk(i, j);
      }

    E_oneNorm = Ek.lpNorm<1>();
    M_oneNorm = Mk.lpNorm<1>();
    M_infNorm = Mk.lpNorm<Eigen::Infinity>();
  } while (E_oneNorm > M_oneNorm * tol);

  Q = Mk.transpose();
  S = Q.inverse() * M;

  return 1.0;
}

// template <typename T, int N0, int N1>
// void Vectorize(const Matrix<T, N0, N1>& inMat, Vector<T, -1>& outVec)
//{
//	vecOut.resize(N0 * N1);
//	memcpy(outVec.data(),
//		   inMat.data(),
//		   N0*N1*sizeof(T));
//}

// template <typename T, int N0, int N1>
// void Unvectorize(const Vector<T, -1>& inVec, Matrix<T, N0, N1>& outMat)
//{
//	memcpy(outMat.data(),
//		   inVec.data(),
//		   N0*N1*sizeof(T));
//}

/*template <typename T, int N0, int N1, int N2>
void Vectorize(const Tensor3<T, N0, N1, N2>& inTen, MatrixX<T>& outMat)
{
        matOut.resize(N0*N1, N2);
        memcpy(outMat.data(),
                   inTen,
                   N0*N1*N2*sizeof(T));
}

template <typename T, int N0, int N1, int N2>
void Unvectorize(const MatrixX<T>& inMat, Tensor3<T, N0, N1, N2>& outTen)
{
        memcpy(outTen,
                   inMat.data(),
                   N0*N1*N2*sizeof(T));
}

template <typename T, int N0, int N1, int N2, int N3>
void Vectorize(const Tensor4r<N0, N1, N2, N3>& inTen, MatrixX<T>& outMat)
{
        outMat.resize(N0 * N1, N2 * N3);
        int colCount = 0;
        for (int j = 0; j < N3; ++j)
                for (int i = 0; i < N2; ++i)
                        Vectorize(inTen[i][j], outMat.cols(colCount++));
}

template <typename T, int N0, int N1, int N2, int N3>
void Unvectorize(const MatrixX<T>& inMat, Tensor4r<N0, N1, N2, N3>& outTen)
{
        int colCount = 0;
        for (int j = 0; j < N3; ++j)
                for (int i = 0; i < N2; ++i)
                        Unvectorize(inMat.cols(colCount++), outTen[i][j]);
}*/
}  // namespace MathUtils
}  // namespace PhySim