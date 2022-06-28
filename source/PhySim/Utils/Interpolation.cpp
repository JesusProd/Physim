//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Utils/Interpolation.h>

#include <PhySim/Utils/MathUtils.h>
#include <PhySim/Utils/GeometryUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

namespace Utils {

namespace Interpolation {

void computeArcLength_0L(const MatrixXd& mVin, VectorXd& vsOut) {
  vsOut.resize(mVin.rows());

  vsOut[0] = 0.0;
  for (int i = 1; i < mVin.rows(); ++i)
    vsOut[i] = vsOut[i - 1] + (mVin.row(i) - mVin.row(i - 1)).norm();
}

void computeArcLength_01(const MatrixXd& mVin, VectorXd& vsOut) {
  computeArcLength_0L(mVin, vsOut);
  for (int i = 0; i < mVin.rows(); ++i)
    vsOut[i] /= vsOut[mVin.rows() - 1];
}

void computeSplineTangents_Cardinal(const MatrixXd& mV,
                                    const VectorXd& vS,
                                    MatrixXd& mT,
                                    Real tension,
                                    bool loop) {
  assert(mV.rows() == vS.size());

  int N = mV.rows();
  int M = mV.cols();
  mT.resize(N, M);

  if (!loop) {
    mT.row(0) = (mV.row(1) - mV.row(0)) / (vS[1] - vS[0]);
    mT.row(N - 1) = (mV.row(N - 1) - mV.row(N - 2)) / (vS[N - 1] - vS[N - 2]);
  } else {
    Vector3d t =
        (mV.row(1) - mV.row(N - 2)) / (vS[1] - vS[0] + vS[N - 1] - vS[N - 2]);
    mT.row(0) = t;
    mT.row(N - 1) = t;
  }

  for (int i = 1; i < N - 1; ++i)
    mT.row(i) = (1 - tension) * (mV.row(i + 1) - mV.row(i - 1)) /
                (vS[i + 1] - vS[i - 1]);
}

void computeSplineTangents_FiniteDiff(const MatrixXd& mV,
                                      const VectorXd& vS,
                                      MatrixXd& mT,
                                      bool loop) {
  assert(mV.rows() == vS.size());

  int N = mV.rows();
  int M = mV.cols();
  mT.resize(N, M);

  if (!loop) {
    mT.row(0) = (mV.row(1) - mV.row(0)) / (vS[1] - vS[0]);
    mT.row(N - 1) = (mV.row(N - 1) - mV.row(N - 2)) / (vS[N - 1] - vS[N - 2]);
  } else {
    Vector3d t =
        0.5 * (((mV.row(N - 1) - mV.row(N - 2)) / (vS[N - 1] - vS[N - 2])) +
               ((mV.row(1) - mV.row(0)) / (vS[1] - vS[0])));
    mT.row(0) = t;
    mT.row(N - 1) = t;
  }

  for (int i = 1; i < N - 1; ++i)
    mT.row(i) = 0.5 * (((mV.row(i) - mV.row(i - 1)) / (vS[i] - vS[i - 1])) +
                       ((mV.row(i + 1) - mV.row(i)) / (vS[i + 1] - vS[i])));
}

void computeSplineTangents_CatmullRom(const MatrixXd& mV,
                                      const VectorXd& vS,
                                      MatrixXd& mT,
                                      bool loop) {
  computeSplineTangents_Cardinal(mV, vS, mT, 0, loop);
}

void interpolateCurve_Hermite(const MatrixXd& mVin,
                              const MatrixXd& mTin,
                              const VectorXd& vSin,
                              Real tolerance,
                              MatrixXd& mVout) {
  int Nin = vSin.size();
  Real minS = vSin(0);
  Real maxS = vSin(Nin - 1);
  Real ranS = maxS - minS;

  interpolateCurve_Hermite(mVin, mTin, vSin, (int)ceil(ranS / tolerance) + 1,
                           mVout);
}

void interpolateCurve_Hermite(const MatrixXd& mVin,
                              const MatrixXd& mTin,
                              const VectorXd& vSin,
                              int Nout,
                              MatrixXd& mVout) {
  VectorXd vSout(Nout);

  int Nin = vSin.size();
  Real minS = vSin(0);
  Real maxS = vSin(Nin - 1);
  Real ranS = maxS - minS;
  Real incS = ranS / (Nout + 1);

  for (int i = 0; i < Nout; ++i)
    vSout[i] = i * incS;

  interpolateCurve_Hermite(mVin, mTin, vSin, vSout, mVout);
}

void interpolateCurve_Hermite(const MatrixXd& mVin,
                              const MatrixXd& mTin,
                              const VectorXd& vSin,
                              const VectorXd& vSout,
                              MatrixXd& mVout) {
  int Nin = vSin.size();
  int Nout = vSout.size();
  int D = mVin.cols();

  mVout.resize(Nout, D);
  mVout.row(0) = mVin.row(0);

  int itD = 0;
  for (int i = 1; i < Nout - 1; i++) {
    while (vSout[i] > vSin[itD + 1])
      itD++;

    Real ds = vSin[itD + 1] - vSin[itD];
    double x = (vSout[i] - vSin[itD]) / ds;
    double x2 = x * x;
    double x3 = x * x * x;

    const Vector3d& p0 = mVin.row(itD);      // In point 0
    const Vector3d& t0 = mTin.row(itD);      // In tangent 0
    const Vector3d& p1 = mVin.row(itD + 1);  // In point 1
    const Vector3d& t1 = mTin.row(itD + 1);  // In tangent 1

    mVout.row(i) = evaluate_CubicHermite01(
        p0, p1, t0 * ds, t1 * ds, x);
  }

  mVout.row(Nout - 1) = mVin.row(Nin - 1);
}

Vector3d evaluate_CubicHermite01(const VectorXd& p0,
                                 const VectorXd& p1,
                                 const VectorXd& t0,
                                 const VectorXd& t1,
                                 Real x) {
  Real x2 = x * x;
  Real x3 = x * x * x;
  return p0 * (2 * x3 - 3 * x2 + 1) + t0 * (x3 - 2 * x2 + x) +
         p1 * (-2 * x3 + 3 * x2) + t1 * (x3 - x2);
}

Vector3d evaluate_BicubicHermite01(const vector<VectorXd>& vp,
                                   const vector<VectorXd>& vts,
                                   const vector<VectorXd>& vtr,
                                   const vector<VectorXd>& vtsr,
                                   Real s,
                                   Real r) {
  MatrixXd mA(4, 4);
  mA(0, 0) = 1;
  mA(0, 1) = 0;
  mA(0, 2) = 0;
  mA(0, 3) = 0;
  mA(1, 0) = 0;
  mA(1, 1) = 0;
  mA(1, 2) = 1;
  mA(1, 3) = 0;
  mA(2, 0) = -3;
  mA(2, 1) = 3;
  mA(2, 2) = -2;
  mA(2, 3) = -1;
  mA(3, 0) = 2;
  mA(3, 1) = -2;
  mA(3, 2) = 1;
  mA(3, 3) = 1;

  VectorXd vs(4);
  VectorXd vr(4);
  vs[0] = 1;
  vr[0] = 1;
  for (int i = 1; i < 4; i++) {
    vs[i] = vs[i - 1] * s;
    vr[i] = vr[i - 1] * r;
  }

  Vector3d vx;

  for (int i = 0; i < 3; ++i) {
    MatrixXd mX(4, 4);
    mX(0, 0) = vp[0][i];
    mX(0, 1) = vp[1][i];
    mX(1, 0) = vp[2][i];
    mX(1, 1) = vp[3][i];
    mX(0, 2) = vtr[0][i];
    mX(0, 3) = vtr[1][i];
    mX(1, 2) = vtr[2][i];
    mX(1, 3) = vtr[3][i];
    mX(2, 0) = vts[0][i];
    mX(2, 1) = vts[1][i];
    mX(3, 0) = vts[2][i];
    mX(3, 1) = vts[3][i];
    mX(2, 2) = vtsr[0][i];
    mX(2, 3) = vtsr[1][i];
    mX(3, 2) = vtsr[2][i];
    mX(3, 3) = vtsr[3][i];
    vx[i] = vs.transpose() * mA * mX * mA.transpose() * vs;
  }

  return vx;
}

void interpolateCurvePatch_Bezier(const MatrixXd& mVin,
                                  const VectorXd& vSin,
                                  Real tolerance,
                                  MatrixXd& mVout) {
  int Nin = vSin.size();
  Real minS = vSin(0);
  Real maxS = vSin(Nin - 1);
  Real ranS = maxS - minS;

  interpolateCurvePatch_Bezier(mVin, vSin, (int)ceil(ranS / tolerance) + 1,
                               mVout);
}

void interpolateCurvePatch_Bezier(const MatrixXd& mVin,
                                  const VectorXd& vSin,
                                  int Nout,
                                  MatrixXd& mVout) {
  VectorXd vSout(Nout);

  int Nin = vSin.size();
  Real minS = vSin(0);
  Real maxS = vSin(Nin - 1);
  Real ranS = maxS - minS;
  Real incS = ranS / (Nout + 1);

  for (int i = 0; i < Nout; ++i)
    vSout[i] = i * incS;

  interpolateCurvePatch_Bezier(mVin, vSin, vSout, mVout);
}

void interpolateCurvePatch_Bezier(const MatrixXd& mVin,
                                  const VectorXd& vSin,
                                  const VectorXd& vSout,
                                  MatrixXd& mVout) {
  int Nin = mVin.rows();
  int Nout = vSin.size();
  int D = mVin.cols();

  mVout.resize(Nout, D);
  mVout.row(0) = mVin.row(0);

  Real minS = vSout(0);
  Real maxS = vSout(Nout - 1);
  Real ranS = maxS - minS;

  for (int i = 1; i < Nout - 1; ++i) {
    VectorXd vc;
    Real x = (vSout[i] - minS) / ranS;
    MathUtils::computeBernsteinCoefficients(Nin - 1, x, vc);

    mVout.row(i).setZero();
    for (int j = 0; j < Nin; ++j)
      mVout.row(i) += vc[j] * mVin.row(j);
  }

  mVout.row(Nout - 1) = mVin.row(Nin - 1);
}

void interpolateSurfacePatch_Bezier(const vector<MatrixXd>& mVin,
                                    const vector<MatrixXd>& mSin,
                                    Real tols,
                                    Real tolr,
                                    vector<MatrixXd>& mVout) {
  assert(mVin.size() > 0);
  assert(mSin.size() > 0);

  int Nin = mSin[0].rows();
  int Min = mSin[0].cols();

  Real minS = mSin[0](0, 0);
  Real minR = mSin[1](0, 1);
  Real maxS = mSin[0](0, Nin - 1);
  Real maxR = mSin[1](Nin - 1, 0);
  Real ranS = maxS - minS;
  Real ranR = maxR - minR;
  interpolateSurfacePatch_Bezier(mVin, mSin, (int)ceil(ranS / tols),
                                 (int)ceil(ranR / tolr), mVout);
}

void interpolateSurfacePatch_Bezier(const vector<MatrixXd>& mVin,
                                    const vector<MatrixXd>& mSin,
                                    int nums,
                                    int numr,
                                    vector<MatrixXd>& mVout) {
  assert(mVin.size() > 0);
  assert(mSin.size() > 0);

  int Nin = mSin[0].rows();
  int Min = mSin[0].cols();

  vector<MatrixXd> mSout(2);
  mSout[0].resize(nums, numr);
  mSout[1].resize(nums, numr);

  Real minS = mSin[0](0, 0);
  Real minR = mSin[1](0, 0);
  Real maxS = mSin[0](Nin - 1, 0);
  Real maxR = mSin[1](0, Min - 1);
  Real incS = (maxS - minS) / (nums - 1);
  Real incR = (maxR - minR) / (nums - 1);

  for (int i = 0; i < nums; ++i) {
    for (int j = 0; j < numr; ++j) {
      mSout[0](i, j) = incS * i;
      mSout[1](i, j) = incR * j;
    }
  }

  interpolateSurfacePatch_Bezier(mVin, mSin, mSout, mVout);
}

void interpolateSurfacePatch_Bezier(const vector<MatrixXd>& mVin,
                                    const vector<MatrixXd>& mSin,
                                    const vector<MatrixXd>& mSout,
                                    vector<MatrixXd>& mVout) {
  assert(mVin.size() > 0);
  assert(mSin.size() > 0);

  int D = (int)mVin.size();
  int Nin = mSin[0].rows();
  int Min = mSin[0].cols();
  int Nout = mSout[0].rows();
  int Mout = mSout[0].cols();

  mVout.resize(D);
  for (int i = 0; i < D; ++i)
    mVout[i].resize(Nout, Mout);

  for (int i = 0; i < Nout; ++i) {
    Real minR = mSout[1](i, 0);
    Real maxR = mSout[1](i, Mout - 1);
    Real ranR = maxR - minR;

    for (int j = 0; j < Mout; ++j) {
      Real minS = mSout[0](0, j);
      Real maxS = mSout[0](Nout - 1, j);
      Real ranS = maxS - minS;

      VectorXd vcx;
      VectorXd vcy;
      Real x = (mSout[0](i, j) - minS) / ranS;
      Real y = (mSout[1](i, j) - minR) / ranR;
      MathUtils::computeBernsteinCoefficients(Nin - 1, x, vcx);
      MathUtils::computeBernsteinCoefficients(Min - 1, y, vcy);

      for (int d = 0; d < D; ++d) {
        mVout[d](i, j) = 0;
        for (int k = 0; k < Nin; ++k)
          for (int l = 0; l < Min; ++l)
            mVout[d](i, j) += vcx[k] * vcy[l] * mVin[d](k, l);
      }
    }
  }
}

void interpolationMatrixSurfacePatch_Bezier(const vector<MatrixXd>& mSin,
                                            int nums,
                                            int numr,
                                            MatrixXd& mV) {
  assert(mSin.size() > 0);

  int Nin = mSin[0].rows();
  int Min = mSin[0].cols();

  vector<MatrixXd> mSout(2);
  mSout[0].resize(nums, numr);
  mSout[1].resize(nums, numr);

  Real minS = mSin[0](0, 0);
  Real minR = mSin[1](0, 0);
  Real maxS = mSin[0](Nin - 1, 0);
  Real maxR = mSin[1](0, Min - 1);
  Real incS = (maxS - minS) / (nums - 1);
  Real incR = (maxR - minR) / (nums - 1);

  for (int i = 0; i < nums; ++i) {
    for (int j = 0; j < numr; ++j) {
      mSout[0](i, j) = incS * i;
      mSout[1](i, j) = incR * j;
    }
  }

  interpolationMatrixSurfacePatch_Bezier(mSin, mSout, mV);
}

void interpolationMatrixSurfacePatch_Bezier(const vector<MatrixXd>& mSin,
                                            Real tols,
                                            Real tolr,
                                            MatrixXd& mV) {
  assert(mSin.size() > 0);

  int Nin = mSin[0].rows();
  int Min = mSin[0].cols();

  Real minS = mSin[0](0, 0);
  Real minR = mSin[1](0, 1);
  Real maxS = mSin[0](0, Nin - 1);
  Real maxR = mSin[1](Nin - 1, 0);
  Real ranS = maxS - minS;
  Real ranR = maxR - minR;
  interpolationMatrixSurfacePatch_Bezier(mSin, (int)ceil(ranS / tols),
                                         (int)ceil(ranR / tolr), mV);
}

void interpolationMatrixSurfacePatch_Bezier(const vector<MatrixXd>& mSin,
                                            const vector<MatrixXd>& mSout,
                                            MatrixXd& mV) {
  assert(mSin.size() > 0);

  int Nin = mSin[0].rows();
  int Min = mSin[0].cols();
  int Nout = mSout[0].rows();
  int Mout = mSout[0].cols();

  mV.resize(Nout * Mout, Nin * Min);

  mV.setZero();

  for (int i = 0; i < Nout; ++i) {
    Real minR = mSout[1](i, 0);
    Real maxR = mSout[1](i, Mout - 1);
    Real ranR = maxR - minR;

    for (int j = 0; j < Mout; ++j) {
      Real minS = mSout[0](0, j);
      Real maxS = mSout[0](Nout - 1, j);
      Real ranS = maxS - minS;

      VectorXd vcx;
      VectorXd vcy;
      Real x = (mSout[0](i, j) - minS) / ranS;
      Real y = (mSout[1](i, j) - minR) / ranR;
      MathUtils::computeBernsteinCoefficients(Nin - 1, x, vcx);
      MathUtils::computeBernsteinCoefficients(Min - 1, y, vcy);

      int offOut = Mout * i + j;

      for (int k = 0; k < Nin; ++k) {
        for (int l = 0; l < Min; ++l) {
          int offIn = Min * k + l;

          mV(offOut, offIn) = vcx[k] * vcy[l];
        }
      }
    }
  }
}

void hermiteInterpolation(const vector<Vector3d>& vin,
                          vector<Vector3d>& vout,
                          bool loop,
                          int outnp,
                          const Vector3d& tanIni,
                          const Vector3d& tanEnd) {
  int innp = (int)vin.size();

  dVector vsin;
  for (int i = 0; i < innp - 1; ++i)
    vsin.push_back((vin[i + 1] - vin[i]).norm());

  int inns = (int)vsin.size();

  double sT = 0;
  for (int i = 0; i < inns; ++i)
    sT += vsin[i];  // Sum length

  int outns = outnp - 1;     // Out number of domains
  double outs = sT / outns;  // Out domain length

  // Must match ends
  vout.resize(outnp);
  vout.back() = vin.back();
  vout.front() = vin.front();

  int sit = 0;
  int nxtp = 1;
  double sums = 0;
  double nxts = outs;
  for (int i = 1; i < outnp - 1; i++) {
    double s = vsin[sit];
    while (i * outs > sums + s) {
      sit++;
      sums += s;
      s = vsin[sit];
    }

    double t = (nxts - sums) / s;
    double t2 = t * t;
    double t3 = t * t * t;

    const Vector3d& p0 = vin[sit];        // In point 0
    const Vector3d& p1 = vin[sit + 1];    // In point 1
    Vector3d ts = (p1 - p0) * (1.0 / s);  // In tan

    // Get tangents
    Vector3d t0, t1;
    if (inns == 1) {
      if (tanIni.norm() > 0.0) {
        t0 = tanIni;
      } else
        t0 = ts;

      if (tanEnd.norm() > 0.0) {
        t1 = tanEnd;
      } else
        t1 = ts;
    } else if (sit == 0 && !loop)  // First domain (no loop)
    {
      if (tanIni.norm() > 0.0) {
        t0 = tanIni;
      } else
        t0 = ts;

      const Vector3d& pn = vin[sit + 2];
      t1 = 0.5 * (ts + (pn - p1) * (1.0 / s));
    } else if (sit == inns - 1 && !loop)  // Last domain (no loop)
    {
      if (tanEnd.norm() > 0.0) {
        t1 = tanEnd;
      } else
        t1 = ts;

      const Vector3d& pp = vin[sit - 1];
      t0 = 0.5 * (ts + (p0 - pp) * (1.0 / s));
    } else if (sit == 0 && loop)  // First domain (loop)
    {
      const Vector3d& pp = vin[innp - 2];
      const Vector3d& pn = vin[sit + 2];
      t0 = 0.5 * (ts + (p0 - pp) * (1.0 / s));  // Mean
      t1 = 0.5 * (ts + (pn - p1) * (1.0 / s));  // Mean
    } else if (sit == inns - 1 && loop)         // Last domain (loop)
    {
      const Vector3d& pp = vin[sit - 1];
      const Vector3d& pn = vin[0];
      t0 = 0.5 * (ts + (p0 - pp) * (1.0 / s));  // Mean
      t1 = 0.5 * (ts + (pn - p1) * (1.0 / s));  // Mean
    } else {
      const Vector3d& pp = vin[sit - 1];
      const Vector3d& pn = vin[sit + 2];
      t0 = 0.5 * (ts + (p0 - pp) * (1.0 / s));  // Mean
      t1 = 0.5 * (ts + (pn - p1) * (1.0 / s));  // Mean
    }

    // Hermite interpolation

    Vector3d p = p0 * (2 * t3 - 3 * t2 + 1) + t0 * (t3 - 2 * t2 + t) * s +
                 p1 * (-2 * t3 + 3 * t2) + t1 * (t3 - t2) * s;

    vout[nxtp++] = p;  // Point

    nxts += outs;
  }
}

void hermiteInterpolation(const vector<Vector2d>& vin,
                          const dVector& vsin,
                          vector<Vector2d>& vout,
                          bool loop,
                          int outnp) {
  int innp = (int)vin.size();
  int inns = (int)vsin.size();

  assert(inns == innp - 1);

  double sT = 0;
  for (int i = 0; i < inns; ++i)
    sT += vsin[i];  // Sum length

  int outns = outnp - 1;     // Out number of domains
  double outs = sT / outns;  // Out domain length

  // Must match ends
  vout.resize(outnp);
  vout.back() = vin.back();
  vout.front() = vin.front();

  int sit = 0;
  int nxtp = 1;
  double sums = 0;
  double nxts = outs;
  for (int i = 1; i < outnp - 1; i++) {
    double s = vsin[sit];
    while (i * outs > sums + s) {
      sit++;
      sums += s;
      s = vsin[sit];
    }

    double t = (nxts - sums) / s;
    double t2 = t * t;
    double t3 = t * t * t;

    const Vector2d& p0 = vin[sit];        // In point 0
    const Vector2d& p1 = vin[sit + 1];    // In point 1
    Vector2d ts = (p1 - p0) * (1.0 / s);  // In tan

    // Get tangents
    Vector2d t0, t1;
    if (inns == 1) {
      t0 = ts;
      t1 = ts;
    } else if (sit == 0 && !loop)  // First domain (no loop)
    {
      t0 = ts;
      const Vector2d& pn = vin[sit + 2];
      t1 = 0.5 * (ts + (pn - p1) * (1.0 / s));
    } else if (sit == inns - 1 && !loop)  // Last domain (no loop)
    {
      t1 = ts;
      const Vector2d& pp = vin[sit - 1];
      t0 = 0.5 * (ts + (p0 - pp) * (1.0 / s));
    } else if (sit == 0 && loop)  // First domain (loop)
    {
      const Vector2d& pp = vin[innp - 2];
      const Vector2d& pn = vin[sit + 2];
      t0 = 0.5 * (ts + (p0 - pp) * (1.0 / s));  // Mean
      t1 = 0.5 * (ts + (pn - p1) * (1.0 / s));  // Mean
    } else if (sit == inns - 1 && loop)         // Last domain (loop)
    {
      const Vector2d& pp = vin[sit - 1];
      const Vector2d& pn = vin[0];
      t0 = 0.5 * (ts + (p0 - pp) * (1.0 / s));  // Mean
      t1 = 0.5 * (ts + (pn - p1) * (1.0 / s));  // Mean
    } else {
      const Vector2d& pp = vin[sit - 1];
      const Vector2d& pn = vin[sit + 2];
      t0 = 0.5 * (ts + (p0 - pp) * (1.0 / s));  // Mean
      t1 = 0.5 * (ts + (pn - p1) * (1.0 / s));  // Mean
    }

    // Hermite interpolation

    Vector2d p = p0 * (2 * t3 - 3 * t2 + 1) + t0 * (t3 - 2 * t2 + t) * s +
                 p1 * (-2 * t3 + 3 * t2) + t1 * (t3 - t2) * s;

    vout[nxtp++] = p;  // Point

    nxts += outs;
  }
}

void hermiteInterpolation(const vector<Frame3d>& vin,
                          const dVector& vsin,
                          vector<Frame3d>& vout,
                          bool loop,
                          int outnp) {
  int innp = (int)vin.size();
  int inns = (int)vsin.size();

  assert(inns == innp - 1);

  double sT = 0;
  for (int i = 0; i < inns; ++i)
    sT += vsin[i];  // Sum length

  int outns = outnp - 1;     // Out number of domains
  double outs = sT / outns;  // Out domain length

  // Must match ends
  vout.resize(outnp);
  vout.back() = vin.back();
  vout.front() = vin.front();

  int sit = 0;
  int nxtp = 1;
  double sums = 0;
  double nxts = outs;
  for (int i = 1; i < outnp - 1; i++) {
    double s = vsin[sit];
    while (i * outs > sums + s) {
      sit++;
      sums += s;
      s = vsin[sit];
    }

    double t = (nxts - sums) / s;
    double t2 = t * t;
    double t3 = t * t * t;

    const Frame3d& p0 = vin[sit];      // In point 0
    const Frame3d& p1 = vin[sit + 1];  // In point 1
    Frame3d ts;                        // In tan
    ts.tan = (p1.tan - p0.tan) * (1.0 / s);
    ts.nor = (p1.nor - p0.nor) * (1.0 / s);
    ts.bin = (p1.bin - p0.bin) * (1.0 / s);

    // Get tangents
    Frame3d t0, t1;
    if (inns == 1) {
      t0 = ts;
      t1 = ts;
    } else if (sit == 0 && !loop)  // First domain (no loop)
    {
      t0 = ts;
      const Frame3d& pn = vin[sit + 2];
      t1.tan = 0.5 * (ts.tan + (pn.tan - p1.tan) * (1.0 / s));
      t1.nor = 0.5 * (ts.nor + (pn.nor - p1.nor) * (1.0 / s));
      t1.bin = 0.5 * (ts.bin + (pn.bin - p1.bin) * (1.0 / s));
    } else if (sit == inns - 1 && !loop)  // Last domain (no loop)
    {
      t1 = ts;
      const Frame3d& pp = vin[sit - 1];
      t0.tan = 0.5 * (ts.tan + (p0.tan - pp.tan) * (1.0 / s));
      t0.nor = 0.5 * (ts.nor + (p0.nor - pp.nor) * (1.0 / s));
      t0.bin = 0.5 * (ts.bin + (p0.bin - pp.bin) * (1.0 / s));
    } else if (sit == 0 && loop)  // First domain (loop)
    {
      const Frame3d& pp = vin[innp - 2];
      const Frame3d& pn = vin[sit + 2];
      t0.tan = 0.5 * (ts.tan + (p0.tan - pp.tan) * (1.0 / s));  // Mean
      t0.nor = 0.5 * (ts.nor + (p0.nor - pp.nor) * (1.0 / s));  // Mean
      t0.bin = 0.5 * (ts.bin + (p0.bin - pp.bin) * (1.0 / s));  // Mean

      t1.tan = 0.5 * (ts.tan + (pn.tan - p1.tan) * (1.0 / s));  // Mean
      t1.nor = 0.5 * (ts.nor + (pn.nor - p1.nor) * (1.0 / s));  // Mean
      t1.bin = 0.5 * (ts.bin + (pn.bin - p1.bin) * (1.0 / s));  // Mean
    } else if (sit == inns - 1 && loop)  // Last domain (loop)
    {
      const Frame3d& pp = vin[sit - 1];
      const Frame3d& pn = vin[0];
      t0.tan = 0.5 * (ts.tan + (p0.tan - pp.tan) * (1.0 / s));  // Mean
      t0.nor = 0.5 * (ts.nor + (p0.nor - pp.nor) * (1.0 / s));  // Mean
      t0.bin = 0.5 * (ts.bin + (p0.bin - pp.bin) * (1.0 / s));  // Mean

      t1.tan = 0.5 * (ts.tan + (pn.tan - p1.tan) * (1.0 / s));  // Mean
      t1.nor = 0.5 * (ts.nor + (pn.nor - p1.nor) * (1.0 / s));  // Mean
      t1.bin = 0.5 * (ts.bin + (pn.bin - p1.bin) * (1.0 / s));  // Mean
    } else {
      const Frame3d& pp = vin[sit - 1];
      const Frame3d& pn = vin[sit + 2];
      t0.tan = 0.5 * (ts.tan + (p0.tan - pp.tan) * (1.0 / s));  // Mean
      t0.nor = 0.5 * (ts.nor + (p0.nor - pp.nor) * (1.0 / s));  // Mean
      t0.bin = 0.5 * (ts.bin + (p0.bin - pp.bin) * (1.0 / s));  // Mean

      t1.tan = 0.5 * (ts.tan + (pn.tan - p1.tan) * (1.0 / s));  // Mean
      t1.nor = 0.5 * (ts.nor + (pn.nor - p1.nor) * (1.0 / s));  // Mean
      t1.bin = 0.5 * (ts.bin + (pn.bin - p1.bin) * (1.0 / s));  // Mean
    }

    // Hermite interpolation

    Frame3d p;

    p.tan = p0.tan * (2 * t3 - 3 * t2 + 1) + t0.tan * (t3 - 2 * t2 + t) * s +
            p1.tan * (-2 * t3 + 3 * t2) + t1.tan * (t3 - t2) * s;

    p.bin = p0.bin * (2 * t3 - 3 * t2 + 1) + t0.bin * (t3 - 2 * t2 + t) * s +
            p1.bin * (-2 * t3 + 3 * t2) + t1.bin * (t3 - t2) * s;

    p.nor = p0.nor * (2 * t3 - 3 * t2 + 1) + t0.nor * (t3 - 2 * t2 + t) * s +
            p1.nor * (-2 * t3 + 3 * t2) + t1.nor * (t3 - t2) * s;

    GeometryUtils::orthonormalizeFrame(p);

    vout[nxtp++] = p;  // Point

    nxts += outs;
  }
}

void hermiteInterpolation(const vector<Frame3d>& vin,
                          const dVector& vsin,
                          vector<Frame3d>& vout,
                          bool loop,
                          double outTol) {
  assert(vin.size() - 1 == vsin.size());

  double length = 0;
  int nPoint = (int)vin.size();
  for (int i = 0; i < nPoint - 1; ++i)
    length += vsin[i];  // Add total

  int outnp = (int)ceil(length / outTol) + 1;  // Output
  hermiteInterpolation(vin, vsin, vout, loop, outnp);
}

void hermiteInterpolation(const vector<Vector3d>& vin,
                          vector<Vector3d>& vout,
                          bool loop,
                          double outTol,
                          const Vector3d& tanIni,
                          const Vector3d& tanEnd) {
  double length = 0;
  int nPoint = (int)vin.size();
  for (int i = 0; i < nPoint - 1; ++i)
    length += (vin[i + 1] - vin[i]).norm();
  int outnp = (int)ceil(length / outTol) + 1;  // Output

  hermiteInterpolation(vin, vout, loop, outnp, tanIni, tanEnd);
}

void hermiteInterpolation(const vector<Vector2d>& vin,
                          const dVector& vsin,
                          vector<Vector2d>& vout,
                          bool loop,
                          double outTol) {
  assert(vin.size() - 1 == vsin.size());

  double length = 0;
  int nPoint = (int)vin.size();
  for (int i = 0; i < nPoint - 1; ++i)
    length += vsin[i];  // Add total

  int outnp = (int)ceil(length / outTol) + 1;  // Output
  hermiteInterpolation(vin, vsin, vout, loop, outnp);
}
}  // namespace Interpolation
}  // namespace Utils
}  // namespace PhySim
