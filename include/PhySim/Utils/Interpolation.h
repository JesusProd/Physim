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

#include <PhySim/BasicTypes.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

struct Frame3d;

namespace Utils {
namespace Interpolation {
void computeSplineTangents_Cardinal(const MatrixXd& mV,
                                    const VectorXd& vS,
                                    MatrixXd& mT,
                                    Real tension = 0,
                                    bool loop = false);
void computeSplineTangents_FiniteDiff(const MatrixXd& mV,
                                      const VectorXd& vS,
                                      MatrixXd& mT,
                                      bool loop = false);
void computeSplineTangents_CatmullRom(const MatrixXd& mV,
                                      const VectorXd& vS,
                                      MatrixXd& mT,
                                      bool loop = false);

void computeArcLength_01(const MatrixXd& mVin, VectorXd& vsOut);
void computeArcLength_0L(const MatrixXd& mVin, VectorXd& vsOut);

void interpolateCurve_Hermite(const MatrixXd& mVin,
                              const MatrixXd& mTin,
                              const VectorXd& vSin,
                              int numPout,
                              MatrixXd& mVout);
void interpolateCurve_Hermite(const MatrixXd& mVin,
                              const MatrixXd& mTin,
                              const VectorXd& vSin,
                              Real tolerance,
                              MatrixXd& mVout);
void interpolateCurve_Hermite(const MatrixXd& mVin,
                              const MatrixXd& mTin,
                              const VectorXd& vSin,
                              const VectorXd& vSout,
                              MatrixXd& mVout);

void interpolateCurvePatch_Bezier(const MatrixXd& mVin,
                                  const VectorXd& vSin,
                                  int numPout,
                                  MatrixXd& mVout);
void interpolateCurvePatch_Bezier(const MatrixXd& mVin,
                                  const VectorXd& vSin,
                                  Real tolerance,
                                  MatrixXd& mVout);
void interpolateCurvePatch_Bezier(const MatrixXd& mVin,
                                  const VectorXd& vSin,
                                  const VectorXd& vSout,
                                  MatrixXd& mVout);

void interpolateSurfacePatch_Bezier(const vector<MatrixXd>& mVin,
                                    const vector<MatrixXd>& mSin,
                                    int nums,
                                    int numr,
                                    vector<MatrixXd>& mVout);
void interpolateSurfacePatch_Bezier(const vector<MatrixXd>& mVin,
                                    const vector<MatrixXd>& mSin,
                                    Real tols,
                                    Real tolr,
                                    vector<MatrixXd>& mVout);
void interpolateSurfacePatch_Bezier(const vector<MatrixXd>& mVin,
                                    const vector<MatrixXd>& mSin,
                                    const vector<MatrixXd>& mSout,
                                    vector<MatrixXd>& mVout);

void interpolationMatrixSurfacePatch_Bezier(const vector<MatrixXd>& mSin,
                                            int nums,
                                            int numr,
                                            MatrixXd& mV);
void interpolationMatrixSurfacePatch_Bezier(const vector<MatrixXd>& mSin,
                                            Real tols,
                                            Real tolr,
                                            MatrixXd& mV);
void interpolationMatrixSurfacePatch_Bezier(const vector<MatrixXd>& mSin,
                                            const vector<MatrixXd>& mSout,
                                            MatrixXd& mV);

Vector3d evaluate_BicubicHermite01(const vector<VectorXd>& vp,
                                   const vector<VectorXd>& vts,
                                   const vector<VectorXd>& vtr,
                                   const vector<VectorXd>& vtsr,
                                   Real s,
                                   Real r);

Vector3d evaluate_CubicHermite01(const VectorXd& p0,
                                 const VectorXd& p1,
                                 const VectorXd& t0,
                                 const VectorXd& t1,
                                 Real s);

void hermiteInterpolation(const vector<Vector3d>& vin,
                          vector<Vector3d>& vout,
                          bool loop,
                          int outnp,
                          const Vector3d& tanIni,
                          const Vector3d& tanEnd);

void hermiteInterpolation(const vector<Vector2d>& vin,
                          const dVector& vsin,
                          vector<Vector2d>& vout,
                          bool loop,
                          int outnp);

void hermiteInterpolation(const vector<Frame3d>& vin,
                          const dVector& vsin,
                          vector<Frame3d>& vout,
                          bool loop,
                          int outnp);

void hermiteInterpolation(const vector<Frame3d>& vin,
                          const dVector& vsin,
                          vector<Frame3d>& vout,
                          bool loop,
                          double outTol);

void hermiteInterpolation(const vector<Vector3d>& vin,
                          vector<Vector3d>& vout,
                          bool loop,
                          double outTol,
                          const Vector3d& tanIni,
                          const Vector3d& tanEnd);

void hermiteInterpolation(const vector<Vector2d>& vin,
                          const dVector& vsin,
                          vector<Vector2d>& vout,
                          bool loop,
                          double outTol);
}  // namespace Interpolation
}  // namespace Utils
}  // namespace PhySim
