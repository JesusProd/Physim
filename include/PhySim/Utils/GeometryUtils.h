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

namespace GeometryUtils {

void eulerRotation(const double* eulerAngles,
                   const double* vectorOriginal,
                   double vectorRotated[3]);

Matrix3d rotationEulerToMatrix(const Vector3d& ve);
Vector3d rotationMatrixToEuler(const Matrix3d& mR);
void rotationEulerToMatrix(const Vector3d& ve, Matrix3d& mR);
void rotationMatrixToEuler(const Matrix3d& mR, Vector3d& ve);
Matrix3d rotationAxisAngleToMatrix(const Vector3d& vaa);
Vector3d rotationMatrixToAxisAngle(const Matrix3d& mR);
void rotationAxisAngleToMatrix(const Vector3d& vaa, Matrix3d& mR);
void rotationMatrixToAxisAngle(const Matrix3d& mR, Vector3d& vaa);

Real distanceToEdge(const Vector3d& vP, const Vector3d& vD);
Real distanceToPlane(const Vector3d& vP, const Vector3d& vN);
Real distanceToUnnormalizedPlane(const Vector3d& vP, const Vector3d& vN);
Real distanceToTriangle(const Vector3d& vP,
                        const Vector3d& vA,
                        const Vector3d& vB,
                        const Vector3d& vC);

Real sqrDistanceToEdge(const Vector3d& vP, const Vector3d& vD);
Real sqrDistanceToPlane(const Vector3d& vP, const Vector3d& vN);
Real sqrDistanceToUnnormalizedPlane(const Vector3d& vP, const Vector3d& vN);
Real sqrDistanceToTriangle(const Vector3d& vP,
                           const Vector3d& vA,
                           const Vector3d& vB,
                           const Vector3d& vC);

Real determineTriangleDistanceSign(const Vector3d& vP,
                                   const Vector3d& vA,
                                   const Vector3d& vB,
                                   const Vector3d& vC,
                                   const Vector3d& vNA,
                                   const Vector3d& vNB,
                                   const Vector3d& vNC,
                                   const Vector3d& vNAB,
                                   const Vector3d& vNBC,
                                   const Vector3d& vNCA);

void computeBestAxisAligment(const MatrixXd& mCloud0,
                             MatrixXd& mR,
                             VectorXd& vt);

void computeBestRigidTransform(const MatrixXd& mCloud0,
                               const MatrixXd& mCloud1,
                               MatrixXd& mR,
                               VectorXd& vt);

void computeBoundingBox(MatrixXd& mV, Vector3d& vmin, Vector3d& vmax);

Frame3d rotateFrame(const Frame3d& F, Real angle);
Frame3d rotateFrame(const Frame3d& F, Matrix3d& R);

void orthonormalizeFrame(Frame3d& F);

void rotateFrame(const vector<Frame3d>& vF,
                 const dVector& va,
                 vector<Frame3d>& vFRot);

Frame3d parallelTransport(const Frame3d& F, const Vector3d& t2);
Vector3d parallelTransport(const Vector3d& u,
                           const Vector3d& t1,
                           const Vector3d& t2);
Vector3d parallelTransportNormalized(const Vector3d& u,
                                     const Vector3d& t1,
                                     const Vector3d& t2);

Vector3d computeCurvatureBinormal(const Vector3d& e0, const Vector3d& e1);

bool lineLineIntersect(const dVector& p1,
                       const dVector& p2,
                       const dVector& p3,
                       const dVector& p4,
                       double& mua,
                       double& mub,
                       double tol);
}  // namespace GeometryUtils
}  // namespace PhySim
