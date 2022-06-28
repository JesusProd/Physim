//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================
#include <PhySim/Utils/GeometryUtils.h>

#include <PhySim/Utils/Auto/rodriguesRotation.h>

#include <PhySim/Utils/MathUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

namespace GeometryUtils {
    using namespace MathUtils;

void eulerRotation(const double* eulerAngles,
                   const double* vectorOriginal,
                   double cgret[3]) {
  double t1;
  double t10;
  double t11;
  double t13;
  double t14;
  double t17;
  double t2;
  double t22;
  double t26;
  double t3;
  double t4;
  double t6;
  double t8;
  double t9;
  double vectorRotated[3];
  vectorRotated[0] = 0;
  vectorRotated[1] = 0;
  vectorRotated[2] = 0;
  t1 = eulerAngles[2];
  t2 = cos(t1);
  t3 = eulerAngles[1];
  t4 = cos(t3);
  t6 = vectorOriginal[0];
  t8 = sin(t3);
  t9 = t2 * t8;
  t10 = eulerAngles[0];
  t11 = sin(t10);
  t13 = sin(t1);
  t14 = cos(t10);
  t17 = vectorOriginal[1];
  t22 = vectorOriginal[2];
  vectorRotated[0] = t2 * t4 * t6 + (t9 * t11 - t13 * t14) * t17 +
                     (t9 * t14 + t13 * t11) * t22;
  t26 = t13 * t8;
  vectorRotated[1] = t13 * t4 * t6 + (t26 * t11 + t2 * t14) * t17 +
                     (t26 * t14 - t2 * t11) * t22;
  vectorRotated[2] = -t8 * t6 + t4 * t11 * t17 + t4 * t14 * t22;
  cgret[0] = vectorRotated[0];
  cgret[1] = vectorRotated[1];
  cgret[2] = vectorRotated[2];
}

Matrix3d rotationEulerToMatrix(const Vector3d& ve) {
  Matrix3d mR;
  rotationEulerToMatrix(ve, mR);
  return mR;
}

Vector3d rotationMatrixToEuler(const Matrix3d& mR) {
  Vector3d ve;
  rotationMatrixToEuler(mR, ve);
  return ve;
}

void rotationEulerToMatrix(const Vector3d& ve, Matrix3d& mR) {
  Matrix3d Rx;
  Matrix3d Ry;
  Matrix3d Rz;

  Rx(0, 0) = 1;
  Rx(0, 1) = 0;
  Rx(0, 2) = 0;
  Rx(1, 0) = 0;
  Rx(1, 1) = cos(ve[0]);
  Rx(1, 2) = -sin(ve[0]);
  Rx(2, 0) = 0;
  Rx(2, 1) = sin(ve[0]);
  Rx(2, 2) = cos(ve[0]);

  Ry(0, 0) = cos(ve[1]);
  Ry(0, 1) = 0;
  Ry(0, 2) = sin(ve[1]);
  Ry(1, 0) = 0;
  Ry(1, 1) = 1;
  Ry(1, 2) = 0;
  Ry(2, 0) = -sin(ve[1]);
  Ry(2, 1) = 0;
  Ry(2, 2) = cos(ve[1]);

  Rz(0, 0) = cos(ve[2]);
  Rz(0, 1) = -sin(ve[2]);
  Rz(0, 2) = 0;
  Rz(1, 0) = sin(ve[2]);
  Rz(1, 1) = cos(ve[2]);
  Rz(1, 2) = 0;
  Rz(2, 0) = 0;
  Rz(2, 1) = 0;
  Rz(2, 2) = 1;

  mR = Rz * Ry * Rx;
}

void rotationMatrixToEuler(const Matrix3d& mR, Vector3d& ve) {
  Real rotXangle = atan2(-mR.row(1).z(), mR.row(2).z());
  Real cosYangle = sqrt(pow(mR.row(0).x(), 2) + pow(mR.row(0).y(), 2));
  Real rotYangle = atan2(mR.row(0).z(), cosYangle);
  Real sinXangle = sin(rotXangle);
  Real cosXangle = cos(rotXangle);
  Real rotZangle = atan2(cosXangle * mR.row(1).x() + sinXangle * mR.row(2).x(),
                         cosXangle * mR.row(1).y() + sinXangle * mR.row(2).y());
  ve = Vector3d(rotXangle, rotYangle, rotZangle);
}

Matrix3d rotationAxisAngleToMatrix(const Vector3d& vaa) {
  Matrix3d mR;
  rotationAxisAngleToMatrix(vaa, mR);
  return mR;
}

Vector3d rotationMatrixToAxisAngle(const Matrix3d& mR) {
  Vector3d vaa;
  rotationMatrixToAxisAngle(mR, vaa);
  return vaa;
}

void rotationAxisAngleToMatrix(const Vector3d& vaa, Matrix3d& mR) {
  Real theta = vaa.norm();

  if (theta <= 1e-9) {
    // Special case for norm 0

    mR = Matrix3d::Identity();
  } else {
    // Rodrigues rotation formula

    Vector3d vaxis = vaa.normalized();
    Matrix3d mK = crossMatrix(vaxis);
    mR = Matrix3d::Identity() + sin(theta) * mK + (1 - cos(theta)) * mK * mK;
  }
}

void rotationMatrixToAxisAngle(const Matrix3d& mR, Vector3d& vaa) {
  Matrix3d mRr = mR - mR.transpose();
  double temp = sqrt(mRr(2, 1) * mRr(2, 1) + mRr(0, 2) * mRr(0, 2) +
                     mRr(1, 0) * mRr(1, 0));
  if (temp < 1e-15)  /// changed by christos, it was -6 the tolerance
  {
    vaa = Vector3d::Zero();
  } else {
    vaa.x() = mRr(2, 1) / temp;
    vaa.y() = mRr(0, 2) / temp;
    vaa.z() = mRr(1, 0) / temp;
    vaa.normalize();
    vaa *= acos((mR(0, 0) + mR(1, 1) + mR(2, 2) - 1) / 2);
  }
}

void computeBestAxisAligment(const MatrixXd& mCloud0,
                             MatrixXd& mR,
                             VectorXd& vt) {
  MatrixXd mU;
  MatrixXd mV;
  VectorXd vs;
  computePCA(mCloud0, mU, mV, vs);
  mR = mU.inverse();
  vt = -mR * (mCloud0.colwise().mean()).transpose();
}

void computeBestRigidTransform(const MatrixXd& mCloud0,
                               const MatrixXd& mCloud1,
                               MatrixXd& mR,
                               VectorXd& vt) {
  assert(mCloud0.rows() == mCloud1.rows());
  assert(mCloud0.cols() == mCloud1.cols());

  int N = mCloud0.rows();
  int M = mCloud0.cols();

  // Compute centroids

  VectorXd vc0 = mCloud0.row(0);
  VectorXd vc1 = mCloud1.row(0);
  for (int i = 1; i < N; ++i) {
    vc0 += mCloud0.row(i);
    vc1 += mCloud1.row(i);
  }
  vc0 /= (Real)N;
  vc1 /= (Real)N;

  MatrixXd mVectors0 = mCloud0.rowwise() - vc0.transpose();
  MatrixXd mVectors1 = mCloud1.rowwise() - vc1.transpose();
  MatrixXd mS = mVectors0.transpose() * mVectors1;

  JacobiSVD<MatrixXd> svd(mS, ComputeFullU | ComputeFullV);
  const MatrixXd& mU = svd.matrixU();
  const MatrixXd& mV = svd.matrixV();
  mR = (mU * mV.transpose());
  if (mR.determinant() < 0) {
    mR -= mU.col(2) * (mV.transpose().row(2) * 2);
  }

  vt = vc1 - mR * vc0;
}

Real distanceToEdge(const Vector3d& vP, const Vector3d& vD) {
  return sqrt(sqrDistanceToEdge(vP, vD));
}

Real distanceToPlane(const Vector3d& vP, const Vector3d& vN) {
  return vP.dot(vN);
}

Real distanceToUnnormalizedPlane(const Vector3d& vP, const Vector3d& vN) {
  return vP.dot(vN) / vN.norm();
}

Real distanceToTriangle(const Vector3d& vP,
                        const Vector3d& vA,
                        const Vector3d& vB,
                        const Vector3d& vC) {
  return sqrt(sqrDistanceToTriangle(vP, vA, vB, vC));
}

Real sqrDistanceToEdge(const Vector3d& vP, const Vector3d& vD) {
  return (vD * saturate(vD.dot(vP) / vD.squaredNorm()) - vP).squaredNorm();
}

Real sqrDistanceToPlane(const Vector3d& vP, const Vector3d& vN) {
  float PdN = vP.dot(vN);
  return PdN * PdN;
}

Real sqrDistanceToUnnormalizedPlane(const Vector3d& vP, const Vector3d& vN) {
  float PdN = vP.dot(vN);
  return PdN * PdN / vN.squaredNorm();
}

Real sqrDistanceToTriangle(const Vector3d& vP,
                           const Vector3d& vA,
                           const Vector3d& vB,
                           const Vector3d& vC) {
  Vector3d vBA = vB - vA;
  Vector3d vPA = vP - vA;
  Vector3d vCB = vC - vB;
  Vector3d vPB = vP - vB;
  Vector3d vAC = vA - vC;
  Vector3d vPC = vP - vC;
  Vector3d vN = vBA.cross(vAC);

  Vector3d vBAxN = vBA.cross(vN);
  Vector3d vCBxN = vCB.cross(vN);
  Vector3d vACxN = vAC.cross(vN);

  bool isOutsideOfTriangle =
      (sign(vBAxN.dot(vPA)) + sign(vCBxN.dot(vPB)) + sign(vACxN.dot(vPC))) < 2;

  Real D =
      isOutsideOfTriangle
          ? min(min(sqrDistanceToEdge(vPA, vBA), sqrDistanceToEdge(vPB, vCB)),
                sqrDistanceToEdge(vPC, vAC))
          : sqrDistanceToUnnormalizedPlane(vPA, vN);

  return D;
}

Real determineTriangleDistanceSign(const Vector3d& vP,
                                   const Vector3d& vA,
                                   const Vector3d& vB,
                                   const Vector3d& vC,
                                   const Vector3d& vNA,
                                   const Vector3d& vNB,
                                   const Vector3d& vNC,
                                   const Vector3d& vNAB,
                                   const Vector3d& vNBC,
                                   const Vector3d& vNCA) {
  Vector3d vAB = vB - vA;
  Vector3d vAP = vP - vA;
  Vector3d vBC = vC - vB;
  Vector3d vBP = vP - vB;
  Vector3d vCA = vA - vC;
  Vector3d vCP = vP - vC;
  Vector3d vN = -vAB.cross(vCA);

  Real AdB = vN.dot(vAP.cross(vBP));
  Real BdC = vN.dot(vBP.cross(vCP));
  Real CdA = vN.dot(vCP.cross(vAP));

  // Test for vertex regions.
  if (vAB.dot(vAP) <= 0.0 && vCA.dot(vAP) >= 0.0)
    return looseSignP(distanceToPlane(vAP, vNA));  // Region of A.
  else if (vBC.dot(vBP) <= 0.0 && vAB.dot(vBP) >= 0.0)
    return looseSignP(distanceToPlane(vBP, vNB));  // Region of B.
  else if (vCA.dot(vCP) <= 0.0 && vBC.dot(vCP) >= 0.0)
    return looseSignP(distanceToPlane(vCP, vNC));  // Region of C.

  // Test for edge regions.
  else if (AdB <= 0.0 && vAB.dot(vAP) >= 0.0 && vAB.dot(vBP) <= 0.0)
    return looseSignP(distanceToPlane(vAP, vNAB));  // Region of AB.
  else if (BdC <= 0.0 && vBC.dot(vBP) >= 0.0 && vBC.dot(vCP) <= 0.0)
    return looseSignP(distanceToPlane(vBP, vNBC));  // Region of BC.
  else if (CdA <= 0.0 && vCA.dot(vCP) >= 0.0 && vAB.dot(vCP) <= 0.0)
    return looseSignP(distanceToPlane(vCP, vNCA));  // Region of CA.

  // The point lies inside of the triangle.
  else
    return looseSignP(distanceToPlane(vAP, vN));
}

void computeBoundingBox(MatrixXd& mV, Vector3d& vmin, Vector3d& vmax) {
  vmin = Vector3d::Ones() * HUGE_VAL;
  vmax = -Vector3d::Ones() * HUGE_VAL;

  for (int i = 0; i < mV.rows(); ++i) {
    if (mV.row(i).x() < vmin.x())
      vmin.x() = mV.row(i).x();

    if (mV.row(i).y() < vmin.y())
      vmin.y() = mV.row(i).y();

    if (mV.row(i).z() < vmin.z())
      vmin.z() = mV.row(i).z();

    if (mV.row(i).x() > vmax.x())
      vmax.x() = mV.row(i).x();

    if (mV.row(i).y() > vmax.y())
      vmax.y() = mV.row(i).y();

    if (mV.row(i).z() > vmax.z())
      vmax.z() = mV.row(i).z();
  }
}

Frame3d parallelTransport(const Frame3d& F, const Vector3d& t2) {
  Frame3d Fin = F;

  Frame3d Fout;
  Fout.tan = t2.normalized();
  Fout.nor = parallelTransport(F.nor, F.tan, Fout.tan);
  Fout.bin = Fout.tan.cross(Fout.nor);

  return Fout;
}

Vector3d parallelTransport(const Vector3d& u,
                           const Vector3d& t1,
                           const Vector3d& t2) {
  double d = t1.dot(t2);

  Vector3d b = t1.cross(t2);

  return u * d + b * (b.dot(u)) * 1 / (1 + d) + b.cross(u);
};

Vector3d parallelTransportNormalized(const Vector3d& u,
                                     const Vector3d& t1,
                                     const Vector3d& t2) {
  Vector3d te = t1.normalized();
  Vector3d tf = t2.normalized();

  double d = te.dot(tf);

  Vector3d b = te.cross(tf);

  return u * d + b * (b.dot(u)) * 1 / (1 + d) + b.cross(u);
};

Vector3d computeCurvatureBinormal(const Vector3d& e0, const Vector3d& e1) {
  Vector3d t0 = e0.normalized();
  Vector3d t1 = e1.normalized();
  return t0.cross(t1) * 2 * (1 / (1 + t0.dot(t1)));
}

void orthonormalizeFrame(Frame3d& F) {
  F.tan.normalize();
  F.bin = F.tan.cross(F.nor).normalized();
  F.nor = F.bin.cross(F.tan).normalized();
}

Frame3d rotateFrame(const Frame3d& frame, Matrix3d& R) {
  Frame3d newFrame;
  newFrame.bin = R * frame.bin;
  newFrame.nor = R * frame.nor;
  newFrame.tan = R * frame.tan;
  return newFrame;
}

Frame3d rotateFrame(const Frame3d& vF, Real angle) {
  Frame3d rot;
  rot.tan = vF.tan;
  Vector3d nor0 = vF.nor;
  Vector3d bin0 = vF.bin;
  rodriguesRotation(rot.tan.data(), angle, nor0.data(), rot.nor.data());
  rodriguesRotation(rot.tan.data(), angle, bin0.data(), rot.bin.data());

  return rot;
}

void rotateFrame(const vector<Frame3d>& vF,
                 const dVector& va,
                 vector<Frame3d>& vFRot) {
  int ne = (int)vF.size();
  assert(ne == va.size());
  assert(ne == vFRot.size());
  for (int i = 0; i < ne; i++) {
    vFRot[i] = rotateFrame(vF[i], va[i]);
  }
}

bool lineLineIntersect(const dVector& p1,
                       const dVector& q1,
                       const dVector& p2,
                       const dVector& q2,
                       double& s,
                       double& t,
                       double tol) {
  double EPS1 = 1e-6;
  double EPS2 = tol;

  double d1[3], d2[3], r[3], a, e, f;
  double c1[3], c2[3];

  d1[0] = q1[0] - p1[0];
  d1[1] = q1[1] - p1[1];
  d1[2] = q1[2] - p1[2];

  d2[0] = q2[0] - p2[0];
  d2[1] = q2[1] - p2[1];
  d2[2] = q2[2] - p2[2];

  r[0] = p1[0] - p2[0];
  r[1] = p1[1] - p2[1];
  r[2] = p1[2] - p2[2];

  a = d1[0] * d1[0] + d1[1] * d1[1] + d1[2] * d1[2];
  e = d2[0] * d2[0] + d2[1] * d2[1] + d2[2] * d2[2];
  f = d2[0] * r[0] + d2[1] * r[1] + d2[2] * r[2];

  // Check if either or both segments degenerate into points
  //
  if ((a <= EPS1) && (e <= EPS1)) {
    s = t = 0.0f;
    c1[0] = p1[0];
    c1[1] = p1[1];
    c1[2] = p1[2];
    c2[0] = p2[0];
    c2[1] = p2[1];
    c2[2] = p2[2];
    return (((c1[0] - c2[0]) * (c1[0] - c2[0]) +
             (c1[1] - c2[1]) * (c1[1] - c2[1]) +
             (c1[2] - c2[2]) * (c1[2] - c2[2]))) < EPS1;
  }

  if (a <= EPS1) {
    // First segment degenerates into a point
    //
    s = 0.0f;
    t = f / e;
    if (t < 0.0f)
      t = 0.0f;
    if (t > 1.0f)
      t = 1.0f;
  } else {
    double c = d1[0] * r[0] + d1[1] * r[1] + d1[2] * r[2];

    if (e <= EPS1) {
      // Second segment degenerates into a point
      //
      t = 0.0f;
      s = -c / a;
      if (s < 0.0f)
        s = 0.0f;
      if (s > 1.0f)
        s = 1.0f;
    } else {
      // Nondegenerate case
      //
      double b = d1[0] * d2[0] + d1[1] * d2[1] + d1[2] * d2[2];
      double denom = a * e - b * b;

      if (denom != 0.0f) {
        s = (b * f - c * e) / denom;
        if (s < 0.0f)
          s = 0.0f;
        if (s > 1.0f)
          s = 1.0f;
      } else
        s = 0.0f;

      double tnom = b * s + f;
      if (tnom < 0.0f) {
        t = 0.0f;
        s = -c / a;
        if (s < 0.0f)
          s = 0.0f;
        if (s > 1.0f)
          s = 1.0f;
      } else if (tnom > e) {
        t = 1.0f;
        s = (b - c) / a;
        if (s < 0.0f)
          s = 0.0f;
        if (s > 1.0f)
          s = 1.0f;
      } else
        t = tnom / e;
    }
  }

  c1[0] = p1[0] + d1[0] * s;
  c1[1] = p1[1] + d1[1] * s;
  c1[2] = p1[2] + d1[2] * s;

  c2[0] = p2[0] + d2[0] * t;
  c2[1] = p2[1] + d2[1] * t;
  c2[2] = p2[2] + d2[2] * t;

  double dist = sqrt((c1[0] - c2[0]) * (c1[0] - c2[0]) +
                     (c1[1] - c2[1]) * (c1[1] - c2[1]) +
                     (c1[2] - c2[2]) * (c1[2] - c2[2]));

  return dist < EPS2 && s >= 0.0 && s <= 1.0 && t >= 0.0 && t <= 1.0;
}

}  // namespace GeometryUtils

}  // namespace PhySim
