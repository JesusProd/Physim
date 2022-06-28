//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Utils/Utils.h>

#include <PhySim/Kinematics/KinematicsEle.h>
#include <PhySim/Physics/Simulables/Simulable.h>
#include <PhySim/Solvers/LinearSolver_EigenLDLT.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

namespace Utils {

void add3D(int i, const Eigen::Vector3d& v, dVector& vd) {
  int offset = 3 * i;
  assert(offset < (int)vd.size());
  for (int i = 0; i < 3; ++i)
    vd[offset + i] += v(i);
}

void set3D(int i, const Eigen::Vector3d& v, dVector& vd) {
  int offset = 3 * i;
  assert(offset < (int)vd.size());
  for (int i = 0; i < 3; ++i)
    vd[offset + i] = v(i);
}

Eigen::Vector3d get3D(int i, const dVector& vd) {
  Eigen::Vector3d v;

  int offset = 3 * i;
  assert(offset < (int)vd.size());
  for (int i = 0; i < 3; ++i)
    v(i) = vd[offset + i];

  return v;
}

void getSubvector(int i, int nele, const dVector& vin, dVector& vout) {
  vout.resize(nele);
  for (int it = 0; it < nele; it++)
    vout[it] = vin[i + it];
}

void setSubvector(int i, int nele, const dVector& vin, dVector& vout) {
  for (int it = 0; it < nele; it++)
    vout[i + it] = vin[it];
}

void getSubvector(int i, int nele, const VectorXd& vin, VectorXd& vout) {
  vout.resize(nele);
  for (int it = 0; it < nele; it++)
    vout(it) = vin(i + it);
}

void setSubvector(int i, int nele, const VectorXd& vin, VectorXd& vout) {
  for (int it = 0; it < nele; it++)
    vout(i + it) = vin(it);
}

dVector toSTL(const Eigen::VectorXd& v) {
  int n = (int)v.size();
  dVector out(n);
  for (int i = 0; i < n; ++i)
    out[i] = v(i);
  return out;
}

iVector toSTL(const Eigen::VectorXi& v) {
  int n = (int)v.size();
  iVector out(n);
  for (int i = 0; i < n; ++i)
    out[i] = v(i);
  return out;
}

dVector toSTL(const Eigen::Vector3d& v) {
  dVector out(3);
  out[0] = v(0);
  out[1] = v(1);
  out[2] = v(2);
  return out;
}

Eigen::VectorXd toEigen(const dVector& v) {
  int n = (int)v.size();
  VectorXd out(n);
  for (int i = 0; i < n; ++i)
    out(i) = v[i];
  return out;
}

Eigen::VectorXi toEigen(const iVector& v) {
  int n = (int)v.size();
  VectorXi out(n);
  for (int i = 0; i < n; ++i)
    out(i) = v[i];
  return out;
}

bool presolveStaticsForFixedDoF(Simulable& simulable,
                                const vector<VectorXd>& vFixedPos) {
  const vector<PtrS<KinematicsEle>>& vDoF = simulable.GetKinematics();

  // TODO: Iterative?

  size_t numDoFSet = vDoF.size();
  int numFree = simulable.GetNumFreeDOF();
  int numFull = simulable.GetNumFullDOF();
  int numFixed = numFull - numFree;

  VectorXd vxFull = VectorXd::Zero(numFree);
  VectorXd dxFull = VectorXd::Zero(numFull);
  simulable.GetDOFVector(vxFull, Tag_Position_X);

  for (size_t i = 0; i < vDoF.size(); ++i) {
    int numDim = vDoF[i]->NumDim();
    int index = vDoF[i]->Index();
    int offset = vDoF[i]->Offset();
    dxFull.block(offset, 0, numDim, 1) =
        vFixedPos[index] - vxFull.segment(offset, numDim);
  }

  // Gather fixed DoF displacement

  const MatrixSd& mFreePerm = simulable.GetFreePermutation();

  AVectorXd vgFull;
  AMatrixSd mHFull;
  simulable.GetGradient(vgFull);
  simulable.GetHessian(mHFull);
  MatrixSd mHFullS = mHFull.selfadjointView<Lower>();
  MatrixSd mHPerm = mFreePerm * mHFullS * mFreePerm.transpose();
  VectorXd vgPerm = mFreePerm * vgFull;
  VectorXd dxPerm = mFreePerm * dxFull;

  // Partition permuted mH, vg and dx

  int offsetUnfix = 0;
  int offsetFixed = simulable.GetNumFreeDOF();
  VectorXd vgU =
      vgPerm.block(offsetUnfix, 0, numFree, 1);  // Gradient at unfixed
  VectorXd dxF =
      dxPerm.block(offsetFixed, 0, numFixed, 1);  // Displacement at fixed
  MatrixSd mA = mHPerm.block(offsetUnfix, offsetUnfix, numFree, numFree);
  MatrixSd mC = mHPerm.block(offsetUnfix, offsetFixed, numFree, numFixed);
  VectorXd vb = -vgU - mC * dxF;

  // Solve the problem

  LinearSolver_EigenLDLT solver(mA, LinearSolverOptions());

  VectorXd vx(vb.size());
  solver.Solve(mA, vb, vx);

  dxPerm.block(0, 0, numFree, 1) = vx;
  vxFull += mFreePerm.transpose() * dxPerm;

  // Set the new positions

  simulable.SetDOFVector(vxFull, Tag_Position_X);

  return true;
}
}  // namespace Utils
}  // namespace PhySim
