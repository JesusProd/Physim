//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Solvers/LinearSolver.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

LinearSolver::LinearSolver() {
  this->m_isInitialized = false;
}

LinearSolver::LinearSolver(const MatrixSd& mA,
                           const LinearSolverOptions& options) {
  this->Init(mA, options);
}

LinearSolver::~LinearSolver() {
  // Nothing to do here...
}

void LinearSolver::Init(const MatrixSd& mA,
                        const LinearSolverOptions& options) {
  this->FreeInternal();

  this->m_options = options;

  this->m_timerLinearSolve = CustomTimer(10, "LIN_SOL", "");

  // Create regularization

  Index N = mA.rows();
  m_mR = MatrixSd(N, N);
  VectorTd vT(N);
  for (int i = 0; i < N; ++i)
    if (this->m_options.regSign < 0.0)
      vT[i] = Triplet<Real>(i, i, -1.0);
    else
      vT[i] = Triplet<Real>(i, i, 1.0);
  m_mR.setFromTriplets(vT.begin(), vT.end());
  m_mR.makeCompressed();

  this->m_isInitialized = true;
}

void LinearSolver::LinearSolver::FreeInternal() {
  this->m_isInitialized = true;
}

void LinearSolver::GetMatrixData(const MatrixSd& mA,
                                 Real& normA,
                                 Real& sinT,
                                 Real& regT) {
  normA = mA.norm();
  // sinT = min(1e-6, max(1e-6, 1e-6*normA));
  // regT = min(1e-3, max(1e-3, 1e-3*normA));
  sinT = max(min(1e-6 * normA, 1e-6), 1e-9);
  regT = 1e-6 * normA;
}

LSResult LinearSolver::Solve(MatrixSd& mA, const VectorXd& vb, VectorXd& vx) {
  if (!this->m_isInitialized)
    this->Init(mA, m_options);

  LSResult solution;

  if (m_options.profileTime)
    this->m_timerLinearSolve.Resume();

  solution = this->SolveInternal(mA, vb, vx);

  if (m_options.profileTime)
    this->m_timerLinearSolve.StopStoreLog();

  return solution;
}

LSResult LinearSolver::Solve(MatrixSd& mA, const MatrixXd& mB, MatrixXd& mX) {
  if (!this->m_isInitialized)
    this->Init(mA, m_options);

  LSResult solution;

  if (m_options.profileTime)
    this->m_timerLinearSolve.Resume();

  solution = this->SolveInternal(mA, mB, mX);

  if (m_options.profileTime)
    this->m_timerLinearSolve.StopStoreLog();

  return solution;
}

LSResult LinearSolver::Solve(MatrixSd& mA, const MatrixSd& mB, MatrixSd& mX) {
  if (!this->m_isInitialized)
    this->Init(mA, m_options);

  LSResult solution;

  if (m_options.profileTime)
    this->m_timerLinearSolve.Resume();

  solution = this->SolveInternal(mA, mB, mX);

  if (m_options.profileTime)
    this->m_timerLinearSolve.StopStoreLog();

  return solution;
}

}  // namespace PhySim
