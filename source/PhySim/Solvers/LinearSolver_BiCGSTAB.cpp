//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Solvers/LinearSolver_BiCGSTAB.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

LinearSolver_BiCGSTAB::LinearSolver_BiCGSTAB() : LinearSolver() {
  // Nothing to do here...
}

LinearSolver_BiCGSTAB::LinearSolver_BiCGSTAB(const MatrixSd& mA,
                                             const LinearSolverOptions& options)
    : LinearSolver(mA, options) {
  this->Init(mA, options);
}

LinearSolver_BiCGSTAB::~LinearSolver_BiCGSTAB() {
  // Nothing to do here...
}

void LinearSolver_BiCGSTAB::Init(const MatrixSd& mA,
                                 const LinearSolverOptions& options) {
  LinearSolver::Init(mA, options);

  MatrixSd mAFull = mA.selfadjointView<Lower>();

  this->m_solver.setTolerance(options.maxError);
  this->m_solver.setMaxIterations(options.maxIters);
  this->m_solver.analyzePattern(mAFull);

  if (m_solver.info() != ComputationInfo::Success) {
    IOUtils::logTrace(
        Verbosity::V1_Default,
        "\n[WARNING] Linear solve: error during preconditioner analysis");
  }
}

void LinearSolver_BiCGSTAB::FreeInternal() {
  // Nothing to do here...
}

LSResult LinearSolver_BiCGSTAB::SolveInternal(MatrixSd& mA,
                                              const VectorXd& vb,
                                              VectorXd& vx) {
  MatrixSd mAFull = mA.selfadjointView<Lower>();

  Index N = mAFull.rows();
  Index M = mAFull.cols();

  Real normA;
  Real sinThres;
  Real regThres;
  this->GetMatrixData(mAFull, normA, sinThres, regThres);

  VectorXd vxPrev = vx;

  // Solve the specified definite positive linear system.
  // Regularize the matrix if needed to create a DP system.

  int i = 0;
  for (i = 0; i < this->m_options.regIters; ++i) {
    if (i != 0) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[INFO] Regularizing system, iteration %u", i);

      mAFull += m_mR * regThres * pow(10, i - 1);
    }

    // Pattern analyzed

    m_solver.factorize(mAFull);

    // Check computation

    if (m_solver.info() != ComputationInfo::Success) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[FAILURE] Linear solve: error during preconditioner "
                        "factorization");
      continue;  // Iterate again
    }

    vx = m_solver.solveWithGuess(vb, vxPrev);

    // Check calculation

    if (m_solver.info() != ComputationInfo::Success) {
      IOUtils::logTrace(
          Verbosity::V1_Default,
          "\n[WARNING] Linear solve: it was impossible to solve accurately");
    }

    //// Check indefiniteness

    // double dot = vb.dot(vx);

    // if (this->m_options.regSign > 0 && dot < 0.0)
    //{
    //	logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: indefinite
    // matrix, dot: %.6e", dot); 	continue;
    //}
    // if (this->m_options.regSign < 0 && dot > 0.0)
    //{
    //	logTrace(Verbosity::V1_Default, "[FAILURE] Linear solve: indefinite
    // matrix, dot: %.6e", dot); 	continue;
    //}

    //// Check exact solution

    // VectorXd vbTest = mA.selfadjointView<Lower>()*vx;
    // double absError = (vb - vbTest).norm();
    // double relError = absError / vb.norm();
    // if (absError > this->m_options.maxError && relError >
    // this->m_options.maxError)
    //{
    //	logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: inexact
    // solution, error: %.6e", relError); 	continue; // Iterate again
    //}

    IOUtils::logTrace(Verbosity::V1_Default,
                      "\n[SUCCESS] Linear solve: solved using BiCGSTAB. Reg: "
                      "%d, Iter: %d, Error: %f",
                      i, m_solver.iterations(), m_solver.error());

    return LSResult::SUCCESS;
  }

  return LSResult::FAILURE;
}

LSResult LinearSolver_BiCGSTAB::SolveInternal(MatrixSd& mA,
                                              const MatrixXd& mB,
                                              MatrixXd& mX) {
  throw new exception("Not implemented");

  return LSResult::FAILURE;
}

LSResult LinearSolver_BiCGSTAB::SolveInternal(MatrixSd& mA,
                                              const MatrixSd& mB,
                                              MatrixSd& mX) {
  throw new exception("Not implemented");

  return LSResult::FAILURE;
}

}  // namespace PhySim
