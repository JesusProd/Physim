//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Solvers/LinearSolver_EigenLDLT.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

LinearSolver_EigenLDLT::LinearSolver_EigenLDLT() : LinearSolver() {
  // Nothing to do here...
}

LinearSolver_EigenLDLT::LinearSolver_EigenLDLT(
    const MatrixSd& mA,
    const LinearSolverOptions& options)
    : LinearSolver(mA, options) {
  this->Init(mA, options);
}

LinearSolver_EigenLDLT::~LinearSolver_EigenLDLT() {
  // Nothing to do here...
}

void LinearSolver_EigenLDLT::Init(const MatrixSd& mA,
                                  const LinearSolverOptions& options) {
  LinearSolver::Init(mA, options);

  this->m_solver.analyzePattern(mA);

  if (m_solver.info() != ComputationInfo::Success) {
    IOUtils::logTrace(Verbosity::V1_Default,
                      "\n[FAILURE] Linear solver: error during analysis");
  }
}

void LinearSolver_EigenLDLT::FreeInternal() {
  // Nothing to do here...
}

LSResult LinearSolver_EigenLDLT::SolveInternal(MatrixSd& mA,
                                               const VectorXd& vb,
                                               VectorXd& vx) {
  int N = (int)mA.rows();
  int M = (int)mA.cols();

  Real normA;
  Real sinThres;
  Real regThres;
  this->GetMatrixData(mA, normA, sinThres, regThres);

  // Solve the specified definite positive linear system.
  // Regularize the matrix if needed to create a DP system.

  int i = 0;
  for (i = 0; i < this->m_options.regIters; ++i) {
    if (i != 0) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[INFO] Regularizing system, iteration %u", i);

      mA += m_mR * regThres * pow(10, i - 1);
    }

    // Pattern analyzed

    IOUtils::logTrace(Verbosity::V1_Default,
                      "\n[INFO] Linear solve: computing...");

    m_solver.factorize(mA);

    // Check computation

    if (m_solver.info() != ComputationInfo::Success) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[FAILURE] Linear solve: error during factorization");
      continue;  // Iterate again
    }

    // Check singularity

    bool jumpSin = false;
    VectorXd vd = m_solver.vectorD();
    for (int j = 0; j < N; ++j) {
      if (fabs(vd(j)) < sinThres) {
        IOUtils::logTrace(
            Verbosity::V1_Default,
            "\n[FAILURE] Linear solve: singular matrix, value: %.9e", vd(j));
        jumpSin = true;
        break;
      }
    }

    if (jumpSin)
      continue;

    vx = m_solver.solve(vb);

    // Check calculation

    if (m_solver.info() != ComputationInfo::Success) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[FAILURE] Linear solve: error during calculation");
      continue;  // Iterate again
    }

    // Check indefiniteness

    double dot = vb.normalized().dot(vx.normalized());

    if (this->m_options.regSign > 0 && dot < -regThres) {
      IOUtils::logTrace(
          Verbosity::V1_Default,
          "\n[FAILURE] Linear solve: indefinite matrix, dot: %.9e", dot);
      continue;
    }
    if (this->m_options.regSign < 0 && dot > +regThres) {
      IOUtils::logTrace(
          Verbosity::V1_Default,
          "\n[FAILURE] Linear solve: indefinite matrix, dot: %.9e", dot);
      continue;
    }

    // Check exact solution

    VectorXd vbTest = mA.selfadjointView<Lower>() * vx;
    double absError = (vb - vbTest).norm();
    double relError = absError / vb.norm();

    if (absError > this->m_options.maxError &&
        relError > this->m_options.maxError) {
      IOUtils::logTrace(
          Verbosity::V1_Default,
          "\n[FAILURE] Linear solve: inexact solution, error: %.9e", relError);
      continue;  // Iterate again
    }

    IOUtils::logTrace(Verbosity::V1_Default,
                      "\n[SUCCESS] Linear solve: solved using Eigen LDLT. Reg: "
                      "%d. Error: %.9e",
                      i, relError);

    return LSResult::SUCCESS;
  }

  return LSResult::FAILURE;
}

LSResult LinearSolver_EigenLDLT::SolveInternal(MatrixSd& mA,
                                               const MatrixXd& mB,
                                               MatrixXd& mX) {
  mX.resize(mB.rows(), mB.cols());
  for (int i = 0; i < mB.cols(); i++) {
    VectorXd x, b;
    x.resize(mB.rows());
    b = mB.block(0, i, mB.rows(), 1);
    this->SolveInternal(mA, b, x);
    mX.block(0, i, mB.rows(), 1) = x;
  }
  return LSResult::SUCCESS;
  // throw exception("Not implemented");

  // return LSResult::FAILURE;
}

LSResult LinearSolver_EigenLDLT::SolveInternal(MatrixSd& mA,
                                               const MatrixSd& mB,
                                               MatrixSd& mX) {
  throw exception("Not implemented");

  return LSResult::FAILURE;
}

}  // namespace PhySim
