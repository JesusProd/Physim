//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Solvers/LinearSolver_EigenCG.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	LinearSolver_EigenCG::LinearSolver_EigenCG() : LinearSolver()
	{
		// Nothing to do here...
	}

	LinearSolver_EigenCG::LinearSolver_EigenCG(const MatrixSd& mA, const LinearSolverOptions& options) : LinearSolver(mA, options)
	{
		this->Init(mA, options);
	}

	LinearSolver_EigenCG::~LinearSolver_EigenCG()
	{
		// Nothing to do here...
	}

	void LinearSolver_EigenCG::Init(const MatrixSd& mA, const LinearSolverOptions& options)
	{
		LinearSolver::Init(mA, options);

		this->m_solver.setTolerance(options.maxError);
		this->m_solver.setMaxIterations(options.maxIters);
		this->m_solver.analyzePattern(mA);

		if (m_solver.info() != LS_SUCCESS)
		{
			IOUtils::logTrace(Verbosity::V1_Default, "\n[WARNING] Linear solve: error during preconditioner analysis");
		}
	}

	void LinearSolver_EigenCG::FreeInternal()
	{
		// Nothing to do here...
	}

	LSResult LinearSolver_EigenCG::SolveInternal(MatrixSd& mA, const VectorXd& vb, VectorXd& vx)
	{
		int N = (int) mA.rows(); 
		int M = (int) mA.cols();

		Real normA;
		Real sinThres;
		Real regThres;
		this->GetMatrixData(mA, normA, sinThres, regThres);

		VectorXd vxPrev = vx;

		// Solve the specified definite positive linear system. 
		// Regularize the matrix if needed to create a DP system.

		int i = 0;
		for (i = 0; i < this->m_options.regIters; ++i)
		{
			if (i != 0)
			{
				IOUtils::logTrace(Verbosity::V1_Default, "\n[INFO] Regularizing system, iteration %u", i);

				mA += m_mR*regThres*pow(10, i - 1);
			}

			// Pattern analyzed

			m_solver.factorize(mA);

			// Check computation

			if (m_solver.info() != LS_SUCCESS)
			{
				IOUtils::logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: error during preconditioner factorization");
				continue; // Iterate again
			}

			vx = m_solver.solveWithGuess(vb, vxPrev);

			// Check calculation

			if (m_solver.info() != LS_SUCCESS)
			{
				IOUtils::logTrace(Verbosity::V1_Default, "\n[WARNING] Linear solve: it was impossible to solve accurately");
			}

			// Check indefiniteness

			double dot = vb.dot(vx);

			if (this->m_options.regSign > 0 && dot < 0.0)
			{
				IOUtils::logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: indefinite matrix, dot: %.6e", dot);
				continue;
			}
			if (this->m_options.regSign < 0 && dot > 0.0)
			{
				IOUtils::logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: indefinite matrix, dot: %.6e", dot);
				continue;
			}

			IOUtils::logTrace(Verbosity::V1_Default, "\n[SUCCESS] Linear solve: solved using Eigen CG. Reg: %d, Iter: %d, Error: %.9e", i, m_solver.iterations(), m_solver.error());

			return LSResult::LS_SUCCESS;
		}

		return LSResult::LS_FAILURE;
	}

	LSResult LinearSolver_EigenCG::SolveInternal(MatrixSd& mA, const MatrixXd& mB, MatrixXd& mX)
	{
		throw new exception("Not implemented");

		return LSResult::LS_FAILURE;
	}

	LSResult LinearSolver_EigenCG::SolveInternal(MatrixSd& mA, const MatrixSd& mB, MatrixSd& mX)
	{
		throw new exception("Not implemented");

		return LSResult::LS_FAILURE;
	}

}
