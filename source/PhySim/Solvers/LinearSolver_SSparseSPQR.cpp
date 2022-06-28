//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Solvers/LinearSolver_SSparseSPQR.h>

#ifdef USE_SSPARSE

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	LinearSolver_SSparseSPQR::LinearSolver_SSparseSPQR() : LinearSolver()
	{
		// Nothing to do here...
	}

	LinearSolver_SSparseSPQR::LinearSolver_SSparseSPQR(const MatrixSd& mA, const LinearSolverOptions& options) : LinearSolver(mA, options)
	{
		this->Init(mA, options);
	}

	LinearSolver_SSparseSPQR::~LinearSolver_SSparseSPQR()
	{
		// Nothing to do here...
	}

	void LinearSolver_SSparseSPQR::Init(const MatrixSd& mA, const LinearSolverOptions& options)
	{
		LinearSolver::Init(mA, options);

		m_solver.compute(mA);

		if (m_solver.info() != Success)
		{
			logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solver: error during computation");
		}
	}

	void LinearSolver_SSparseSPQR::Free()
	{
		// Nothing to do here...
	}

	SolveResult LinearSolver_SSparseSPQR::SolveInternal(MatrixSd& mA, const VectorXd& vb, VectorXd& vx)
	{
		MatrixSd mAFull = mA.selfadjointView<Lower>();

		int N = (int) mA.rows();
		int M = (int) mA.cols();

		Real normA;
		Real sinThres;
		Real regThres;
		this->GetMatrixData(mA, normA, sinThres, regThres);

		// Solve the specified definite positive linear system. 
		// Regularize the matrix if needed to create a DP system.

		int i = 0;
		for (i = 0; i < this->m_options.regIters; ++i)
		{
			if (i != 0)
			{
				logTrace(Verbosity::V1_Default, "\n[INFO] Regularizing system, iteration %u", i);

				mAFull += m_mR*regThres*pow(10, i - 1);
			}

			// Pattern analyzed

			m_solver.compute(mAFull);

			// Check computation

			if (m_solver.info() != Success)
			{
				logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: error during computation");
				continue; // Iterate again
			}

			//// Check singularity

			//bool jumpSin = false;
			//VectorXd vd = m_solver.vectorD();
			//for (int j = 0; j < N; ++j)
			//{
			//	if (fabs(vd(j)) < sinThres)
			//	{
			//		logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: singular matrix, value: %.6e", vd(j));
			//		jumpSin = true;
			//		break;
			//	}
			//}

			//if (jumpSin)
			//	continue;

			vx = m_solver.solve(vb);

			// Check calculation

			if (m_solver.info() != Success)
			{
				logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: error during calculation");
				continue; // Iterate again
			}

			//// Check indefiniteness

			//double dot = vb.dot(vx);

			//if (this->m_options.regSign > 0 && dot < 0.0)
			//{
			//	logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: indefinite matrix, dot: %.6e", dot);
			//	continue;
			//}
			//if (this->m_options.regSign < 0 && dot > 0.0)
			//{
			//	logTrace(Verbosity::V1_Default, "[FAILURE] Linear solve: indefinite matrix, dot: %.6e", dot);
			//	continue;
			//}

			// Check exact solution

			VectorXd vbTest = mA.selfadjointView<Lower>()*vx;
			double absError = (vb - vbTest).norm();
			double relError = absError / vb.norm();
			if (absError > this->m_options.maxError && relError > this->m_options.maxError)
			{
				logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: inexact solution, error: %.6e", relError);
				continue; // Iterate again
			}

			logTrace(Verbosity::V1_Default, "\n[SUCCESS] Linear solve: solved using SSparse SPQR. Reg: %d", i);

			return SolveResult::Success;
		}

		return SolveResult::Failure;
	}

	SolveResult LinearSolver_SSparseSPQR::SolveInternal(MatrixSd& mA, const MatrixXd& mB, MatrixXd& mX)
	{
		throw new exception("Not implemented");

		return SolveResult::Failure;
	}

	SolveResult LinearSolver_SSparseSPQR::SolveInternal(MatrixSd& mA, const MatrixSd& mB, MatrixSd& mX)
	{
		throw new exception("Not implemented");

		return SolveResult::Failure;
	}


	//SolveResult LinearSolver_SSparseSPQR::Solve_CG(MatrixSd& mA, const VectorXd& vb, VectorXd& vx)
	//{
	//	int N;
	//	Real normA;
	//	MatrixSd mR;
	//	Real sinThres;
	//	Real regThres;
	//	this->PrepareLinearSolve(mA, N, normA, sinThres, regThres, mR);

	//	// Solve the specified definite positive linear system. 
	//	// Regularize the matrix if needed to create a DP system.

	//	int i = 0;
	//	for (i = 0; i < this->options.regIters; ++i)
	//	{
	//		if (i != 0)
	//		{
	//			logTrace(Verbosity::V1_Default, "\n[INFO] Regularizing system, iteration %u", i);

	//			mA += mR*regThres*pow(10, i - 1);
	//		}

	//		ConjugateGradient<MatrixSd> solver;
	//		solver.setTolerance(this->options.maxError);
	//		solver.setMaxIterations(this->options.maxIters);

	//		solver.compute(mA);

	//		// Check computation

	//		if (solver.info() != Success)
	//		{
	//			logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: error during computation");
	//			continue; // Iterate again
	//		}

	//		vx = solver.solve(vb);

	//		// Check calculation

	//		if (solver.info() != Success)
	//		{
	//			logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: error during calculation");
	//			continue; // Iterate again
	//		}

	//		// Check indefiniteness

	//		double dot = vb.dot(vx);

	//		if (this->options.regSign > 0 && dot < 0.0)
	//		{
	//			logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: indefinite matrix, dot: %.6e", dot);
	//			continue;
	//		}
	//		if (this->options.regSign < 0 && dot > 0.0)
	//		{
	//			logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: indefinite matrix, dot: %.6e", dot);
	//			continue;
	//		}

	//		// Check exact solution

	//		VectorXd vbTest = mA.selfadjointView<Lower>()*vx;
	//		double absError = (vb - vbTest).norm();
	//		double relError = absError / vb.norm();
	//		if (absError > this->options.maxError && relError > this->options.maxError)
	//		{
	//			logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: inexact solution, error: %.6e", relError);
	//			continue; // Iterate again
	//		}

	//		logTrace(Verbosity::V1_Default, "\n[SUCCESS] Linear solve: solved using Conjugate GradientFull after %i regularization steps", i);

	//		return SolveResult::Success;
	//	}

	//	return SolveResult::Failure;
	//}

	//SolveResult LinearSolver_SSparseSPQR::Solve_LU(MatrixSd& mA, const VectorXd& vb, VectorXd& vx)
	//{
	//	int N;
	//	Real normA;
	//	MatrixSd mR;
	//	Real sinThres;
	//	Real regThres;
	//	this->PrepareLinearSolve(mA, N, normA, sinThres, regThres, mR);

	//	// Solve the specified definite positive linear system. 
	//	// Regularize the matrix if needed to create a DP system.

	//	int i = 0;
	//	for (i = 0; i < this->options.regIters; ++i)
	//	{
	//		if (i != 0)
	//		{
	//			logTrace(Verbosity::V1_Default, "\n[INFO] Regularizing system, iteration %u", i);

	//			mA += mR*regThres*pow(10, i - 1);
	//		}

	//		SparseLU<MatrixSd> solver;

	//		solver.compute(mA);

	//		// Check factorization

	//		if (solver.info() != Success)
	//		{
	//			logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: error during factorization");
	//			continue; // Iterate again
	//		}

	//		vx = solver.solve(vb);

	//		// Check calculation

	//		if (solver.info() != Success)
	//		{
	//			logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: error during calculation");
	//			continue; // Iterate again
	//		}

	//		// Check indefiniteness

	//		double dot = vb.dot(vx);

	//		if (this->options.regSign > 0 && dot < 0.0)
	//		{
	//			logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: indefinite matrix, dot: %.6e", dot);
	//			continue;
	//		}
	//		if (this->options.regSign < 0 && dot > 0.0)
	//		{
	//			logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: indefinite matrix, dot: %.6e", dot);
	//			continue;
	//		}

	//		// Check exact solution

	//		VectorXd vbTest = mA.selfadjointView<Lower>()*vx;
	//		double absError = (vb - vbTest).norm();
	//		double relError = absError / vb.norm();
	//		if (absError > this->options.maxError && relError > this->options.maxError)
	//		{
	//			logTrace(Verbosity::V1_Default, "\n[FAILURE] Linear solve: inexact solution, error: %.6e", relError);
	//			continue; // Iterate again
	//		}

	//		logTrace(Verbosity::V1_Default, "\n[SUCCESS] Linear solve: solved using LU Factorization after %i regularization steps", i);

	//		return SolveResult::Success;
	//	}

	//	return SolveResult::Failure;
	//}

}

#endif