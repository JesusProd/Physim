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


#include <PhySim/Solvers/OptimSolver.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class OptimSolver_USQP_LS : public OptimSolver
	{

	public:

		OptimSolver_USQP_LS(IOptimProblem* pProblem, const OptimSolverOptions& options) : OptimSolver(pProblem, options)
		{
			// Nothing to do here...
		}

		virtual const OptimState& SolveStepInternal() override;

	};
}
