//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#ifdef USE_KNITRO

#include <PhySim/Solvers/OptimSolver_Knitro.h>

#include "KTRSolver.h"

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	OptimSolver_Knitro::OptimSolver_Knitro(IOptimProblem* pProblem)
	{
		this->m_isInitialized = false;

		this->Init(pProblem);
	}

	OptimSolver_Knitro::~OptimSolver_Knitro()
	{
		// Nothing to do here...
	}

	void OptimSolver_Knitro::Init(IOptimProblem* pProblem)
	{
		this->m_pProblem = pProblem;
		this->m_isInitialized = true;
	}

	int OptimSolver_Knitro::FunctionCallback(
		const int evalRequestCode,
		const int n,
		const int m,
		const int nnzJ,
		const int nnzH,
		const double* const x,
		const double* const l,
		double* const obj,
		double* const vc,
		double* const vg,
		double* const mJ,
		double* const mH,
		double* const vH,
		void* userParams)
	{
		OptimSolver_Knitro* pSolver = static_cast<OptimSolver_Knitro*>(userParams);

		IOptimProblem* pProblem = pSolver->Problem();

		int N = pProblem->GetNumVariables();
		int M = pProblem->GetNumConstraints();

		Eigen::VectorXd vx = Eigen::Map<const Eigen::VectorXd>(x, N);

		if (evalRequestCode == KTR_RC_EVALFC)
		{
			pProblem->PrePerformStep();

			pProblem->SetVariables(vx);

			pProblem->PrePerformStep();

			if (!pProblem->GetEnergy(*obj))
				return -1; // Invalid compute
		}
		else if (evalRequestCode == KTR_RC_EVALGA)
		{
			pProblem->SetVariables(vx);

			VectorXd vgInt;
			if (!pProblem->GetGradient(vgInt))
				return -1; // Invalid compute
			
			for (int i = 0; i < N; ++i)
				vg[i] = vgInt[i]; // Copy
		}
		else if (evalRequestCode == KTR_RC_EVALH)
		{
			pProblem->SetVariables(vx);

			// Get new HessianFull

			MatrixSd mHInt;
			VectorTd vHInt;
			if (!pProblem->GetHessian(mHInt))
				return -1;  // Invalid compute

			MatrixSd mHtInt = mHInt.transpose();
			eigenSparseMatrixToTriplets(mHtInt, vHInt);

			int countIdx = 0;
			for (size_t i = 0; i < vHInt.size(); ++i)
				if (vHInt[i].col() >= vHInt[i].row())
					mH[countIdx++] = vHInt[i].value();
		}
		else
		{
			return -1;
		}

		return 0;
	}

	//int OptimSolver_Knitro::NewPointCallback(
	//	KTR_context_ptr kc,
	//	const int n,
	//	const int m,
	//	const int nnzJ,
	//	const double* const x,
	//	const double* const l,
	//	const double obj,
	//	const double* const vc,
	//	const double* const vg,
	//	const double* const mJ,
	//	void* userParams)
	//{
	//	OptimSolver_Knitro* pSolver = reinterpret_cast<OptimSolver_Knitro*>(userParams);

	//	pSolver->OnStepFinished();

	//	pSolver->Problem()->OnIterFinish();
	//	pSolver->Problem()->OnIterStart();

	//	return 0;
	//}

	SolveResult OptimSolver_Knitro::SolveFull()
	{
		if (!this->m_isInitialized)
			return SolveResult::Failure;

		KTR_context *kc;
		
		kc = KTR_new();
		
		if (kc == NULL)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to create Knitro context");
			return SolveResult::Failure;  
		}
		
		// Setting options
		
		if (KTR_set_int_param_by_name(kc, "algorithm", 1) != 0)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to setup algorithm");
			return SolveResult::Failure;
		}	
		
		if (KTR_set_int_param_by_name(kc, "presolve", KTR_PRESOLVE_NONE) != 0)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to setup presolving");
			return SolveResult::Failure;
		}

		if (KTR_set_int_param_by_name(kc, "maxit", 100) != 0)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to setup iterations");
			return SolveResult::Failure;
		}

		if (KTR_set_int_param_by_name(kc, "linsolver", KTR_LINSOLVER_MA57) != 0)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to setup linear solver");
			return SolveResult::Failure;
		}

		if (KTR_set_int_param_by_name(kc, "gradopt", KTR_GRADOPT_EXACT) != 0)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to setup gradient type");
			return SolveResult::Failure;
		}

		if (KTR_set_int_param_by_name(kc, "hessopt", KTR_HESSOPT_EXACT) != 0)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to setup Hessian type");
			return SolveResult::Failure;
		}

		if (KTR_set_int_param_by_name(kc, "outlev", 4) != 0)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to setup output level");
			return SolveResult::Failure;
		}

		if (KTR_set_int_param_by_name(kc, "derivcheck", 0) != 0)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to setup derivative check");
			return SolveResult::Failure;
		}

		if (KTR_set_int_param_by_name(kc, "outmode", KTR_OUTMODE_FILE) != 0)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to setup output mode");
			return SolveResult::Failure;
		}
		
		// Register callbacks
		
		if (KTR_set_func_callback(kc, &OptimSolver_Knitro::FunctionCallback) != 0)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to setup energy callback");
			return SolveResult::Failure;
		}

		if (KTR_set_grad_callback(kc, &OptimSolver_Knitro::FunctionCallback) != 0)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to setup gradient callback");
			return SolveResult::Failure;
		}

		if (KTR_set_hess_callback(kc, &OptimSolver_Knitro::FunctionCallback) != 0)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to setup Hessian callback");
			return SolveResult::Failure;
		}

		//if (KTR_set_newpt_callback(kc, &OptimSolver_Knitro::NewPointCallback) != 0)
		//{
		//	logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to setup Hessian callback");
		//	return SolveResult::Failure;
		//}
		
		// Init problem

		int N = this->m_pProblem->GetNumVariables();
		int M = this->m_pProblem->GetNumConstraints();

		// Variable bounds

		VectorXd vlb;
		VectorXd vub;
		m_pProblem->GetLowerBound(vlb);
		m_pProblem->GetUpperBound(vub);
		for (int i = 0; i < this->m_pProblem->GetNumVariables(); ++i)
		{
			if (vlb[i] == -HUGE_VAL)
				vlb[i] = -KTR_INFBOUND;

			if (vub[i] == +HUGE_VAL)
				vub[i] = KTR_INFBOUND;
		}

		// Matrix sparsity

		MatrixSd mJ;
		MatrixSd mH;
		this->m_pProblem->GetJacobian(mJ);
		this->m_pProblem->GetHessian(mH);
		VectorTd vJ;
		VectorTd vHt;
		MatrixSd mHt = mH.transpose();
		eigenSparseMatrixToTriplets(mJ, vJ);
		eigenSparseMatrixToTriplets(mHt, vHt);

		iVector vrowsJ;
		iVector vcolsJ;
		iVector vrowsH;
		iVector vcolsH;
		vrowsJ.reserve(vJ.size());
		vcolsJ.reserve(vJ.size());
		vrowsH.reserve(vHt.size());
		vcolsH.reserve(vHt.size());
		for (size_t i = 0; i < vJ.size(); ++i)
		{
			vrowsJ[i] = vJ[i].row();
			vcolsJ[i] = vJ[i].col();
		}
		for (size_t i = 0; i < vHt.size(); ++i)
		{
			if (vHt[i].col() >= vHt[i].row())
			{
				vrowsH.push_back(vHt[i].row());
				vcolsH.push_back(vHt[i].col());
			}
		}

		int nnzJ = (int) vrowsJ.size();
		int nnzH = (int) vrowsH.size();

		this->m_pProblem->OnSolveStart();
		this->m_pProblem->OnIterStart();

		// Initial solution

		VectorXd v0;
		m_pProblem->GetVariables(v0);


		if (KTR_init_problem(kc, 
			N, KTR_OBJGOAL_MINIMIZE, KTR_OBJTYPE_GENERAL, vlb.data(), vub.data(), // Objective
			M, NULL, NULL, NULL, nnzJ, vrowsJ.data(), vcolsJ.data(), // Constraints
			nnzH, vrowsH.data(), vcolsH.data(), v0.data(), // HessianFull
			NULL) != 0)
		{
			logTrace(Verbosity::V1_Default, "\n[ERROR] Knitro solve: unable to initialize ");
			return SolveResult::Failure;
		}

		VectorXd vx(N);
		VectorXd vl(N);
		Real objective;

		if (KTR_solve(kc, vx.data(), vl.data(), 0, &objective, NULL, NULL, NULL, NULL, NULL, m_pProblem) != 0)
		{
			this->m_pProblem->OnSolveFinish();

			logTrace(Verbosity::V1_Default, "\n[FAILURE] Knitro solve: unable to solve problem");

			return SolveResult::Failure;
		}
		else
		{
			this->m_pProblem->OnSolveFinish();

			logTrace(Verbosity::V1_Default, "\n[SUCCESS] Knitro solve: optimization problem solved");

			return SolveResult::Success;
		}
	}

	//SolveResult OptimSolver_Knitro::SolveFull()
	//{
	//	if (!this->m_isInitialized)
	//		return SolveResult::Failure; 

	//	//KTR_context *kc;

	//	//kc = KTR_new();

	//	//KTR_set_int_param_by_name(kc, "outmode", KTR_OUTMODE_FILE);

	//	SolveResult fullResult;

	//	// Initialize

	//	logTrace(Verbosity::V1_Default, "\n[TRACE] Creating Knitro solver");

	//	knitro::KTRSolver* pSolver;

	//	try
	//	{
	//		pSolver = new knitro::KTRSolver(this->m_pProblem,
	//										KTR_GRADOPT_EXACT,
	//										KTR_HESSOPT_EXACT);

	//	}
	//	catch (knitro::KTRException e)
	//	{
	//		logTrace(Verbosity::V1_Default, "\n[ERROR] Impossible to create Knitro solver instance: %", e.message().c_str());

	//		return SolveResult::Failure;
	//	};

	//	pSolver->useNewptCallback();
	//	pSolver->loadParamFile(m_config);
	//	pSolver->setParam("outmode", KTR_OUTMODE_FILE);
	//	pSolver->setParam("derivcheck", KTR_DERIVCHECK_SECOND);
	//	pSolver->setParam("derivcheck_type", KTR_DERIVCHECK_CENTRAL);

	//	logTrace(Verbosity::V1_Default, "\n[TRACE] Starting Knitro optimization");
	//	logTrace(Verbosity::V1_Default, "\n--");
	//	logTrace(Verbosity::V1_Default, "\n");

	//	int solveStatus = 0;

	//	try
	//	{
	//		solveStatus = pSolver->solve();

	//	}
	//	catch (exception e)
	//	{
	//		logTrace(Verbosity::V1_Default, "\n[ERROR] Impossible to solve Knitro solver instance: %", e.what());

	//		return SolveResult::Failure;
	//	};

	//	logTrace(Verbosity::V1_Default, "\n");
	//	logTrace(Verbosity::V1_Default, "\n--");
	//	logTrace(Verbosity::V1_Default, "\n[TRACE] Finished Knitro optimization");

	//	if (solveStatus == 0)
	//	{
	//		fullResult = SolveResult::Success;
	//	}
	//	else
	//	{
	//		fullResult = SolveResult::Failure;
	//	}

	//	delete pSolver;

	//	return fullResult;
	//}

	SolveResult OptimSolver_Knitro::SolveStep()
	{
		if (!this->m_isInitialized)
			return SolveResult::Failure;

		SolveResult stepResult;

		throw new exception("Not implemented");

		return stepResult;
	}

}

#endif