//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Solvers/OptimSolver.h>

#include <PhySim/Solvers/LinearSolver_CudaSC.h>
#include <PhySim/Solvers/LinearSolver_EigenLDLT.h>
#include <PhySim/Solvers/LinearSolver_EigenCG.h>
#include <PhySim/Solvers/LinearSolver_BiCGSTAB.h>
#include <PhySim/Solvers/LinearSolver_CholmodLDLT.h>
#include <PhySim/Solvers/LinearSolver_SSparseSPQR.h>

#include <PhySim/Utils/IOUtils.h>


namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	OptimSolver::OptimSolver(IOptimProblem* pProblem, const OptimSolverOptions& options)
	{
		this->m_isInitialized = false;

		this->Init(pProblem, options);
	}

	OptimSolver::~OptimSolver()
	{
		if (this->m_pLSolver != NULL)
			delete this->m_pLSolver;
	}

	void OptimSolver::Init(IOptimProblem* pProblem, const OptimSolverOptions& options)
	{
		this->m_pProblem = pProblem;
		this->m_options = options;

		// Initialize state

		this->m_state = OptimState();
		this->m_state.m_N = pProblem->GetNumVariables();
		this->m_state.m_M = pProblem->GetNumConstraints();
		pProblem->GetVariables(this->m_state.m_vx);
		pProblem->GetConstraint(this->m_state.m_vc);
		pProblem->GetEnergy(this->m_state.m_obj);
		pProblem->GetGradient(this->m_state.m_vg);
		pProblem->GetHessian(this->m_state.m_mH);
		this->m_state.m_v0 = this->m_state.m_vx;
		this->m_state.m_opt = this->m_state.m_vg.norm();
		this->m_state.m_fea = this->m_state.m_vc.norm();
		this->m_state.m_countIt = 0;

		// Initialize linear solver

		LinearSolverOptions linearOptions;
		linearOptions.type = options.lsSolverType;
		linearOptions.maxError = options.lsTolMaxError;
		linearOptions.maxIters = options.lsNumMaxIters;

		switch (linearOptions.type)
		{
			case LSSolverType::LS_EigenLDLT: 
				this->m_pLSolver = new LinearSolver_EigenLDLT(this->m_state.m_mH, linearOptions);
				break;
			case LSSolverType::LS_EigenCG:
				this->m_pLSolver = new LinearSolver_EigenCG(this->m_state.m_mH, linearOptions);
				break;
			case LSSolverType::LS_BiCGSTAB:
				this->m_pLSolver = new LinearSolver_BiCGSTAB(this->m_state.m_mH, linearOptions);
				break;
#ifdef USE_SSPARSE
			case LSSolverType::LS_CholmodLDLT:
				this->m_pLSolver = new LinearSolver_CholmodLDLT(mA, linearOptions);
				break;
			case LSSolverType::LS_SSparseSPQR:
				this->m_pLSolver = new LinearSolver_SSparseSPQR(mA, linearOptions);
				break;
#endif
#ifdef USE_CUDA
			case LSSolverType::LS_CUDASC:
				this->m_pLSolver = new LinearSolver_CudaSC(mA, linearOptions);
				break;
#endif
		}

		// Initialize custom timer

		this->m_timerLinearSolve = CustomTimer(10, "OPT_SOL", "");
		this->m_timerComputeStep = CustomTimer(10, "OPT_STE", "");
		this->m_timerLineSearch = CustomTimer(10, "OPT_LiS", "");

		this->m_fullCallback = NULL;
		this->m_stepCallback = NULL;

		this->m_bfgs.m_isInit = false;
		this->m_bfgs.m_isInit = false;

		this->m_isInitialized = true;

		IOUtils::logTrace(Verbosity::V1_Default, "\n--");
		IOUtils::logTrace(Verbosity::V1_Default, "\nINITIALIZED: Solver");
		IOUtils::logTrace(Verbosity::V1_Default, "\nVariable number: %i", this->m_state.m_N);
		IOUtils::logTrace(Verbosity::V1_Default, "\nConstraint number: %i", this->m_state.m_M);
		IOUtils::logTrace(Verbosity::V1_Default, "\nInitial objective: %f", this->m_state.m_obj);
		IOUtils::logTrace(Verbosity::V1_Default, "\nInitial optimality: %f", this->m_state.m_opt);
		IOUtils::logTrace(Verbosity::V1_Default, "\nInitial feasibility: %f", this->m_state.m_fea);
		IOUtils::logTrace(Verbosity::V1_Default, "\n--");
	}

	//OptimSolverOptions OptimSolver::CreateDefaultOptions()
	//{
	//	OptimSolverOptions options;
	//	options.tolMaxError = 1e-6;
	//	options.numMaxIters = 1000;
	//	options.lsSolverType = LSSolverType::LS_EigenLDLT;
	//	options.qpSolverType = QPSolverType::QP_Newton;
	//	options.lSearch_iters = 10;
	//	options.lSearch_alpha = 0.5;
	//	options.maxStepSize = 1.0;
	//	options.profileTime = true;
	//	return options; 
	//}

	//bool OptimSolver::IsProblemSolved()
	//{
	//	if (!this->m_isInitialized)
	//		return false;

	//	if (!this->m_pProblem->IsFullyConstrained())
	//		return false;

	//	this->m_pProblem->GetGradient(this->m_state.m_vg);
	//	this->m_state.m_opt = this->m_state.m_vg.norm();
	//	return this->m_state.m_opt < m_options.tolMaxError;
	//}

	const OptimState& OptimSolver::SolveStep()
	{
		if (this->m_state.m_steepest)
		{
			QPSolverType originalSol = this->m_options.qpSolverType;
			this->m_options.qpSolverType = QPSolverType::QP_Steepest;
			this->m_state = this->SolveStepInternal();
			this->m_options.qpSolverType = originalSol;
		}
		else
		{
			this->SolveStepInternal();
		}

		this->OnEvent_AfterStep();
		
		return this->m_state;
	}

	const OptimState& OptimSolver::SolveFull()
	{
		if (!this->m_isInitialized)
			return this->m_state;

		m_pProblem->OnSolveStart(this->m_state);

		VectorXd vg;
		Real vgNorm = 0;
		int itCount = 0;

		this->m_pProblem->GetGradient(this->m_state.m_vg);
		this->m_state.m_opt = this->m_state.m_vg.norm();

		IOUtils::logTrace(Verbosity::V1_Default, "\n\n");
		IOUtils::logTrace(Verbosity::V1_Default, "\n-------------------------------------------------------------------------------------------------");
		IOUtils::logTrace(Verbosity::V1_Default, "\n-------------------------------------------------------------------------------------------------");
		IOUtils::logTrace(Verbosity::V1_Default, "\nStarting problem %s SOLVE. Optimality: %f. Max. Steps: %d. Max. Error: %f", this->m_pProblem->GetName().c_str(), m_state.m_opt, this->m_options.numMaxIters, this->m_options.tolMaxError);
		IOUtils::logTrace(Verbosity::V1_Default, "\n-------------------------------------------------------------------------------------------------");
		IOUtils::logTrace(Verbosity::V1_Default, "\n-------------------------------------------------------------------------------------------------");

		this->m_state.m_result = OSResult::OR_SUCCESS;
		QPSolverType qpType = m_options.qpSolverType;
		LSearchType	lsType = m_options.lSearchType;

		this->m_state.m_steepest = false;

		// While the gradient is not zero, iterate

		while (true)
		{
			itCount++;

			// Solve step

			this->SolveStep();

			this->m_pProblem->GetGradient(this->m_state.m_vg);
			this->m_state.m_opt = this->m_state.m_vg.norm();

			if (this->m_pProblem->IsFullyConstrained())
			{
				if (this->m_state.m_result == OSResult::OR_NONDESC)
				{
					if (!this->m_options.trySteepest)
					{
						break; // Cannot improve
					}
					else
					{
						if (this->m_state.m_steepest)
						{
							break; // Cannot improve
						}

						// Try the next step with steepest
						this->m_state.m_steepest = true;
					}
				}

				if (this->m_state.m_result == OSResult::OR_MINIMPR)
				{
					if (this->m_state.m_minImprCount == this->m_options.minImprMaxCount)
						break; // Already converged

					if (this->m_options.trySteepest)
					{
						// Try the next step with the oposite method as current
						this->m_state.m_steepest = !this->m_state.m_steepest;
					}
				}

				if (this->m_state.m_result == OSResult::OR_MINSTEP)
				{
					if (this->m_state.m_minStepCount == this->m_options.minStepMaxCount)
						break; // Already converged

					if (this->m_options.trySteepest)
					{
						// Try the next step with the oposite method as current
						this->m_state.m_steepest = !this->m_state.m_steepest;
					}
				}

				if (this->m_state.m_result == OSResult::OR_SUCCESS)
				{
					if (this->m_state.m_opt <= m_options.tolMaxError)
						break; // Already converged
					
					this->m_state.m_result = OSResult::OR_ONGOING;
				}

				if (this->m_state.m_result == OSResult::OR_ONGOING)
				{
					// Try the next step with selected
					this->m_state.m_steepest = false;
				}
			}

			if (itCount + 1 > this->m_options.numMaxIters)
			{
				this->m_state.m_result = OSResult::OR_MAXITER;
				break;
			}
		}

		string status;
		switch (this->m_state.m_result)
		{
		case OR_FAILURE: status = "[FAILURE]"; break;
		case OR_SUCCESS: status = "[SUCCESS]"; break;
		case OR_MINSTEP: status = "[MINSTEP]"; break;
		case OR_MINIMPR: status = "[MINIMPR]"; break;
		case OR_MAXITER: status = "[MAXITER]"; break;
		case OR_NONDESC: status = "[NONDESC]"; break;
		}

		IOUtils::logTrace(Verbosity::V1_Default, "\n-------------------------------------------------------------------------------------------------");
		IOUtils::logTrace(Verbosity::V1_Default, "\n-------------------------------------------------------------------------------------------------");
		IOUtils::logTrace(Verbosity::V1_Default, "\nFinishing problem. Steps: %d, Optimality: %f. Status: %s", itCount, m_state.m_opt, status.c_str());
		IOUtils::logTrace(Verbosity::V1_Default, "\n-------------------------------------------------------------------------------------------------");
		IOUtils::logTrace(Verbosity::V1_Default, "\n-------------------------------------------------------------------------------------------------");
		IOUtils::logTrace(Verbosity::V1_Default, "\n\n");

		m_pProblem->OnSolveFinish(this->m_state);

		this->OnEvent_AfterFull();

		return this->m_state;
	}


	bool OptimSolver::ComputeStep(VectorXd& dx, int trial)
	{
		this->m_timerComputeStep.Start();

		bool result = false;

		switch (this->m_options.qpSolverType)
		{
		case QPSolverType::QP_Newton: result = this->ComputeStep_Newton(dx, trial); break;
		case QPSolverType::QP_BFGS_D: result = this->ComputeStep_BFGSDirect(dx, trial); break;
		case QPSolverType::QP_BFGS_I: result = this->ComputeStep_BFGSInverse(dx, trial); break;
		case QPSolverType::QP_LBFGS: result = this->ComputeStep_LBFGS(dx, trial); break;
		case QPSolverType::QP_Steepest: dx = -this->m_state.m_vg; result = true; break;
		default:
			throw exception("Invalid step type");
		}

		if (!result)
		{
			dx = -this->m_state.m_vg;
		}

		// Limit the size of dx if necessary

		double stepLength = dx.norm();
		if (stepLength > this->m_options.maxStepSize)
			dx = (dx / stepLength)*this->m_options.maxStepSize;

		this->m_timerComputeStep.StopStoreLog();

		return true;
	}

	bool OptimSolver::LineSearch(const VectorXd& dxTry, VectorXd& dxFin, Real& objNew, AVectorXd& vgNew, int& numBis)
	{
		this->m_timerLineSearch.Start();

		bool improved = false;

		switch (this->m_options.lSearchType)
		{
		case LSearchType::LSearch_Simple: improved = this->LineSearch_Simple(dxTry, dxFin, objNew, vgNew, numBis); break;
		case LSearchType::LSearch_Armijo: improved = this->LineSearch_Armijo(dxTry, dxFin, objNew, vgNew, numBis); break;
		case LSearchType::LSearch_WolfeWeak: improved = this->LineSearch_WolfeWeak(dxTry, dxFin, objNew, vgNew, numBis); break;
		case LSearchType::LSearch_WolfeStrong: improved = this->LineSearch_WolfeStrong(dxTry, dxFin, objNew, vgNew, numBis); break;
		case LSearchType::LSearch_None: improved = this->LSearch_None(dxTry, dxFin, objNew, vgNew, numBis); break;
		default:
			throw exception("Invalid line search type");
		}

		if (improved)
		{
			IOUtils::logTrace(Verbosity::V1_Default, "\n[SUCCESS] Improved. Objective: %.9e. Optimality: %.9e. Bis: %i. Step: %.9e\n--", objNew, vgNew.norm(), numBis, dxFin.norm());
		}
		else
		{
			IOUtils::logTrace(Verbosity::V1_Default, "\n[FAILURE] Non-Descent. Objective: %.9e. Optimality: %.9e. Bis: %i. Step: %.9e\n--", objNew, vgNew.norm(), numBis, dxFin.norm());
		}

		this->m_timerLineSearch.StopStoreLog();

		return improved;
	}

	void OptimSolver::UpdateStepResult(const VectorXd& dx, const Real& objNew, const AVectorXd& vgNew)
	{
		Real impr = this->m_state.m_obj - objNew;
		Real step = dx.norm();

		if (impr > 0)
		{
			Real optNew = vgNew.norm();

			if (impr < this->m_options.tolMinImpr)
			{
				this->m_state.m_minImprCount++;
				this->m_state.m_result = OSResult::OR_MINIMPR;
			}
			else if (step < this->m_options.tolMinStep)
			{
				this->m_state.m_minStepCount++;
				this->m_state.m_result = OSResult::OR_MINSTEP;
			}
			else if (optNew <= this->m_options.tolMaxError)
			{
				this->m_state.m_result = OSResult::OR_SUCCESS;
				this->m_state.m_minImprCount = 0;
				this->m_state.m_minStepCount = 0;
			}
			else
			{
				this->m_state.m_result = OSResult::OR_ONGOING;
				this->m_state.m_minImprCount = 0;
				this->m_state.m_minStepCount = 0;
			}

			this->m_state.m_v0 = this->m_state.m_vx;
			this->m_state.m_vx += dx;
			this->m_state.m_vg = vgNew;
			this->m_state.m_obj = objNew;
			this->m_state.m_opt = optNew;
		}
		else
		{
			this->m_state.m_result = OSResult::OR_NONDESC;

			//VectorXd vminPos = this->m_state.m_vx - 2 * dx;
			//VectorXd vmaxPos = this->m_state.m_vx + 2 * dx;
			//VectorXd vranPos = vmaxPos - vminPos;
			//MatrixXd menergies(2, 1000);
			//for (int i = 0; i < 1000; ++i)
			//{
			//	Real energy;
			//	Real alpha = (Real) i / (Real) (1000 - 1);
			//	m_pProblem->SetVariables(vminPos + vranPos*alpha);
			//	m_pProblem->GetEnergy(energy);
			//	menergies(0, i) = (vranPos*alpha).norm();
			//	menergies(1, i) = energy;
			//}

			//logFile("energyConvergence.csv", matrixToString_CSV(menergies));

			//VectorXd vmaxPos = this->m_state.m_vx + 0.00001 * dx.normalized();
			//VectorXd vranPos = vmaxPos - this->m_state.m_vx;
			//MatrixXd menergies(2, 1001);
			//for (int i = 0; i < 1001; ++i)
			//{
			//	Real energy;
			//	Real alpha = (Real)i / (Real)(1001 - 1);
			//	m_pProblem->SetVariables(this->m_state.m_vx + vranPos*alpha);
			//	m_pProblem->GetEnergy(energy);
			//	menergies(0, i) = (vranPos*alpha).norm();
			//	menergies(1, i) = energy;
			//}

			//logFile("energyConvergence.csv", matrixToString_CSV(menergies));
		}
	}

	bool OptimSolver::ComputeStep_Newton(VectorXd& dx, int trial)
	{
		this->m_pProblem->GetHessian(this->m_state.m_mH);

		MatrixSd mH = this->m_state.m_mH;

		IOUtils::logTrace(Verbosity::V1_Default, "\n[TRACE] Computing solver step trial: %d", trial);

		if (trial > 0)
		{
			Real normA;
			Real sinThres;
			Real regThres;
			this->m_pLSolver->GetMatrixData(mH, normA, sinThres, regThres);
			MatrixSd mI(mH.rows(), mH.cols());
			mI.setIdentity();
			mH += mI*regThres*pow(10, trial - 1);
		}

		// Solve the system H*dx = -g

		dx = this->m_state.m_vx - this->m_state.m_v0;

		this->m_timerLinearSolve.Start();
		this->m_pLSolver->Solve(mH, -(1.0)*this->m_state.m_vg, dx);
		this->m_timerLinearSolve.StopStoreLog();

		return true;
	}

	bool OptimSolver::ComputeStep_BFGSDirect(VectorXd& dx, int trial)
	{
		this->UpdateBFGS_Direct(this->m_state.m_vx, this->m_state.m_vg);

		// Solve the system H*dx = -g

		dx = this->m_state.m_vx - this->m_state.m_v0;

		this->m_timerLinearSolve.Start();
		this->m_pLSolver->Solve(this->m_bfgs.m_mBFGS, -(1.0)*this->m_state.m_vg, dx);
		this->m_timerLinearSolve.StopStoreLog();

		return true;
	}

	bool OptimSolver::ComputeStep_BFGSInverse(VectorXd& dx, int trial)
	{
		this->UpdateBFGS_Inverse(this->m_state.m_vx, this->m_state.m_vg);


		// Solve multiplication -H^-1*g

		dx = -this->m_bfgs.m_mBFGS*this->m_state.m_vg;
		
		return true;
	}

	bool OptimSolver::ComputeStep_LBFGS(VectorXd& dx, int trial)
	{
		if (!this->m_bfgs.m_isInit)
		{
			this->m_bfgs.m_vx_P = this->m_state.m_vx;
			this->m_bfgs.m_vg_P = this->m_state.m_vg;

			this->m_bfgs.m_M = 3;
			this->m_bfgs.m_N = (int) this->m_state.m_vx.size();
			this->m_bfgs.m_mS.resize(m_bfgs.m_N, m_bfgs.m_M);
			this->m_bfgs.m_mY.resize(m_bfgs.m_N, m_bfgs.m_M);
			this->m_bfgs.m_vys.resize(m_bfgs.m_M);
			this->m_bfgs.m_va.resize(m_bfgs.m_M);
			this->m_bfgs.m_K = 0;

			// Steep!
			dx = -this->m_state.m_vg;

			this->m_bfgs.m_isInit = true;

			IOUtils::logTrace(Verbosity::V1_Default, "\n[INFO] Initializing BFGS step to gradient");

			return true;
		}

		// Update s and y
		// s_{k+1} = x_{k+1} - x_k
		// y_{k+1} = g_{k+1} - g_k

		MapVecXd svec(&this->m_bfgs.m_mS(0, this->m_bfgs.m_K), this->m_bfgs.m_N);
		MapVecXd yvec(&this->m_bfgs.m_mY(0, this->m_bfgs.m_K), this->m_bfgs.m_N);
		svec.noalias() = this->m_state.m_vx - this->m_bfgs.m_vx_P;
		yvec.noalias() = this->m_state.m_vg - this->m_bfgs.m_vg_P;

		// ys = y's = 1/rho
		// yy = y'y

		Real ys = yvec.dot(svec);
		Real yy = yvec.squaredNorm();
		this->m_bfgs.m_vys[this->m_bfgs.m_K] = ys;

		IOUtils::logTrace(Verbosity::V1_Default, "\n[INFO] Updated BFGS step with discriminant %f", ys);

		dx.noalias() = -this->m_state.m_vg;

		// Recursive formula to compute d = -H * g

		int bound = std::min(this->m_bfgs.m_M, this->m_bfgs.m_K+1);
		this->m_bfgs.m_K = (this->m_bfgs.m_K + 1) % this->m_bfgs.m_M;
		int j = this->m_bfgs.m_K;
		for (int i = 0; i < bound; i++)
		{
			j = (j + this->m_bfgs.m_M - 1) % this->m_bfgs.m_M;
			MapVecXd sj(&this->m_bfgs.m_mS(0, j), this->m_bfgs.m_N);
			MapVecXd yj(&this->m_bfgs.m_mY(0, j), this->m_bfgs.m_N);
			this->m_bfgs.m_va[j] = sj.dot(dx) / this->m_bfgs.m_vys[j];
			dx.noalias() -= this->m_bfgs.m_va[j] * yj;
		}


		dx *= (ys / yy);

		for (int i = 0; i < bound; i++)
		{
			MapVecXd sj(&this->m_bfgs.m_mS(0, j), this->m_bfgs.m_N);
			MapVecXd yj(&this->m_bfgs.m_mY(0, j), this->m_bfgs.m_N);
			Real beta = yj.dot(dx) / this->m_bfgs.m_vys[j];
			dx.noalias() += (this->m_bfgs.m_va[j] - beta) * sj;
			j = (j + 1) % this->m_bfgs.m_N;
		}

		return true;
	}

	bool OptimSolver::UpdateBFGS_Inverse(const VectorXd& vxNew, const VectorXd& vgNew)
	{
		if (!this->m_bfgs.m_isInit) return this->InitialBFGS(vxNew, vgNew);

		// TODO

		return false;
	}

	bool OptimSolver::UpdateBFGS_Direct(const VectorXd& vxNew, const VectorXd& vgNew)
	{
		if (!this->m_bfgs.m_isInit) return this->InitialBFGS(vxNew, vgNew);

		this->m_bfgs.m_vs = vxNew - this->m_bfgs.m_vx_P;
		this->m_bfgs.m_vy = vgNew - this->m_bfgs.m_vg_P;
		
		this->m_bfgs.m_vx_P = vxNew;
		this->m_bfgs.m_vg_P = vgNew;
		
		double D = m_bfgs.m_vy.dot(m_bfgs.m_vs);
		
		if (D > 1e-6) // ?
		{
			IOUtils::logTrace(Verbosity::V1_Default, "\n[SUCCESS] Updating BFGS matrix with discriminant %f", D);

			Real rho = 1.0 / D; // For too small discriminant, the update is invalid

			// Update the HessianFull matrix (direct one) (Numerical Optimization, Nocedal, Wright, page 25)
			// Hk+1 = (I - rho * m_sk * m_yk^T) * Hk * (I - rho * m_yk * m_sk^T) + rho * m_yk * m_yk^T

			MatrixSd& W = this->m_bfgs.m_mBFGS;
			int N = (int)m_bfgs.m_mBFGS.rows();

			m_bfgs.m_mBFGSMod_A(N, N);
			m_bfgs.m_mBFGSMod_B(N, N);

			for (int i = 0; i < N; i++)
			{
				for (int j = 0; j <= i; j++)
				{
					if (i == j)
					{
						m_bfgs.m_mBFGSMod_A(i, i) = 1.0 - rho * m_bfgs.m_vy[i] * m_bfgs.m_vs[i];
					}
					else
					{
						m_bfgs.m_mBFGSMod_A(i, j) = -rho * m_bfgs.m_vs[i] * m_bfgs.m_vy[j];
						m_bfgs.m_mBFGSMod_A(j, i) = -rho * m_bfgs.m_vs[j] * m_bfgs.m_vy[i];
					}
			
					m_bfgs.m_mBFGSMod_B(i, j) = rho * m_bfgs.m_vy[i] * m_bfgs.m_vy[j];
					m_bfgs.m_mBFGSMod_B(j, i) = rho * m_bfgs.m_vy[j] * m_bfgs.m_vy[i];
				}
			}
			
			W = (m_bfgs.m_mBFGSMod_A*this->m_bfgs.m_mBFGS.toDense()*m_bfgs.m_mBFGSMod_A.transpose() + m_bfgs.m_mBFGSMod_B).sparseView();
		}
		else
		{
			IOUtils::logTrace(Verbosity::V1_Default, "\n[FAILURE] BFGS discriminant to low %f", D);

			this->InitialBFGS(vxNew, vgNew);
		}

		return true;
	}

	bool OptimSolver::InitialBFGS(const VectorXd& vxNew, const VectorXd& vgNew)
	{
		IOUtils::logTrace(Verbosity::V1_Default, "\n[INFO] Initializing BFGS matrix to Hessian");

		this->m_bfgs.m_vx_P = vxNew;
		this->m_bfgs.m_vg_P = vgNew;

		this->m_bfgs.m_mBFGS = this->m_state.m_mH;

		MatrixSd mHfull = this->m_bfgs.m_mBFGS.selfadjointView<Lower>();
		this->m_bfgs.m_mBFGS = mHfull; // Copy necessary to do it effective

		// Set identity

		int N = (int)m_bfgs.m_mBFGS.rows();

		this->m_bfgs.m_mBFGS *= 0.0;
		for (int i = 0; i < N; ++i)
			this->m_bfgs.m_mBFGS.coeffRef(i, i) = 1.0;

		this->m_bfgs.m_isInit = true;

		return true;
	}

	bool OptimSolver::LineSearch_Simple(const VectorXd& dxIn, VectorXd& dxOut, Real& objNew, AVectorXd& vgNew, int& numBis)
	{
		Real factor = 1;

		bool improved = false;

		this->m_pProblem->PrePerformStep(m_state);

		numBis = -1;

		do
		{
			numBis++;

			// Perform step!

			dxOut = factor*dxIn;

			IOUtils::logTrace(Verbosity::V4_DeepShit, "\n[TRACE] Trying LS iteration %d, step size: %.6e", numBis, dxOut.norm());

			this->m_pProblem->IncVariables(dxOut);
			this->m_pProblem->PosPerformStep(m_state);

			// Recompute optimized energy

			this->m_pProblem->GetEnergy(objNew);
			this->m_pProblem->GetGradient(vgNew);

			// Improvement?

			if (objNew < this->m_state.m_obj)
			{
				improved = true;
				break; // Found!
			}

			// It didn't improve so reduce the step

			factor *= this->m_options.lSearch_alpha;
			this->m_pProblem->PrePerformStep(m_state);

		} while (dxOut.norm() > this->m_options.tolMinStep && numBis <= this->m_options.lsearch_numTry);



		return improved;
	}

	bool OptimSolver::LineSearch_Armijo(const VectorXd& dxIn, VectorXd& dxOut, Real& objNew, AVectorXd& vgNew, int& numBis)
	{
		Real factor = 1;
		Real t = this->m_options.lSearch_wolfe1*dxIn.normalized().dot(this->m_state.m_vg);

		bool improved = false;
		this->m_pProblem->PrePerformStep(m_state);
		
		numBis = -1;

		do
		{
			numBis++;

			// Perform step!

			dxOut = factor*dxIn;
			this->m_pProblem->IncVariables(dxOut);
			this->m_pProblem->PosPerformStep(this->m_state);

			// Recompute optimized energy

			this->m_pProblem->GetEnergy(objNew);
			this->m_pProblem->GetGradient(vgNew);

			// Improvement?

			if (objNew <= this->m_state.m_obj + factor*t)
			{
				improved = true;
				break; // Found!
			}

			// It didn't improve so reduce the step

			factor *= this->m_options.lSearch_alpha;
			this->m_pProblem->PrePerformStep(m_state);

		} while (dxOut.norm() > this->m_options.tolMinStep && numBis <= this->m_options.lsearch_numTry);

		return improved;
	}

	bool OptimSolver::LineSearch_WolfeWeak(const VectorXd& dxIn, VectorXd& dxOut, Real& objNew, AVectorXd& vgNew, int& numBis)
	{
		Real factor = 1;
		Real c1 = this->m_options.lSearch_wolfe1;
		Real c2 = this->m_options.lSearch_wolfe2;
		Real m = dxIn.dot(this->m_state.m_vg);

		Real t1 = c1*m;
		Real t2 = -c2*m;

		bool improved = false;
		this->m_pProblem->PrePerformStep(m_state);
		
		numBis = -1;

		do
		{
			numBis++;

			// Perform step!

			dxOut = factor*dxIn;
			this->m_pProblem->IncVariables(dxOut);
			this->m_pProblem->PosPerformStep(this->m_state);

			// Recompute optimized energy and gradient

			this->m_pProblem->GetEnergy(objNew);
			this->m_pProblem->GetGradient(vgNew);

			// Improvement?

			if (objNew <= this->m_state.m_obj + factor*t1 && -dxIn.dot(vgNew) <= t2)
			{
				improved = true;
				break; // Found!
			}

			// It didn't improve so reduce the step

			factor *= this->m_options.lSearch_alpha;
			this->m_pProblem->PrePerformStep(m_state);

		} while (dxOut.norm() > this->m_options.tolMinStep && numBis <= this->m_options.lsearch_numTry);

		return improved;
	}

	bool OptimSolver::LineSearch_WolfeStrong(const VectorXd& dxIn, VectorXd& dxOut, Real& objNew, AVectorXd& vgNew, int& numBis)
	{
		Real factor = 1;
		Real c1 = this->m_options.lSearch_wolfe1;
		Real c2 = this->m_options.lSearch_wolfe2;
		Real m = dxIn.dot(this->m_state.m_vg);

		Real t1 = c1*m;
		Real t2 = -c2*m;
		Real t3 = c2*abs(m);

		bool improved = false;
		this->m_pProblem->PrePerformStep(m_state);
		
		numBis = -1;

		do
		{
			numBis++;

			// Perform step!

			dxOut = factor*dxIn;
			this->m_pProblem->IncVariables(dxOut);
			this->m_pProblem->PosPerformStep(this->m_state);

			// Recompute optimized energy and gradient

			this->m_pProblem->GetEnergy(objNew);
			this->m_pProblem->GetGradient(vgNew);

			// Improvement?

			if (objNew <= this->m_state.m_obj + factor*t1 && -dxIn.dot(vgNew) <= t2 && abs(dxIn.dot(vgNew)) <= t3)
			{
				improved = true;
				break; // Found!
			}

			// It didn't improve so reduce the step

			factor *= this->m_options.lSearch_alpha;
			this->m_pProblem->PrePerformStep(m_state);

		} while (dxOut.norm() > this->m_options.tolMinStep && numBis <= this->m_options.lsearch_numTry);

		return improved;
	}

	bool OptimSolver::LSearch_None(const VectorXd& dxIn, VectorXd& dxOut, Real& objNew, AVectorXd& vgNew, int& numBis)
	{
		dxOut = dxIn;
		this->m_pProblem->PrePerformStep(this->m_state);
		this->m_pProblem->IncVariables(dxOut);
		this->m_pProblem->PosPerformStep(this->m_state);

		this->m_pProblem->GetEnergy(objNew);
		this->m_pProblem->GetGradient(vgNew);
		numBis = 0;

		return true;
	}


}
