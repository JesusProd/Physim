//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#pragma once

#include <PhySim/Solvers/OptimProblem.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class ISimulable;
    struct OptimState;

	class OptimProblem_BasicDynamic : public OptimProblem
	{
	protected:
		ISimulable* m_pM;

		Real						m_dt;
		VectorXd					m_vx0;
		VectorXd					m_vv0;
		VectorXd					m_pS;
		AMatrixSd					m_mHess;
		AMatrixSd					m_mMass;
		AVectorXd					m_vDMvDt;

	public:

		OptimProblem_BasicDynamic(ISimulable* pM, Real dt);
		virtual ~OptimProblem_BasicDynamic();

		virtual void Init(ISimulable* pM, Real dt);

		virtual void GetVariables(VectorXd& vx) const override;
		virtual void IncVariables(const VectorXd& vx) override;
		virtual void SetVariables(const VectorXd& vx) override;

		virtual bool GetEnergy(Real& e) override;
		virtual bool GetGradient(AVectorXd& vg) override;
		virtual bool GetHessian(AMatrixSd& mH) override;

		virtual bool OnSolveStart(const OptimState& state) override;
		virtual bool OnSolveFinish(const OptimState& state) override;

		virtual bool OnIterStart(const OptimState& state) override;
		virtual bool OnIterFinish(const OptimState& state) override;

		virtual bool PrePerformStep(const OptimState& state) override;
		virtual bool PosPerformStep(const OptimState& state) override;

		virtual bool IsFullyConstrained() override;

	};
}