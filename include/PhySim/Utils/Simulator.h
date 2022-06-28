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


#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Physics/Boundary/BCondition.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Simulable;
	class OptimSolver;
	class OptimProblem;
    struct OptimState;

	class Simulator
	{

	private:

		PtrS<OptimProblem>						m_pProblem;
		PtrS<OptimSolver>						m_pSolver;
		PtrS<Simulable>							m_pModel;
		vector<PtrS<IBCondition>>				m_vpBC;

		MatrixXi								m_mSurface;
		MatrixXd								m_mVertices;

	public:
		PtrS<OptimProblem> GetProblem() { return this->m_pProblem; }
		PtrS<OptimSolver> GetSolver() { return this->m_pSolver; }
		PtrS<Simulable> GetSimulable() { return this->m_pModel; }
		const vector<PtrS<IBCondition>>& GetBC() { return this->m_vpBC; }

		const MatrixXd& GetVertices() { return this->m_mVertices; }
		const MatrixXi& GetSurface() { return this->m_mSurface; }

		Simulator();

		virtual ~Simulator();

		virtual void FreeInternal();

		virtual bool Init(PtrS<Simulable> pModel,
						  PtrS<OptimProblem> pProblem,
					      PtrS<OptimSolver> pSolver,
						  const vector<PtrS<IBCondition>>& vpBC = vector<PtrS<IBCondition>>());

		virtual OptimState SolveStep();

		virtual OptimState SolveFull();

		//static BCondition* CreateBC_Gravity(BCSetup& bc, Simulable* pModel);
		//static BCondition* CreateBC_SelBoxFile(BCSetup& bc, Simulable* pModel);
		//static BCondition* CreateBC_SelBoxBounds(BCSetup& bc, Simulable* pModel);
		//static BCondition* CreateBC_Indices(BCSetup& bc, Simulable* pModel);

	};
}