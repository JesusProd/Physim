//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Utils/Simulator.h>

#include <PhySim/Solvers/OptimProblem.h>
#include <PhySim/Solvers/OptimSolver.h>

#include <PhySim/Physics/Boundary/BC_Force.h>
#include <PhySim/Physics/Boundary/BC_FixDoF.h>
#include <PhySim/Physics/Boundary/BC_Gravity.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	Simulator::Simulator()
	{
		// Nothing to do here...
	}

	Simulator::~Simulator()
	{
		// Nothing to do here...
	}

	void Simulator::FreeInternal()
	{
		this->m_pModel.reset();
		this->m_pProblem.reset();
		this->m_pSolver.reset();
		this->m_vpBC.clear();
	}

	bool Simulator::Init(PtrS<Simulable> pModel,
						 PtrS<OptimProblem> pProblem,
						 PtrS<OptimSolver> pSolver,
						 const vector<PtrS<IBCondition>>& vpBC)
	{
		assert(pModel.get() != NULL);
		assert(pProblem.get() != NULL);
		assert(pSolver.get() != NULL);

		this->FreeInternal();

		this->m_pModel.reset();
		this->m_pProblem.reset();
		this->m_pSolver.reset();
		this->m_vpBC.clear();

		this->m_pModel = pModel;
		this->m_pProblem = pProblem;
		this->m_pSolver = pSolver;
		this->m_vpBC = vpBC;

		return true;
	}

	OptimState Simulator::SolveStep()
	{
		int maxIters = m_pSolver->Options().numMaxIters;
		this->m_pSolver->Options().numMaxIters = 1;
		OptimState state = this->m_pSolver->SolveFull();
		this->m_pSolver->Options().numMaxIters = maxIters;

		return state;
	}

	OptimState Simulator::SolveFull()
	{
		return this->m_pSolver->SolveFull();
	}

	//BCondition* Simulator::CreateBC_Gravity(BCSetup& bc, Simulable* pModel)
	//{
	//	//bc.m_type = BCType::Gravity;
	//	//bc.m_vini.push_back(bc.m_vini);
	//	//bc.m_pBC = new BC_Gravity(pModel, bc);
	//	//return bc.m_pBC;

	//	return NULL;
	//}

	//BCondition* Simulator::CreateBC_SelBoxFile(BCSetup& bc, Simulable* pModel)
	//{
	//	//if (!LoadSelectionBox(
	//	//	bc.m_selPath,
	//	//	bc.m_vselMin,
	//	//	bc.m_vselMax))
	//	//	return NULL;

	//	return Simulator::CreateBC_SelBoxBounds(bc, pModel);
	//}

	//BCondition* Simulator::CreateBC_SelBoxBounds(BCSetup& bc, Simulable* pModel)
	//{
	//	//Vector3d vselMin = bc.m_vselMin.cast<Real>();
	//	//Vector3d vselMax = bc.m_vselMax.cast<Real>();
	//	//bc.m_vpDoF = pModel->SelectDoF(vselMin, vselMax, bc.m_selSpace);
	//	//size_t numDoF = bc.m_vpDoF.size();
	//	//bc.m_vini.resize(numDoF);
	//	//bc.m_vpDoF.resize(numDoF);

	//	//for (size_t i = 0; i < numDoF; ++i)
	//	//{
	//	//	switch (bc.m_type)
	//	//	{
	//	//	case BCType::Force:
	//	//		bc.m_vini[i] = Vector3d(
	//	//			bc.m_vini.x(),
	//	//			bc.m_vini.y(),
	//	//			bc.m_vini.z());
	//	//		break;
	//	//	case BCType::Fixed:
	//	//		// TODO
	//	//		break;
	//	//	}
	//	//}

	//	//switch (bc.m_type)
	//	//{
	//	//case BCType::Fixed: bc.m_pBC = new BC_FixDoF(pModel, bc); break;
	//	//case BCType::Force: bc.m_pBC = new BC_Force(pModel, bc); break;
	//	//}

	//	//return bc.m_pBC;

	//	return NULL;
	//}

	//BCondition* Simulator::CreateBC_Indices(BCSetup& bc, Simulable* pModel)
	//{
	//	BCondition* pCondition;

	//	// TODO

	//	return pCondition;
	//}

}
