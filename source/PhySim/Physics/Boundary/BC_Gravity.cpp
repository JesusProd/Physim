//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Boundary/BC_Gravity.h>

#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Physics/Elements/EnergyElement_Gravity.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	BC_Gravity::BC_Gravity(Simulable* pModel, const Vector3d& vg) : BCondition(pModel)
	{
		this->m_vini.resize(1);
		this->m_vini[0] = vg;
	}

	BC_Gravity::~BC_Gravity(void)
	{
		// Nothing to do here...
	}

	void BC_Gravity::Init()
	{
		// Nothing to do here
	}

	void BC_Gravity::Update()
	{
		vector<VectorXd> vval;
		this->GetCurValues(vval);

		auto vMassEle = this->m_pModel->GetMassElements();
		for (auto pMassEle : vMassEle)
			pMassEle->SetGravity(vval[0]);

		this->m_pModel->DirtyMechanics();
	}

}

