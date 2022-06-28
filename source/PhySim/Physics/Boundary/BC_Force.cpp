//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Boundary/BC_Force.h>

#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Physics/Elements/EnergyElement_Force.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	BC_Force::BC_Force(Simulable* pModel, const vector<KinematicsEle*>& vpDoF, const vector<VectorXd>& vforce) : BCondition(pModel)
	{
		assert(vforce.size() == vpDoF.size());
		
		this->m_vpDoF = vpDoF;
		this->m_vini = vforce;
	}

	BC_Force::BC_Force(Simulable* pModel, const vector<KinematicsEle*>& vpDoF, const VectorXd& vforce) : BCondition(pModel)
	{
		this->m_vpDoF = vpDoF;

		int numP = (int) this->m_vpDoF.size();

		this->m_vini.resize(numP);
		for (int i = 0; i < numP; ++i)
			this->m_vini[i] = vforce;
	}


	BC_Force::~BC_Force(void)
	{
		// Nothing to do here...
	}

	void BC_Force::Init()
	{
		int numP = (int) this->m_vpDoF.size();

		// Initialize energy element

		this->m_vpEne.reserve(1);

		this->m_pEleForce = PtrS<EnergyElement_Force>(new EnergyElement_Force(this->m_pModel, this->m_vpDoF));
		m_pEleForce->SetForces(this->m_vini);
		this->m_vpEne.push_back(m_pEleForce);
	}

	void BC_Force::Update()
	{
		vector<VectorXd> vval;
		this->GetCurValues(vval);

		m_pEleForce->SetForces(vval);
		this->m_pModel->DirtyMechanics();
	}

}

