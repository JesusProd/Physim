//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Boundary/BC_FixDoFPartial.h>

#include <PhySim/Physics/Simulables/Simulable_Mesh.h>

#include <PhySim/Physics/Elements/ConstraintSet_FixedPosition.h>
#include <PhySim/Physics/Elements/EnergyElement_SoftConstraint.h>

#include <PhySim/Kinematics/KinematicsEle.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	BC_FixDoFPartial::BC_FixDoFPartial(Simulable* pModel, const vector<IDoFSet*>& vpDoF, const vector<VectorXd>& vt, const vector<bVector>& vFixed) : BCondition(pModel)
	{
		this->m_vpDoF = vpDoF;

		if (vFixed.empty())
		{
			this->m_vFixed.resize(this->m_vpDoF.size());
			for (size_t i = 0; i < m_vpDoF.size(); ++i)
				this->m_vFixed[i].resize(this->m_vpDoF[i]->NumDim(), true);
		}
		else this->m_vFixed = vFixed;

		// Initialize values to hold current position

		int numP = (int)vpDoF.size();
		this->m_vini.resize(numP);
		for (size_t i = 0; i < numP; ++i)
		{
			if (!vt.empty()) this->m_vini[i] = vt[i];
			else this->m_vini[i] = vpDoF[i]->GetValue();
		}
	}

	BC_FixDoFPartial::~BC_FixDoFPartial(void)
	{
		// Nothing to do here...
	}

	void BC_FixDoFPartial::Init()
	{
		int numP = (int)m_vpDoF.size();

		// Enforce DoF position fixation

		for (size_t i = 0; i < numP; ++i)
			m_vpDoF[i]->SetFixed(m_vFixed[i]);

		m_pModel->DirtyFixed();
	}

	void BC_FixDoFPartial::Update()
	{
		vector<VectorXd> vval;
		this->GetCurValues(vval);

		assert(vval.size() == m_vpDoF.size());

		bool changed = false;

		for (size_t i = 0; i < (int)m_vpDoF.size(); ++i)
		{
			VectorXd vx = m_vpDoF[i]->GetValue();
			for (Index j = 0; j < vx.size(); ++j)
				if (this->m_vFixed[i][j])
				{
					if (vx[j] != vval[i][j])
					{
						vx[j] = vval[i][j];
						changed = true;
					}	
				}

			m_vpDoF[i]->SetValue(vx);
		}

		this->m_pModel->GetKinematicsTree()->MapPositionX();

		if (changed)
			this->m_pModel->DirtyMechanics();
	}

}

