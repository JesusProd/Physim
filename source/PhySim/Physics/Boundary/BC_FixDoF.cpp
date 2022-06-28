//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Boundary/BC_FixDoF.h>

#include <PhySim/Physics/Simulables/Simulable_Mesh.h>

#include <PhySim/Physics/Elements/ConstraintSet_FixedPosition.h>
#include <PhySim/Physics/Elements/EnergyElement_SoftConstraint.h>

#include <PhySim/Kinematics/KinematicsEle.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	BC_FixDoF::BC_FixDoF(Simulable* pModel, const vector<IDoFSet*>& vpDoF, const vector<VectorXd>& vt, bool isSoft, Real kAini, Real kAend) : BCondition(pModel)
	{
		this->m_vpDoF = vpDoF;
		this->m_isSoft = isSoft;
		this->m_kAend = kAend;
		this->m_kAini = kAini;

		// Initialize values to hold current position

		int numP = (int)vpDoF.size();
		this->m_vini.resize(numP);
		for (size_t i = 0; i < numP; ++i)
		{
			if (!vt.empty()) this->m_vini[i] = vt[i];
			else this->m_vini[i] = vpDoF[i]->GetValue();
		}
	}

	BC_FixDoF::~BC_FixDoF(void)
	{
		// Nothing to do here...
	}

	void BC_FixDoF::Init()
	{
		int numP = (int) m_vpDoF.size();

		if (this->m_isSoft)
		{
			// Initialize energy elements

			this->m_vpEne.reserve(numP);

			for (size_t i = 0; i < numP; ++i)
			{
				PtrS<ConstraintSet> pCon(new ConstraintSet_FixedPosition(m_pModel, (KinematicsEle*)m_vpDoF[i], m_vini[i]));
				PtrS<EnergyElement> pEne(new EnergyElement_SoftConstraint(m_pModel, pCon, this->m_kAini));
				this->m_vpEne.push_back(pEne);
				this->m_vpCon.push_back(pCon);
			}
		}
		else
		{
			// Enforce DoF position fixation

			for (size_t i = 0; i < numP; ++i)
				m_vpDoF[i]->SetFixed(true);
		
			m_pModel->DirtyFixed();
		}
	}

	void BC_FixDoF::Update()
	{
		vector<VectorXd> vval;
		this->GetCurValues(vval);

		assert(vval.size() == m_vpDoF.size());

		if (this->m_isSoft)
		{
			Real a = ((Real)CurStage() / (Real)NumStage());
			Real kA = m_kAini + a*(m_kAend - m_kAini);

			bool changed = false;

			for (int i = 0; i < (int)m_vpDoF.size(); ++i)
			{
				ConstraintSet_FixedPosition* pCon = static_cast<ConstraintSet_FixedPosition*>(this->m_vpCon[i].get());
				EnergyElement_SoftConstraint* pEne = static_cast<EnergyElement_SoftConstraint*>(this->m_vpEne[i].get());

				if (pCon->Target() != vval[i])
				{
					pCon->Target() = vval[i];
					changed = true;
				}

				if (pEne->Stiffness() != kA)
				{
					pEne->Stiffness() = kA;
					changed = true;
				}
			}

			if (changed)
				this->m_pModel->DirtyMechanics();
		}
		else
		{
			bool changed = false;

			for (size_t i = 0; i < (int)m_vpDoF.size(); ++i)
			{
				if (m_vpDoF[i]->GetValue() != vval[i])
				{
					m_vpDoF[i]->SetValue(vval[i]);
					changed = true;
				}	
			}
				
			if (changed)
				this->m_pModel->DirtyMechanics();
		}
	}

}

