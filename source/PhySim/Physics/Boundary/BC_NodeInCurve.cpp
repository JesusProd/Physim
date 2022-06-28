//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Boundary/BC_NodeInCurve.h>

#include <PhySim/Physics/Simulables/Simulable_Mesh.h>

#include <PhySim/Physics/Elements/ConstraintSet_NodeInCurve.h>
#include <PhySim/Physics/Elements/EnergyElement_SoftConstraint.h>

#include <PhySim/Physics/DoFSet.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	BC_NodeInCurve::BC_NodeInCurve(Simulable* pModel, const vector<IDoFSet*>& vpDoF, PtrS<Curve> pCurve, bool isSoft, Real kAini, Real kAend) : BCondition(pModel)
	{
		this->m_vpDoF = vpDoF;
		this->m_kAend = kAend;
		this->m_kAini = kAini;
		this->m_isSoft = isSoft;
		this->m_pCurve = pCurve;

		this->m_vini.resize(this->m_pCurve->NumNodes());
		for (int i = 0; i < this->m_pCurve->NumNodes(); ++i)
			this->m_vini[i] = pCurve->Nodes()[i]->Traits().Vector3d(Tag_Position_0);
	}

	BC_NodeInCurve::~BC_NodeInCurve(void)
	{
		// Nothing to do here...
	}

	void BC_NodeInCurve::Init()
	{
		int numP = (int)m_vpDoF.size();

		this->m_vpCon.reserve(numP);
		for (size_t i = 0; i < numP; ++i)
		{
			KEleParticle3D* pDOF = (KEleParticle3D*) m_vpDoF[i];
			PtrS<ConstraintSet> pCon(new ConstraintSet_NodeInCurve(m_pModel, true, pDOF, this->m_pCurve));
			this->m_vpCon.push_back(pCon);
		}

		if (this->m_isSoft)
		{
			// Initialize energy elements

			this->m_vpEne.reserve(numP);

			for (size_t i = 0; i < numP; ++i)
			{
				PtrS<ConstraintSet> pCon = dynamic_pointer_cast<ConstraintSet>(this->m_vpCon[i]);
				PtrS<EnergyElement> pEne(new EnergyElement_SoftConstraint(this->m_pModel, pCon, this->m_kAini));
				this->m_vpEne.push_back(pEne);
			}
		}
	}

	void BC_NodeInCurve::Update()
	{
		vector<VectorXd> vval;
		this->GetCurValues(vval);

		for (int i = 0; i < vval.size(); ++i)
		{
			this->m_pCurve->Nodes()[i]->Traits().Vector3d(Tag_Position_0) = vval[i];
		}

		if (this->m_isSoft)
		{
			Real a = ((Real)CurStage() / (Real)NumStage());
			Real kA = m_kAini + a*(m_kAend - m_kAini);

			bool changed = false;

			for (int i = 0; i < (int)m_vpDoF.size(); ++i)
			{
				EnergyElement_SoftConstraint* pEne = static_cast<EnergyElement_SoftConstraint*>(this->m_vpEne[i].get());

				if (pEne->Stiffness() != kA)
				{
					pEne->Stiffness() = kA;
					changed = true;
				}
			}

			if (changed)
				this->m_pModel->DirtyMechanics();
		}
	}

}
