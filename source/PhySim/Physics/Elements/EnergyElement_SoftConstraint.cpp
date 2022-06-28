//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_SoftConstraint.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/Face_Tri.h>
#include <PhySim/Physics/Simulables/Simulable.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	EnergyElement_SoftConstraint::EnergyElement_SoftConstraint(Simulable* pModel, const PtrS<ConstraintSet>& pCon, Real k, bool isDynamic) : EnergyElement(pModel, isDynamic)
	{
		this->m_pCon = pCon;

		m_vDoF = m_pCon->GetSupportDoF();
		int N = m_pCon->GetSupportSize();

		this->m_energy = 0;
		this->m_vgradient = VectorXd::Zero(N);
		this->m_mHessian = MatrixXd::Zero(N,N);

		this->m_k = k;

		// TODO: Any meaning?
		this->m_intVolume = 0; 
	}

	EnergyElement_SoftConstraint::~EnergyElement_SoftConstraint()
	{
		// Nothing to do here...
	}

	void EnergyElement_SoftConstraint::Init()
	{
		// Nothing to do here...
	}

	void EnergyElement_SoftConstraint::ComputeAndStore_Energy_Internal()
	{
		if (!this->m_pCon->IsActive())
		{
			this->m_energy = 0;
		}
		else
		{
			this->m_pCon->ComputeAndStore_Constraint();
			const VectorXd& vc = m_pCon->GetConstraint();
			this->m_energy = 0.5*this->m_k*vc.dot(vc);
		}
	}

	void EnergyElement_SoftConstraint::ComputeAndStore_Gradient_Internal()
	{
		if (!this->m_pCon->IsActive())
		{
			this->m_vgradient.setZero();
		}
		else
		{
			this->m_pCon->ComputeAndStore_Constraint();
			this->m_pCon->ComputeAndStore_Jacobian();
			const VectorXd& vc = m_pCon->GetConstraint();
			const MatrixXd& mJ = m_pCon->GetJacobian();
			this->m_vgradient = this->m_k*mJ.transpose()*vc;
		}
	}

	void EnergyElement_SoftConstraint::ComputeAndStore_Hessian_Internal()
	{
		if (!this->m_pCon->IsActive())
		{
			this->m_mHessian.setZero();
		}
		else
		{
			this->m_pCon->ComputeAndStore_Constraint();
			this->m_pCon->ComputeAndStore_Jacobian();
			this->m_pCon->ComputeAndStore_Hessian();
			const VectorXd& vc = m_pCon->GetConstraint();
			const MatrixXd& mJ = m_pCon->GetJacobian();
			const vector<MatrixXd>& vmH = m_pCon->GetLocalHessian();

			this->m_mHessian = this->m_k*mJ.transpose()*mJ;
			//for (int i = 0; i < m_pCon->GetSize(); ++i)
			//	this->m_mHessian += this->m_k*vmH[i]*vc(i);
		}
	}

}