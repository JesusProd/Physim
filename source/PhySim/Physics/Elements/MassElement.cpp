//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/MassElement.h>

#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Kinematics/KinematicsEle.h>

#include <PhySim/Utils/DerivativeAssembler.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	MassElement::MassElement(Simulable* pModel, PtrS<ParameterSet> pParams) : IMassElement(pModel)
	{
		this->m_pParams = pParams;

		m_vDMDtv = VectorXd::Zero(0);
		m_mMass = MatrixXd::Zero(0, 0);

		this->m_vgravity.setZero();
	}

	MassElement::~MassElement()
	{
		// Nothing to do...
	}

	void MassElement::Init()
	{
		// Nothing to do...
	}

	void MassElement::AssembleGlobal_DMDtv(AVectorXd& vglobalDMDtv)
	{
		if (this->m_vDMDtv.size() == 0)
			return; // Ignore assembly

		this->m_pAssembler->PropagateAndAssembleGradient(this->m_vDMDtv, vector<IDoFSet*>(this->m_vDoF.begin(), this->m_vDoF.end()), vglobalDMDtv);
	}

	void MassElement::AssembleGlobal_Mass(AMatrixSd& mglobalMass)
	{
		if (this->m_mMass.size() == 0)
			return; // Ignore assembly

		this->m_pAssembler->PropagateAndAssembleHessian(this->m_mMass, vector<IDoFSet*>(this->m_vDoF.begin(), this->m_vDoF.end()), mglobalMass);
	}

}