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


#include <PhySim/Physics/Elements/EnergyElement.h>
#include <PhySim/Physics/Elements/ConstraintSet.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class EnergyElement_SoftConstraint : public EnergyElement
	{

	protected:
		PtrS<ConstraintSet> m_pCon;
		Real m_k;

	public:
		EnergyElement_SoftConstraint(Simulable* pModel, const PtrS<ConstraintSet>& pCon, Real k, bool isDynamic = true);
		virtual ~EnergyElement_SoftConstraint(void);

		virtual void Init();

		virtual Real& Stiffness() { return this->m_k; }

		virtual void ComputeAndStore_Energy_Internal();
		virtual void ComputeAndStore_Gradient_Internal();
		virtual void ComputeAndStore_Hessian_Internal();
	};

}


