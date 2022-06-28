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


#include <PhySim/Physics/Elements/MassElement.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Poly;
	class Simulable;
	class ParameterSet;

	class MassElement_Lumped : public MassElement
	{
	protected:
		Poly* m_pPoly;

	public:

		MassElement_Lumped(Simulable* pModel, Poly* pPoly, PtrS<ParameterSet> pParams);

		virtual ~MassElement_Lumped(void);

		virtual void Init();

		virtual void ComputeAndStore_Energy_Internal() override;
		virtual void ComputeAndStore_Gradient_Internal() override;

		virtual void ComputeAndStore_Mass() override;

		virtual int GetHessianSize() const override { return 0; };

		virtual Real GetElementMass() const { return this->m_mMass(0,0); };

	};
}
