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

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Edge;
	class Simulable;
	class ParameterSet;

	class EnergyElement_SpringCross : public EnergyElement
	{
	protected:

		Edge* m_pEdge0;
		Edge* m_pEdge1;
		Real m_restStrain;
		ParameterSet* m_pMaterial;

	public:
		EnergyElement_SpringCross(Simulable* pModel, Edge* pEdge0, Edge* pEdge1, ParameterSet* pMaterial);
		virtual ~EnergyElement_SpringCross(void);

		virtual void Init();

		virtual Real ComputeStrain(Tag s);
		inline virtual Real GetRestStrain() const { return this->m_restStrain; }
		inline virtual void GetRestStrain(Real rl) { this->m_restStrain = rl; }

		inline virtual ParameterSet* GetMaterial() { return this->m_pMaterial; }

		virtual void ComputeAndStore_Energy_Internal();
		virtual void ComputeAndStore_Gradient_Internal();
		virtual void ComputeAndStore_Hessian_Internal();

	};
}

