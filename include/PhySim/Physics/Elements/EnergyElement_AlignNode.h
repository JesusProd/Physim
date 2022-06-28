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

	class Node;
	class Simulable;

	class EnergyElement_AlignNode : public EnergyElement
	{

	protected:
		Node* m_pEmbedding;
		Vector3d m_vxt;
		Real m_kA;


	public:
		EnergyElement_AlignNode(Simulable* pModel, Node* pNode);
		virtual ~EnergyElement_AlignNode(void);

		virtual void Init();

		virtual Real GetStiffness() const { return this->m_kA; }
		virtual void SetStiffness(Real kA) { this->m_kA = kA; }

		virtual const Vector3d& GetTargetPoint() const { return this->m_vxt; }
		virtual void SetTargetPoint(const Vector3d& vxt) { this->m_vxt = vxt; }

		virtual void ComputeAndStore_Energy_Internal();
		virtual void ComputeAndStore_Gradient_Internal();
		virtual void ComputeAndStore_Hessian_Internal();
	};

}


