//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, IST Austria
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
	class Edge;
	class Simulable_DER;

	class EnergyElement_DER_Connection: public EnergyElement
	{

	protected:
		Node* m_pCenter;
		Edge* m_pEdge;
		MatrixXd m_mDeDx;
		Simulable_DER* m_pModelDER;

	public:
		EnergyElement_DER_Connection(Simulable* pModel, Node* pCenter, Edge* pEdge);
		virtual ~EnergyElement_DER_Connection(void);

		virtual string GetName() const override { return "[DERConnection]"; };

		virtual void Init();

		virtual void ComputeAndStore_Energy_Internal();
		virtual void ComputeAndStore_Gradient_Internal();
		virtual void ComputeAndStore_Hessian_Internal();

	};
}

