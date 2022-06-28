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
#include <PhySim/Utils/ParameterSet.h>
#include <PhySim/Geometry/Polytopes/Node.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Node;

	class EnergyElement_FaceNodeColl_Quad : public EnergyElement
	{
	public:
		EnergyElement_FaceNodeColl_Quad(Simulable* pModel, Face* pFace, Node* pNode);

		virtual ~EnergyElement_FaceNodeColl_Quad(void);

		virtual void ComputeAndStore_Energy_Internal();
		virtual void ComputeAndStore_Gradient_Internal();
		virtual void ComputeAndStore_Hessian_Internal();

	};
}