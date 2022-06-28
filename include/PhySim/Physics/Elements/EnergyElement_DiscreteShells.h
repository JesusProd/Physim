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
	class Edge;
	class Mesh_Face;
	
	class EnergyElement_DiscreteShells : public EnergyElement
	{
	protected:
		Edge* m_pEdge;
		Mesh_Face* m_pMesh;
		vector<Node*> m_vnodes;

		Real m_theta0;
		Real m_shape0;

	public:
		EnergyElement_DiscreteShells(Simulable* pModel, Edge* pEdge);
		virtual ~EnergyElement_DiscreteShells(void);

		virtual void Init();

		virtual void ComputeAndStore_Energy_Internal();
		virtual void ComputeAndStore_Gradient_Internal();
		virtual void ComputeAndStore_Hessian_Internal();

		virtual Real ComputeShape(Tag s = Tag_Position_X);
		virtual Real ComputeDihedral(Tag s = Tag_Position_X);

		inline virtual Real GetRestTheta() const { return this->m_theta0; }
		inline virtual void SetRestTheta(Real t0) { this->m_theta0 = t0; }

		inline virtual Real GetRestShape() const { return this->m_shape0; }
		inline virtual void SetRestShape(Real g0) { this->m_shape0 = g0; }

	protected:
		Real helperDistance(const Vector3d& vx, const Vector3d& va, const Vector3d& vb);
		Vector2d helperBarycentric(const Vector3d& vx, const Vector3d& va, const Vector3d& vb);
	};
}

