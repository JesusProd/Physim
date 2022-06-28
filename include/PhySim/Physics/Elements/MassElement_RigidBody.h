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

	class Geometry;
	class Simulable;
	class ParameterSet;

	class MassElement_RigidBody : public MassElement
	{
	protected:
		Geometry* m_pGeom;
		Real m_mass0; // Mass
		Vector3d m_vc0; // COM
		Matrix3d m_mI0; // Inertia

	public:

		MassElement_RigidBody(Simulable* pModel, Geometry* pGeom, PtrS<ParameterSet> pParam);

		virtual ~MassElement_RigidBody(void);

		virtual void Init();

		virtual void ComputeAndStore_Mass();

		virtual void ComputeAndStore_Energy_Internal() override;
		virtual void ComputeAndStore_Gradient_Internal() override;

		virtual int GetHessianSize() const override { return 0; };

		virtual const Real& GetCentroidMass() const { return this->m_mass0; };
		virtual const Matrix3d& GetInertiaMoment() const { return this->m_mI0; };

	};
}
