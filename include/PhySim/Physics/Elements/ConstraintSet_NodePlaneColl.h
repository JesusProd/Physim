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


#include <PhySim/Physics/Elements/ConstraintSet.h>
#include <PhySim/Kinematics/KEleParticle3D.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class ConstraintSet_NodePlaneColl : public ConstraintSet
	{
	protected:

		Vector3d m_vpoints;
		Vector3d m_vn;
		Real m_tol;

	public:

		ConstraintSet_NodePlaneColl(Simulable* pModel, bool isSoft, KEleParticle3D* pDOF, const Vector3d& vp, const Vector3d& vn, Real tol);

		virtual ~ConstraintSet_NodePlaneColl(void);

		inline virtual Vector3d& PlaneNormal() { return this->m_vn; }

		virtual void Init();

		virtual void ProjectConstraint();
		virtual void ComputeAndStore_Values();
		virtual void ComputeAndStore_Jacobian();
		virtual void ComputeAndStore_Hessian();

	};
}