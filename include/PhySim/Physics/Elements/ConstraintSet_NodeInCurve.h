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
#include <PhySim/Geometry/Meshes/Curve.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class ConstraintSet_NodeInCurve : public ConstraintSet
	{
	protected:

		PtrS<Curve> m_pCurve;
		//Node* m_pEmbedding;

		// Cached 
		Matrix3d m_mP;
		Vector3d m_v0;
		Vector3d m_v1;
		Vector3d m_vt;

	public:

		ConstraintSet_NodeInCurve(Simulable* pModel, bool isSoft, KEleParticle3D* pDOF, PtrS<Curve> pCurve);

		virtual ~ConstraintSet_NodeInCurve(void);

		inline virtual PtrS<Curve> TargetCurve() { return this->m_pCurve; }

		virtual void Init();

		virtual void ProjectConstraint();
		virtual void ComputeAndStore_Values();
		virtual void ComputeAndStore_Jacobian();
		virtual void ComputeAndStore_Hessian();

	};
}