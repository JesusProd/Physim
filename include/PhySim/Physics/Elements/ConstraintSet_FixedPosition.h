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

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class ConstraintSet_FixedPosition : public ConstraintSet
	{
	protected:

		VectorXd m_vt;

	public:

		ConstraintSet_FixedPosition(Simulable* pModel, KinematicsEle* pDoF, const VectorXd& vt);

		virtual ~ConstraintSet_FixedPosition(void);

		inline virtual VectorXd& Target() { return this->m_vt; }

		virtual void Init();

		virtual void ProjectConstraint();
		virtual void ComputeAndStore_Values();
		virtual void ComputeAndStore_Jacobian();

	};
}