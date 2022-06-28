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


#include <PhySim/Physics/Boundary/BCondition.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class BC_Gravity : public BCondition
	{
	public:
		BC_Gravity(Simulable* pModel, const Vector3d& vg);
		virtual ~BC_Gravity(void);

		inline virtual string Name() const { return "Gravity"; };

		inline virtual VectorXd& Gravity() { return this->m_vini[0]; }

		virtual void Init() override;
		virtual void Update() override;

	};
}
