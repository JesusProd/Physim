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

	class EnergyElement_Force;

	class BC_Force : public BCondition
	{
	protected:
		vector<KinematicsEle*>				m_vpDoF;

		PtrS<EnergyElement_Force>		m_pEleForce;

	public:
		BC_Force(Simulable* pModel, const vector<KinematicsEle*>& vpDoF, const VectorXd& vf);
		BC_Force(Simulable* pModel, const vector<KinematicsEle*>& vpDoF, const vector<VectorXd>& vf);
		
		virtual ~BC_Force(void);

		inline virtual string Name() const { return "Force"; };

		virtual void Init() override;
		virtual void Update() override;

	};
}
