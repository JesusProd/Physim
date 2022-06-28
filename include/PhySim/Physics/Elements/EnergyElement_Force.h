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

	class EnergyElement_Force : public EnergyElement
	{

	protected:
		vector<VectorXd> m_vforces;

	public:
		EnergyElement_Force(Simulable* pModel, const vector<KinematicsEle*>& vDoF);
		virtual ~EnergyElement_Force(void);

		virtual void Init();

		virtual const vector<VectorXd>& GetForces() const { return this->m_vforces; }
		virtual void SetForces(const vector<VectorXd>& vf) { this->m_vforces = vf; }

		virtual void ComputeAndStore_Energy_Internal();
		virtual void ComputeAndStore_Gradient_Internal();
	};

}


