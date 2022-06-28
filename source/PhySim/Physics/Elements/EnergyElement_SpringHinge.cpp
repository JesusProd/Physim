//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_SpringHinge.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Physics/Simulables/Simulable.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	EnergyElement_SpringHinge::EnergyElement_SpringHinge(Simulable* pModel, Edge* pEdge0, Edge* pEdge1, ParameterSet* pMaterial) : EnergyElement(pModel)
	{
		this->m_pEdge0 = pEdge0;
		this->m_pEdge1 = pEdge1;

		this->m_vDoF.resize(3);
		this->m_vDoF[0] = pEdge0->GetTail()->Traits().Kinematics(Tag::Tag_DOF_0);
		this->m_vDoF[1] = pEdge0->GetHead()->Traits().Kinematics(Tag::Tag_DOF_0);
		this->m_vDoF[2] = pEdge1->GetHead()->Traits().Kinematics(Tag::Tag_DOF_0);

		this->m_vgradient.resize(9);
		this->m_mHessian.resize(9, 9);

		this->m_pMaterial = pMaterial;
	}

	EnergyElement_SpringHinge::~EnergyElement_SpringHinge()
	{
		// Nothing to do...
	}

	void EnergyElement_SpringHinge::Init()
	{
		this->m_restAngle = this->ComputeRestAngle();
		this->m_intVolume = 0.5*(this->m_pEdge0->VolumeBasis(Tag_Position_0) + 
								 this->m_pEdge1->VolumeBasis(Tag_Position_0));
	}

	Real EnergyElement_SpringHinge::ComputeRestAngle()
	{
		Vector3d a0 = this->m_vDoF[0]->GetValue();
		Vector3d b0 = this->m_vDoF[1]->GetValue();
		Vector3d c0 = this->m_vDoF[2]->GetValue();
		Vector3d n0 = (b0 - a0).normalized();
		Vector3d n1 = (c0 - b0).normalized();
		return acos(n0.dot(n1));
	}

	void EnergyElement_SpringHinge::ComputeAndStore_Energy_Internal()
	{
		Vector3d x1 = this->m_vDoF[0]->GetValue();
		Vector3d x2 = this->m_vDoF[1]->GetValue();
		Vector3d x3 = this->m_vDoF[2]->GetValue();

		Real K = (*this->m_pMaterial)[ParameterSet::Param_BendingK];
		Real L0 = m_intVolume;
		Real A0 = m_restAngle;

		{
#include "../Maple/SpringHingeEnergy.mcg"
		
			this->m_energy = t37;
		}
	}

	void EnergyElement_SpringHinge::ComputeAndStore_Gradient_Internal()
	{
		Vector3d x1 = this->m_vDoF[0]->GetValue();
		Vector3d x2 = this->m_vDoF[1]->GetValue();
		Vector3d x3 = this->m_vDoF[2]->GetValue();

		Real K = (*this->m_pMaterial)[ParameterSet::Param_BendingK];
		Real L0 = m_intVolume;
		Real A0 = m_restAngle;

		Real vgx[9];

		{
#include "../Maple/SpringHingeGradient.mcg"
		}

		for (int i = 0; i < 9; ++i)
			m_vgradient(i) = vgx[i];
	}

	void EnergyElement_SpringHinge::ComputeAndStore_Hessian_Internal()
	{
		Vector3d x1 = this->m_vDoF[0]->GetValue();
		Vector3d x2 = this->m_vDoF[1]->GetValue();
		Vector3d x3 = this->m_vDoF[2]->GetValue();

		Real K = (*this->m_pMaterial)[ParameterSet::Param_BendingK];
		Real L0 = m_intVolume;
		Real A0 = m_restAngle;

		Real mHx[9][9];

		{
#include "../Maple/SpringHingeHessian.mcg"
		}

		for (int i = 0; i < 9; ++i)
			for (int j = 0; j < 9; ++j)
				m_mHessian(i,j) = mHx[i][j];
	}

}