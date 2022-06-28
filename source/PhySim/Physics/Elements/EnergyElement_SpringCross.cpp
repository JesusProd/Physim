//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_SpringCross.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Physics/Simulables/Simulable.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	EnergyElement_SpringCross::EnergyElement_SpringCross(Simulable* pModel, Edge* pEdge0, Edge* pEdge1, ParameterSet* pMaterial) : EnergyElement(pModel)
	{
		this->m_pEdge0 = pEdge0;
		this->m_pEdge1 = pEdge1;

		this->m_vDoF.resize(4);
		this->m_vDoF[0] = pEdge0->GetTail()->Traits().Kinematics(Tag::Tag_DOF_0);
		this->m_vDoF[1] = pEdge0->GetHead()->Traits().Kinematics(Tag::Tag_DOF_0);
		this->m_vDoF[2] = pEdge1->GetTail()->Traits().Kinematics(Tag::Tag_DOF_0);
		this->m_vDoF[3] = pEdge1->GetHead()->Traits().Kinematics(Tag::Tag_DOF_0);

		this->m_vgradient.resize(12);
		this->m_mHessian.resize(12, 12);

		this->m_pMaterial = pMaterial;
	}

	EnergyElement_SpringCross::~EnergyElement_SpringCross()
	{
		// Nothing to do...
	}

	void EnergyElement_SpringCross::Init()
	{
		this->m_restStrain = this->ComputeStrain(Tag_Position_0);
		this->m_intVolume = (this->m_pEdge0->VolumeBasis(Tag_Position_0) +
						     this->m_pEdge1->VolumeBasis(Tag_Position_0));
	}

	Real EnergyElement_SpringCross::ComputeStrain(Tag s)
	{
		Vector3d a0 = this->m_vDoF[0]->GetValue();
		Vector3d b0 = this->m_vDoF[1]->GetValue();
		Vector3d c0 = this->m_vDoF[2]->GetValue();
		Vector3d d0 = this->m_vDoF[3]->GetValue();
		Real L0 = (b0 - a0).norm();
		Real L1 = (d0 - c0).norm();
		return L0 - L1;
	}

	void EnergyElement_SpringCross::ComputeAndStore_Energy_Internal()
	{
		Vector3d x1 = this->m_vDoF[0]->GetValue();
		Vector3d x2 = this->m_vDoF[1]->GetValue();
		Vector3d x3 = this->m_vDoF[2]->GetValue();
		Vector3d x4 = this->m_vDoF[3]->GetValue();

		Real K = (*this->m_pMaterial)[ParameterSet::Param_ShearK];
		Real L0 = m_intVolume;
		Real R0 = m_restStrain;

		{
#include "../Maple/SpringCrossEnergy.mcg"

			this->m_energy = t21;
		}
	}

	void EnergyElement_SpringCross::ComputeAndStore_Gradient_Internal()
	{
		Vector3d x1 = this->m_vDoF[0]->GetValue();
		Vector3d x2 = this->m_vDoF[1]->GetValue();
		Vector3d x3 = this->m_vDoF[2]->GetValue();
		Vector3d x4 = this->m_vDoF[3]->GetValue();

		Real K = (*this->m_pMaterial)[ParameterSet::Param_ShearK];
		Real L0 = m_intVolume;
		Real R0 = m_restStrain;

		Real vgx[12];

		{
#include "../Maple/SpringCrossGradient.mcg"
		}

		for (int i = 0; i < 12; ++i)
			m_vgradient(i) = vgx[i];
	}

	void EnergyElement_SpringCross::ComputeAndStore_Hessian_Internal()
	{
		Vector3d x1 = this->m_vDoF[0]->GetValue();
		Vector3d x2 = this->m_vDoF[1]->GetValue();
		Vector3d x3 = this->m_vDoF[2]->GetValue();
		Vector3d x4 = this->m_vDoF[3]->GetValue();

		Real K = (*this->m_pMaterial)[ParameterSet::Param_ShearK];
		Real L0 = m_intVolume;
		Real R0 = m_restStrain;

		Real mHx[12][12];

		{
#include "../Maple/SpringCrossHessian.mcg"
		}

		for (int i = 0; i < 12; ++i)
			for (int j = 0; j < 12; ++j)
				m_mHessian(i, j) = mHx[i][j];
	}

}