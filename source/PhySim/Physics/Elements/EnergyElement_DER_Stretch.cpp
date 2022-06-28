//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, IST Austria
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_DER_Stretch.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/Edge.h>

#include <PhySim/Physics/Simulables/Simulable_DER.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	EnergyElement_DER_Stretch::EnergyElement_DER_Stretch(Simulable* pModel, Edge* pEdge) : EnergyElement(pModel)
	{
		this->m_pModelDER = static_cast<Simulable_DER*>(pModel);

		this->m_pEdge = pEdge;

		this->m_vDoF.resize(2);
		this->m_vDoF[0] = pEdge->GetTail()->Traits().Kinematics(Tag_DOF_0);
		this->m_vDoF[1] = pEdge->GetHead()->Traits().Kinematics(Tag_DOF_0);

		this->m_vgradient.resize(6);
		this->m_mHessian.resize(6, 6);

		this->m_mDeDx.resize(3,6);
		this->m_mDeDx.setZero();
		this->m_mDeDx.block(0, 0, 3, 3) = -Matrix3d::Identity();
		this->m_mDeDx.block(0, 3, 3, 3) = Matrix3d::Identity();

		this->Init();
	}

	EnergyElement_DER_Stretch::~EnergyElement_DER_Stretch(void)
	{
		// Nothing to do here...
	}

	void EnergyElement_DER_Stretch::Init()
	{
		this->m_intVolume = this->ComputeLength(Tag_Position_0);
	}

	void EnergyElement_DER_Stretch::ComputeAndStore_Energy_Internal()
	{
		Vector3d vx12 = this->m_pEdge->Vector(Tag_Position_X);

		ParameterSet& parameters = *this->m_pModelDER->MaterialDistribution()->GetValueAtDomainPoint(this->m_pEdge->ID());

		double* x12 = vx12.data();
		Real L0 = this->m_intVolume;
		Real rh = this->m_pEdge->Traits().Double(Tag_Size_0);
		Real rw = this->m_pEdge->Traits().Double(Tag_Size_1);
		Real E = parameters[ParameterSet::Param_Young];

#include "../Maple/RodEdge3DEnergyOPT.mcg"

		this->m_energy = t15;
	}

	void EnergyElement_DER_Stretch::ComputeAndStore_Gradient_Internal()
	{
		Vector3d vx12 = this->m_pEdge->Vector(Tag_Position_X);

		ParameterSet& parameters = *this->m_pModelDER->MaterialDistribution()->GetValueAtDomainPoint(this->m_pEdge->ID());

		double* x12 = vx12.data();
		Real L0 = this->m_intVolume;
		Real rh = this->m_pEdge->Traits().Double(Tag_Size_0);
		Real rw = this->m_pEdge->Traits().Double(Tag_Size_1);
		Real E = parameters[ParameterSet::Param_Young];

		VectorXd vfx(3);

#include "../Maple/RodEdge3DForceOPT.mcg"

		this->m_vgradient = -this->m_mDeDx.transpose()*vfx;
	}

	void EnergyElement_DER_Stretch::ComputeAndStore_Hessian_Internal()
	{
		Vector3d vx12 = this->m_pEdge->Vector(Tag_Position_X);

		ParameterSet& parameters = *this->m_pModelDER->MaterialDistribution()->GetValueAtDomainPoint(this->m_pEdge->ID());

		double* x12 = vx12.data();
		Real L0 = this->m_intVolume;
		Real rh = this->m_pEdge->Traits().Double(Tag_Size_0);
		Real rw = this->m_pEdge->Traits().Double(Tag_Size_1);
		Real E = parameters[ParameterSet::Param_Young];

		Real mJx[3][3];

#include "../Maple/RodEdge3DJacobianOPT.mcg"

		MatrixXd mJxE(3, 3);
		mJxE.setZero();

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; ++j)
			{
				if (isfinite(mJx[i][j]))
					mJxE(i, j) = mJx[i][j];
				else
					IOUtils::logTrace(Verbosity::V1_Default, "In %s: Hessian value is NAN\n", "");
			}

		this->m_mHessian = -this->m_mDeDx.transpose()*mJxE*this->m_mDeDx;
	}

	Real EnergyElement_DER_Stretch::ComputeLength(Tag s)
	{
		if (s != Tag_Position_0 && s != Tag_Position_X)
			throw PhySim::exception("Invalid Tag: [Position_0 | Position_X]");

		return this->m_pEdge->VolumeBasis(s);
	}

}