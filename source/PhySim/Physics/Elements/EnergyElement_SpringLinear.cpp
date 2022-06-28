//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_SpringLinear.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Physics/Simulables/Simulable.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	EnergyElement_SpringLinear::EnergyElement_SpringLinear(Simulable* pModel, Edge* pEdge, ParameterSet* pMaterial) : EnergyElement(pModel)
	{
		this->m_pEdge = pEdge;

		this->m_vDoF.resize(2);
		this->m_vDoF[0] = pEdge->GetTail()->Traits().Kinematics(Tag::Tag_DOF_0);
		this->m_vDoF[1] = pEdge->GetHead()->Traits().Kinematics(Tag::Tag_DOF_0);

		this->m_vgradient.resize(3);
		this->m_mHessian.resize(3, 3);

		this->m_pMaterial = pMaterial;
	}

	EnergyElement_SpringLinear::~EnergyElement_SpringLinear()
	{
		// Nothing to do...
	}

	void EnergyElement_SpringLinear::Init()
	{
		this->m_intVolume = this->m_restLength = this->ComputeRestLength();
	}

	Real EnergyElement_SpringLinear::ComputeRestLength()
	{
		return this->m_pEdge->VolumeBasis(Tag_Position_0);
	}

	void EnergyElement_SpringLinear::ComputeAndStore_Energy_Internal()
	{
		// Get defo nodes

		Vector3d ax = this->m_vDoF[0]->GetValue();
		Vector3d bx = this->m_vDoF[1]->GetValue();

		// Strain

		double s = ((bx - ax).norm() / this->m_restLength - 1);

		// Stiffness

		double k = (*this->m_pMaterial)[ParameterSet::Param_StretchK];

		this->m_energy = 0.5*k*this->m_restLength*s*s;
	}

	void EnergyElement_SpringLinear::ComputeAndStore_Gradient_Internal()
	{
		// Get defo nodes

		Vector3d ax = this->m_vDoF[0]->GetValue();
		Vector3d bx = this->m_vDoF[1]->GetValue();

		// Compute lengths

		double Lx = (bx - ax).norm();

		// Strain

		double s = (Lx / this->m_restLength - 1);

		// Stiffness

		double k = (*this->m_pMaterial)[ParameterSet::Param_StretchK];

		// GradientFull

		Vector3d ub = (bx - ax) / Lx;
		Vector3d gb = k*s*ub;
		this->m_vgradient.block<3, 1>(0, 0) = gb;
	}

	void EnergyElement_SpringLinear::ComputeAndStore_Hessian_Internal()
	{
		// Get defo nodes

		Vector3d ax = this->m_vDoF[0]->GetValue();
		Vector3d bx = this->m_vDoF[1]->GetValue();

		// Compute lengths

		double L0 = this->m_restLength;
		double Lx = (bx - ax).norm();

		// Strain

		double s = (Lx / L0 - 1);

		// Stiffness

		double k = (*this->m_pMaterial)[ParameterSet::Param_StretchK];

		// HessianFull

		Vector3d ub = (bx - ax) / Lx;
		double L0inv = 1.0 / L0;
		double Lxinv = 1.0 / Lx;
		Matrix3d mI = Matrix3d::Identity();
		Matrix3d ububT = ub*ub.transpose();
		this->m_mHessian.block<3, 3>(0, 0) = k*(L0inv*ububT + s*((mI - ububT)*Lxinv));
	}

	//void EnergyElement_SpringLinear::AssembleGlobal_Gradient(VectorXd& vtotalGradient)
	//{
	//	vtotalGradient.block<3, 1>(this->m_vDoF[0]->Offset(), 0) += -this->m_vgradient;
	//	vtotalGradient.block<3, 1>(this->m_vDoF[1]->Offset(), 0) += this->m_vgradient;
	//}

	//void EnergyElement_SpringLinear::AssembleGlobal_Hessian(VectorTd& vtotalHessian)
	//{
	//	int offset0 = this->m_vDoF[0]->Offset();
	//	int offset1 = this->m_vDoF[1]->Offset();

	//	assembleSparseMatrix(vtotalHessian, offset0, offset0, m_mHessian.block<3, 3>(0, 0));
	//	assembleSparseMatrix(vtotalHessian, offset1, offset1, m_mHessian.block<3, 3>(0, 0));

	//	if (offset0 > offset1)
	//	{
	//		assembleSparseMatrix(vtotalHessian, offset0, offset1, -m_mHessian.block<3, 3>(0, 0));
	//	}
	//	else
	//	{
	//		assembleSparseMatrix(vtotalHessian, offset1, offset0, -m_mHessian.block<3, 3>(0, 0));
	//	}
	//}

	//void EnergyElement_SpringLinear::AssembleGlobal_FastPreallocatedHessian()
	//{
	//	int offset0 = this->m_vDoF[0]->Offset();
	//	int offset1 = this->m_vDoF[1]->Offset();

	//	for (int i = 0; i < 3; ++i)
	//	{
	//		for (int j = 0; j < 3; ++j)
	//		{
	//			*m_mHessianP(i, j) += m_mHessian(i, j);
	//		}
	//	}

	//	for (int i = 0; i < 3; ++i)
	//	{
	//		for (int j = 0; j < 3; ++j)
	//		{
	//			*m_mHessianP(3 + i, 3 + j) += m_mHessian(i, j);
	//		}
	//	}
	//		
	//	if (offset0 > offset1)
	//	{
	//		for (int i = 0; i < 3; ++i)
	//		{
	//			for (int j = 0; j < 3; ++j)
	//			{
	//				*m_mHessianP(i, 3 + j) += -m_mHessian(i, j);
	//			}
	//		}
	//	}
	//	else
	//	{
	//		for (int i = 0; i < 3; ++i)
	//		{
	//			for (int j = 0; j < 3; ++j)
	//			{
	//				*m_mHessianP(3 + i, j) += -m_mHessian(i, j);
	//			}
	//		}
	//	}
	//}

}