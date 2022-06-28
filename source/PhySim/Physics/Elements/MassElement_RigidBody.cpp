//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/MassElement_RigidBody.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Physics/Simulables/Simulable.h>
#include <PhySim/Physics/Simulables/Simulable_RB.h>
#include <PhySim/Kinematics/KEleRigidBody3D.h>

#include <PhySim/Utils/MathUtils.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;


	MassElement_RigidBody::MassElement_RigidBody(Simulable* pModel, Geometry* pGeom, PtrS<ParameterSet> pParam) : MassElement(pModel, pParam)
	{
		this->m_pGeom = pGeom;

		this->m_vDoF.resize(1);
		this->m_vDoF[0] = this->m_pGeom->Traits().Kinematics(Tag::Tag_DOF_0);

		m_vDMDtv = VectorXd::Zero(6);
		m_mMass = MatrixXd::Zero(6, 6);

		this->m_vgradient = VectorXd::Zero(6);
		this->m_mHessian = MatrixXd::Zero(6, 6);
	}

	MassElement_RigidBody::~MassElement_RigidBody(void)
	{
		// Nothing to do here...
	}

	void MassElement_RigidBody::Init()
	{
		Real rho = this->m_pParams->GetParameter(ParameterSet::Param_Density);
		this->m_pGeom->MassProperties(Tag_Position_0, rho, this->m_mass0, this->m_vc0, this->m_mI0); 
	}

	void MassElement_RigidBody::ComputeAndStore_Mass()
	{
		KEleRigidBody3D* pRB = static_cast<KEleRigidBody3D*>(this->m_pGeom->Traits().Kinematics(Tag::Tag_DOF_0));

		Matrix3d mR = pRB->ComputeFullRotation();
		Matrix3d mI = (mR*m_mI0*mR.transpose());

		this->m_mMass.block(0, 0, 3, 3) = Matrix3d::Identity()*m_mass0;
		this->m_mMass.block(3, 3, 3, 3) = mI;

		Vector3d vw = pRB->GetVelocity().segment(3, 3);
		Matrix3d mW;
		MathUtils::crossMatrix(vw, mW);

		this->m_vDMDtv.segment(3, 3) = mW*(mI*vw);
	}

	void MassElement_RigidBody::ComputeAndStore_Energy_Internal()
	{
		KEleRigidBody3D* pRB = static_cast<KEleRigidBody3D*>(m_pGeom->Traits().Kinematics(Tag::Tag_DOF_0));

		this->m_energy = pRB->GetPositionX().segment(0, 3).dot(-this->m_vgravity * m_mass0);
	}

	void MassElement_RigidBody::ComputeAndStore_Gradient_Internal()
	{
		// Constant
		this->m_vgradient.segment(0, 3) = -this->m_vgravity*m_mass0;
	}

}