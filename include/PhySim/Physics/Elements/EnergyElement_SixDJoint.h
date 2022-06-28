#pragma once

#include <PhySim/CommonIncludes.h>


#include <PhySim/Physics/Elements/EnergyElement.h>
#include <PhySim/Utils/Utils.h>
#include <PhySim/Kinematics/KEleRigidBody3D.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class EnergyElement_SixDJoint :		public EnergyElement
	{
	protected:


		PtrS<KEleRigidBody3D> m_pRBp, m_pRBc;	// the two rigids that the joint is defined between them, parent and child
		MatrixXd m_K;					// 6x6 matrix with stiffness, it can be vector also
		Matrix3d m_R0;					//the orientation of the joint, defined by us
		Matrix3d m_Rp, m_Rc;			// the orientation of the parent and child rigid body

		Vector3d m_rp, m_rc, m_jointCenter, m_xNodep, m_xNodec ;			// vector of rigid cm to joint point, and the coordinates of the node thatbelongs to the joint
	
		//PtrS<ParameterSet> m_pMaterial;

	public:
		EnergyElement_SixDJoint(Simulable* pModel, PtrS<KEleRigidBody3D> pRBp, PtrS<KEleRigidBody3D> pRBc, MatrixXd K);
		EnergyElement_SixDJoint(Simulable* pModel, PtrS<KEleRigidBody3D> pRBp, PtrS<KEleRigidBody3D> pRBc, MatrixXd K, Matrix3d R0, Vector3d jointCenter);

		virtual ~EnergyElement_SixDJoint(void);

		virtual void Init();
		virtual void UpdateMechanics() override;
		//virtual Real ComputeRestLength();
		//inline virtual Real GetRestLength() const { return this->m_restLength; }
		//inline virtual void SetRestLength(Real rl) { this->m_restLength = rl; }



		virtual void ComputeAndStore_Energy_Internal();
		virtual void ComputeAndStore_Energy_Internal_Only_Translation();
		virtual void ComputeAndStore_Gradient_Internal();
		virtual void ComputeAndStore_Gradient_Internal_Only_Translation();
		virtual void ComputeAndStore_Hessian_Internal();
		virtual void SetParameters(MatrixXd K) { m_K = K;}
		virtual MatrixXd GetParameters() { return m_K; }

		//inline virtual PtrS<ParameterSet> GetMaterial() { return this->m_pMaterial; }

	};
}