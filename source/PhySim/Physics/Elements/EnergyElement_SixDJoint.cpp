
//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Christos Koutras, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_SixDJoint.h>

#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Utils/GeometryUtils.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	EnergyElement_SixDJoint::EnergyElement_SixDJoint(Simulable* pModel, PtrS<KEleRigidBody3D> pRBp, PtrS<KEleRigidBody3D> pRBc, MatrixXd K) : EnergyElement(pModel)
	{

		this->m_vDoF.resize(2);
		this->m_vDoF[0] = pRBp.get();
		this->m_vDoF[1] = pRBc.get();
		this->m_vgradient.resize(12);
		this->m_mHessian.resize(12, 12);
		this->m_pRBp = pRBp;
		this->m_pRBc = pRBc;
		this->m_K = K;
		this->m_R0.setZero();
		this->m_jointCenter.setZero();
		this->m_Rp = pRBp->ComputeFullRotation();
		this->m_Rc = pRBc->ComputeFullRotation();
	}

	EnergyElement_SixDJoint::EnergyElement_SixDJoint(Simulable* pModel, PtrS<KEleRigidBody3D> pRBp, PtrS<KEleRigidBody3D> pRBc, 
		 MatrixXd K, Matrix3d R0, Vector3d jointCenter) : EnergyElement(pModel)
	{

		this->m_vDoF.resize(2);
		this->m_vDoF[0] = pRBp.get();
		this->m_vDoF[1] = pRBc.get();
		this->m_vgradient.resize(12);
		this->m_mHessian.resize(12, 12);
		this->m_pRBp = pRBp;
		this->m_pRBc = pRBc;
		this->m_K = K;
		this->m_R0 = R0;
		this->m_jointCenter = jointCenter;
		this->m_Rp = pRBp->ComputeFullRotation();
		this->m_Rc = pRBc->ComputeFullRotation();
	}

	EnergyElement_SixDJoint::~EnergyElement_SixDJoint()
	{
		// Nothing to do...
	}

	void EnergyElement_SixDJoint::Init()
	{
		// Initialize stuff that has to do with undeformed configuration
		if (m_jointCenter.norm()< 1e-6) {
			this->m_jointCenter = 0.5 * (m_vDoF[0]->GetValue().segment(0, 3) + m_vDoF[1]->GetValue().segment(0, 3));
		}
		m_rp = m_jointCenter - m_pRBp->GetPositionX().segment(0, 3);
		m_rc = m_jointCenter - m_pRBc->GetPositionX().segment(0, 3);			//carefull here with the signs, not sure if I should change something

		if (m_R0.norm() < 1e-6) {									// calculate R0, orientation of the joints between vertebra
			Vector3d pos1, pos2, newY, newX, newZ, xvec, projection;
			pos1 = m_pRBc->GetPositionX().segment(0, 3);
			//std::cout << "pos1 = " << pos1 << std::endl;		
			pos2 = m_pRBp->GetPositionX().segment(0, 3);
			//std::cout << "pos2 = " << pos2 << std::endl;
			if ((pos1 - pos2).norm()<= 1e-5) {
				newX = Vector3d(1.0, 0.0, 0.0);
				newY = Vector3d(0.0, 1.0, 0.0);				
				newZ = Vector3d(0.0, 0.0, 1.0);
			}
			else if ((pos1[1] - pos2[1]) * (pos1[1] - pos2[1]) <= 1e-5 ) {
				newY = Vector3d(0.0, 1.0, 0.0);
				newX = pos2 - pos1;
				newX = newX / newX.norm();
				newZ = newX.cross(newY);
			}
			else {
				newY = pos1 - pos2;								//here because of the two cubes next to each other, newX and newY are exactly the same
				newY = newY / newY.norm();						// this is not the case in the real case, so I do not try to fix the problem now

				//std::cout << "newY = " << newY << std::endl;
				xvec = Vector3d(1.0, 0.0, 0.0);
				//std::cout<< "gia na doume edo" <<(xvec.transpose() * newY) * (1 / (newY.norm() * newY.norm()))<<std::endl;
				projection = xvec.dot(newY)  * (1 / (newY.norm() * newY.norm())) * newY;
				newX = xvec - projection;
				newX = newX / newX.norm();
				newZ = newX.cross(newY);
			}
			Matrix3d rotmat;
			rotmat.row(0) = newX;
			rotmat.row(1) = newY;
			rotmat.row(2) = newZ;    //CAREFULL here R0 is in global coordinates, in the paper it is in local so I need to transform it
			//std::cout << "rotation matrix rotmat" << rotmat << std::endl;
			Matrix3d Rp = m_pRBp->ComputeFullRotation();
			this->m_R0 = Rp.transpose() * rotmat;				// here I transfer R0 to the parent coordinates
			//std::cout << "rotation matrix m_R0 = "<<m_R0 << std::endl;
		}
		//m_R0.setIdentity();		/////	TO COMMENT NOT CORRECT
		/*m_rp = Vector3d(0.05, -0.05, -0.05);
		m_rc = Vector3d(-0.05, +0.05, +0.05);*/

		//m_rp = Vector3d(0.05, -0.0, +0.0);
		//m_rc = Vector3d(-0.05, +0.05, +0.05);
		// need to add R0 here, R0 is the initial orientation of the frame. For the moment I am passing it from outside but
		// it can be computed here. It depends on the center of masses of the rigids.
	}

	void EnergyElement_SixDJoint::UpdateMechanics()
	{

		this->m_Rp = m_pRBp->ComputeFullRotation();
		this->m_Rc = m_pRBc->ComputeFullRotation();


		Vector3d posParent = m_pRBp->GetPositionX().segment(0, 3);
		Vector3d poschild = m_pRBc->GetPositionX().segment(0, 3);

		m_xNodep = m_pRBp->GetPositionX().segment(0, 3) + m_Rp * m_rp;		//node of the joint belonging to parent
 		m_xNodec = m_pRBc->GetPositionX().segment(0, 3) + m_Rc * m_rc;		//node of the joint belonging to child

	}

	void EnergyElement_SixDJoint::ComputeAndStore_Energy_Internal()
	{
		// Strain

		//double s = ((bx - ax).norm() / this->m_restLength - 1);
		Vector3d dx = m_R0.transpose() * m_Rp.transpose() * (m_xNodec - m_xNodep);
		Vector3d dthita = GeometryUtils::rotationMatrixToAxisAngle(m_R0.transpose() * m_Rp.transpose() * m_Rc * m_R0);   // I need to transform this to axis angle

		this->m_energy = 0.5 * dx.transpose() * m_K.block(0, 0, 3, 3) * dx;
		this->m_energy += 0.5 * dthita.transpose() * m_K.block(3, 3, 3, 3) * dthita;
	}

	void EnergyElement_SixDJoint::ComputeAndStore_Energy_Internal_Only_Translation()
	{
		// Strain
		Vector3d dx = m_R0.transpose() * m_Rp.transpose() * (m_xNodec - m_xNodep);
		Vector3d dthita = GeometryUtils::rotationMatrixToAxisAngle(m_R0.transpose() * m_Rp.transpose() * m_Rc * m_R0);   // I need to transform this to axis angle

		this->m_energy = 0.5 * dx.transpose() * m_K.block(0, 0, 3, 3) * dx;
	}


	void EnergyElement_SixDJoint::ComputeAndStore_Gradient_Internal()
	{
		// Strain

		Vector3d dx = m_R0.transpose() * m_Rp.transpose() * (m_xNodec - m_xNodep);
		Vector3d dphi = GeometryUtils::rotationMatrixToAxisAngle(m_R0.transpose() * m_Rp.transpose() * m_Rc * m_R0);   // same as dthita above, change to be consistent with the axis angle .doc
		
		// Translational part
		Matrix3d ddxdxc = m_R0.transpose() * m_Rp.transpose();
		Vector3d Fc = ddxdxc.transpose() * m_K.block(0, 0, 3, 3) * dx;  
		Vector3d Fp = - Fc;

		//rotational part

		Matrix3d phiInRotMat = GeometryUtils::rotationAxisAngleToMatrix(dphi);
		Matrix3d I, MtoInv;
		I.setIdentity();
		Eigen::Matrix3d dphi_hat;
		dphi_hat << 0, -dphi(2), dphi(1),
			dphi(2), 0, -dphi(0),
			-dphi(1), dphi(0), 0;

		Vector3d Tc, Tp;
		MtoInv = phiInRotMat - I + dphi * dphi.transpose();
		if (MtoInv.norm() > 1e-8)
		{
			Matrix3d dphidthita = MtoInv.inverse() * (dphi_hat + dphi * dphi.transpose());  
			Tp = -m_R0 * dphidthita.transpose() * m_K.block(3, 3, 3, 3) * dphi;// - m_Rp * dphidthita.transpose() * m_K.block(3, 3, 3, 3) * dphi;
			Tc = -Tp;
			//Tp = m_K(3,3) * dphi.transpose() * (- m_Rp);
			//Tc = -Tp;

		}
		else {
			Tc.setZero();
			Tp.setZero();
		}


		// extra rotational part. derivative of eq 6 in paper with respect to rot dofs inludes both parts
		Vector3d thitap = m_pRBp->GetPosition0().segment(3, 3);
		Matrix3d Rp0 = m_pRBp->GetCachedRotation();
		Matrix3d Rc0 = m_pRBc->GetCachedRotation();
		Matrix3d ddxdthitap, ddxdthitac, eye, skewRc0rc, skewThitap;
		Vector3d rc_hat;
		eye.setIdentity();
		rc_hat = Rc0 * m_rc;

		skewThitap << 0, -thitap(2), thitap(1),
			thitap(2), 0, -thitap(0),
			-thitap(1), thitap(0), 0;

		skewRc0rc << 0, -rc_hat(2), rc_hat(1),
			rc_hat(2), 0, -rc_hat(0),
			-rc_hat(1), rc_hat(0), 0;

		//ddxdthitac = - m_R0.transpose() * Rp0.transpose() * (eye - skewThitap) * skewRc0rc;	
		ddxdthitac = - m_R0.transpose() * m_Rp.transpose() * skewRc0rc;	
		Vector3d extraTc = ddxdthitac.transpose() * m_K.block(0, 0, 3, 3) * dx;



		Vector3d b, rp_hat;
		Matrix3d skewb, skewRp0rp, Rthitap;
		Rthitap = GeometryUtils::rotationAxisAngleToMatrix(thitap);
		rp_hat = Rp0 * m_rp;
		skewRp0rp << 0, -rp_hat(2), rp_hat(1),
			rp_hat(2), 0, -rp_hat(0),
			-rp_hat(1), rp_hat(0), 0;

		b = m_xNodec - m_xNodep;
		skewb << 0, -b(2), b(1),
			b(2), 0, -b(0),
			-b(1), b(0), 0;


		ddxdthitap = m_R0.transpose() * m_Rp.transpose() * skewRp0rp + m_R0.transpose() * Rp0.transpose() * skewb;
		Vector3d extraTp = ddxdthitap.transpose() * m_K.block(0, 0, 3, 3) * dx;


		// GradientFull		
		m_vgradient.setZero();
		this->m_vgradient.segment(0, 3) = Fp;
		this->m_vgradient.segment(3, 3) = extraTp + Tp;
		this->m_vgradient.segment(6, 3) = Fc ;
		this->m_vgradient.segment(9, 3) = extraTc +Tc;

		/*VectorXd fakegradient;
		fakegradient.resize(12);
		fakegradient.setZero();
		this->m_vgradient = fakegradient;*/
}


	void EnergyElement_SixDJoint::ComputeAndStore_Gradient_Internal_Only_Translation()
	{
		// Strain

		Vector3d dx = m_R0.transpose() * m_Rp.transpose() * (m_xNodec - m_xNodep);
		Vector3d dphi = GeometryUtils::rotationMatrixToAxisAngle(m_R0.transpose() * m_Rp.transpose() * m_Rc * m_R0);   // same as dthita above, change to be consistent with the axis angle .doc

		// Translational part
		Matrix3d ddxdxc = m_R0.transpose() * m_Rp.transpose();
		Vector3d Fc = ddxdxc.transpose() * m_K.block(0, 0, 3, 3) * dx;
		Vector3d Fp = -Fc;

		//rotational part

		Matrix3d phiInRotMat = GeometryUtils::rotationAxisAngleToMatrix(dphi);
		Matrix3d I, MtoInv;

		// extra rotational part. derivative of eq 6 in paper with respect to rot dofs inludes both parts
		Vector3d thitap = m_pRBp->GetPosition0().segment(3, 3);
		Matrix3d Rp0 = m_pRBp->GetCachedRotation();
		Matrix3d Rc0 = m_pRBc->GetCachedRotation();
		Matrix3d ddxdthitap, ddxdthitac, eye, skewRc0rc, skewThitap;
		Vector3d rc_hat;
		eye.setIdentity();
		rc_hat = Rc0 * m_rc;

		skewThitap << 0, -thitap(2), thitap(1),
			thitap(2), 0, -thitap(0),
			-thitap(1), thitap(0), 0;
		skewRc0rc << 0, -rc_hat(2), rc_hat(1),
			rc_hat(2), 0, -rc_hat(0),
			-rc_hat(1), rc_hat(0), 0;

		//ddxdthitac = - m_R0.transpose() * Rp0.transpose() * (eye - skewThitap) * skewRc0rc;	
		ddxdthitac = -m_R0.transpose() * m_Rp.transpose() * skewRc0rc;
		Vector3d extraTc = ddxdthitac.transpose() * m_K.block(0, 0, 3, 3) * dx;

		Vector3d b, rp_hat;
		Matrix3d skewb, skewRp0rp, Rthitap;
		Rthitap = GeometryUtils::rotationAxisAngleToMatrix(thitap);
		rp_hat = Rp0 * m_rp;
		skewRp0rp << 0, -rp_hat(2), rp_hat(1),
			rp_hat(2), 0, -rp_hat(0),
			-rp_hat(1), rp_hat(0), 0;

		b = m_xNodec - m_xNodep;
		skewb << 0, -b(2), b(1),
			b(2), 0, -b(0),
			-b(1), b(0), 0;
		ddxdthitap = m_R0.transpose() * m_Rp.transpose() * skewRp0rp + m_R0.transpose() * Rp0.transpose() * skewb;
		Vector3d extraTp = ddxdthitap.transpose() * m_K.block(0, 0, 3, 3) * dx;


		// GradientFull		
		m_vgradient.setZero();
		this->m_vgradient.segment(0, 3) = Fp;
		this->m_vgradient.segment(3, 3) = extraTp;
		this->m_vgradient.segment(6, 3) = Fc;
		this->m_vgradient.segment(9, 3) = extraTc;
	}


	void EnergyElement_SixDJoint::ComputeAndStore_Hessian_Internal()
	{

		Vector3d dx = m_R0.transpose() * m_Rp.transpose() * (m_xNodec - m_xNodep);
		Vector3d dphi = GeometryUtils::rotationMatrixToAxisAngle(m_R0.transpose() * m_Rp.transpose() * m_Rc * m_R0);   // same as dthita above, change to be consistent with the axis angle .doc

		// Translational part
		//Matrix3d ddxdxc = m_R0.transpose() * m_Rp.transpose();
		//Matrix3d thc = ddxdxc.transpose() * m_K.block(0, 0, 3, 3) * ddxdxc;     // need to think about this minus
		////
		// for avoiding many calculations
		Matrix3d Rp0 = m_pRBp->GetCachedRotation();
		Matrix3d Rc0 = m_pRBc->GetCachedRotation();

		Matrix3d R0tRpt = m_R0.transpose() * m_Rp.transpose();
		Matrix3d RpR0K = m_Rp * m_R0 * m_K.block(0, 0, 3, 3);
		Matrix3d skewRp0rp, skewRc0rc, skewb, skewRp0R0KDx, skewRp0RpKDx;
		Vector3d Rc0rc, Rp0rp, b, Rp0R0KDx, Rp0RpKDx;
		Rp0rp = Rp0 * m_rp;
		Rc0rc = Rc0 * m_rc;
		skewRp0rp << 0, -Rp0rp(2), Rp0rp(1),
			Rp0rp(2), 0, -Rp0rp(0),
			-Rp0rp(1), Rp0rp(0), 0;
		skewRc0rc << 0, -Rc0rc(2), Rc0rc(1),
			Rc0rc(2), 0, -Rc0rc(0),
			-Rc0rc(1), Rc0rc(0), 0;
		b = m_xNodec - m_xNodep;
		skewb << 0, -b(2), b(1),
			b(2), 0, -b(0),
			-b(1), b(0), 0;
		Rp0R0KDx = Rp0 * m_R0 * m_K.block(0, 0, 3, 3) * dx;
		skewRp0R0KDx << 0, -Rp0R0KDx(2), Rp0R0KDx(1),
			Rp0R0KDx(2), 0, -Rp0R0KDx(0),
			-Rp0R0KDx(1), Rp0R0KDx(0), 0;

	/*	Rp0Rp0R0KDx = Rp0 * Rp0 * m_R0 * m_K.block(0, 0, 3, 3) * dx;
		skewRp0Rp0R0KDx << 0, -Rp0Rp0R0KDx(2), Rp0Rp0R0KDx(1),
			Rp0Rp0R0KDx(2), 0, -Rp0Rp0R0KDx(0),
			-Rp0Rp0R0KDx(1), Rp0Rp0R0KDx(0), 0;*/

	/*	Rp0RpKDx = Rp0 * m_Rp * m_K.block(0, 0, 3, 3) * dx;
		skewRp0RpKDx << 0, -Rp0RpKDx(2), Rp0RpKDx(1),
			Rp0RpKDx(2), 0, -Rp0RpKDx(0),
			-Rp0RpKDx(1), Rp0RpKDx(0), 0;*/

		Matrix3d d2Edxcxc = RpR0K * R0tRpt;
		// d2Edxpxp = d2Edxcxc
		Matrix3d d2Edxcxp = -d2Edxcxc;
		// d2Edxpxc = d2Edxcxp
		Matrix3d d2Edxcthitac = RpR0K * R0tRpt *( -skewRc0rc);
		Matrix3d d2Edxpthitac = -d2Edxcthitac;
		Matrix3d d2Edxcthitap = RpR0K * (R0tRpt * skewRp0rp + m_R0.transpose() * Rp0.transpose() * skewb) - skewRp0R0KDx;
		Matrix3d d2Edxpthitap = -d2Edxcthitap;
		// d2Edthitapxp = d2Edxpthitap

		Matrix3d d2Edthitacthitac = skewRc0rc * RpR0K * R0tRpt * (-skewRc0rc);
		Matrix3d d2Edthitacthitap = skewRc0rc * (-skewRp0R0KDx) + skewRc0rc * RpR0K *
			(R0tRpt * skewRp0rp + m_R0.transpose() * Rp0.transpose() * skewb);

		// d2Ethitapthitac = d2Ethitacthitap^t
		Matrix3d d2Edthitapthitap = -skewRp0rp * (-skewRp0R0KDx)
			+ skewRp0R0KDx * skewRp0rp
			+ (-skewRp0rp * m_Rp - skewb * Rp0) * m_R0 * m_K.block(0, 0, 3, 3) * 
			(R0tRpt * skewRp0rp + m_R0.transpose() * Rp0.transpose() * skewb);




		//rotational part
		Matrix3d phiInRotMat = GeometryUtils::rotationAxisAngleToMatrix(dphi);
		Matrix3d I, MtoInv;
		I.setIdentity();
		Eigen::Matrix3d dphi_hat;
		dphi_hat << 0, -dphi(2), dphi(1),
			dphi(2), 0, -dphi(0),
			-dphi(1), dphi(0), 0;

		Matrix3d rhc;
		MtoInv = phiInRotMat - I + dphi * dphi.transpose();
		if (MtoInv.norm() > 1e-8)
		{
			Matrix3d dphidthita = MtoInv.inverse() * (dphi_hat + dphi * dphi.transpose());
			//rhc = -m_Rp * dphidthita.transpose() * m_K.block(3, 3, 3, 3) * dphidthita  * (-m_Rp.transpose());
			rhc = -m_R0 * dphidthita.transpose() * m_K.block(3, 3, 3, 3) * dphidthita *(-m_R0.transpose());
			//rhc = m_K.block(3, 3, 3, 3);// *dphidthita;
		}
		else {
			rhc.setZero();		}



		m_mHessian.setZero();
		this->m_mHessian.block<3, 3>(0, 0) += d2Edxcxc;
		this->m_mHessian.block<3, 3>(6, 6) += d2Edxcxc;
		this->m_mHessian.block<3, 3>(0, 6) += d2Edxcxp;
		this->m_mHessian.block<3, 3>(6, 0) += d2Edxcxp;



		this->m_mHessian.block<3, 3>(0, 3) += d2Edxpthitap;
		this->m_mHessian.block<3, 3>(0, 9) += d2Edxpthitac;

		this->m_mHessian.block<3, 3>(3, 0) += d2Edxpthitap.transpose();
		this->m_mHessian.block<3, 3>(3, 3) += d2Edthitapthitap;
		this->m_mHessian.block<3, 3>(3, 6) += d2Edxcthitap.transpose();
		this->m_mHessian.block<3, 3>(3, 9) += d2Edthitacthitap.transpose();

		this->m_mHessian.block<3, 3>(6, 3) += d2Edxcthitap;
		this->m_mHessian.block<3, 3>(6, 9) += d2Edxcthitac;

		this->m_mHessian.block<3, 3>(9, 0) += d2Edxpthitac.transpose();
		this->m_mHessian.block<3, 3>(9, 3) += d2Edthitacthitap;
		this->m_mHessian.block<3, 3>(9, 6) += d2Edxcthitac.transpose();
		this->m_mHessian.block<3, 3>(9, 9) += d2Edthitacthitac;




		this->m_mHessian.block<3, 3>(3, 3) += rhc;
		this->m_mHessian.block<3, 3>(9, 9) += rhc;
		this->m_mHessian.block<3, 3>(3, 9) += -rhc;
		this->m_mHessian.block<3, 3>(9, 3) += -rhc;

	}
}