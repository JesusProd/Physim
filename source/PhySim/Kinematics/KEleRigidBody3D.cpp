//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC MAdrid
//
//==========================================================

#include <PhySim/Kinematics/KEleRigidBody3D.h>

#include <PhySim/Geometry/Geometry.h>

#include <PhySim/Utils/GeometryUtils.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	PtrS<KinematicsEle> KEleRigidBody3D::Clone()
	{
		return PtrS<KinematicsEle>(new KEleRigidBody3D(this->GetModel(), this->m_pGeometry));
	}

	void KEleRigidBody3D::Initialize()
	{
		// Position

		if (!this->m_pGeometry->Traits().HasTrait(Tag_Position_X))
			this->m_pGeometry->Traits().AddTrait(Tag_Position_X, this->m_pGeometry->Centroid(Tag::Tag_Position_X));

		if (!this->m_pGeometry->Traits().HasTrait(Tag_Position_0))
			this->m_pGeometry->Traits().AddTrait(Tag_Position_0, this->m_pGeometry->Centroid(Tag::Tag_Position_0));

		if (!this->m_pGeometry->Traits().HasTrait(Tag_Velocity))
			this->m_pGeometry->Traits().AddTrait(Tag_Velocity, Vector3d::Zero().eval());


		// Rotation
		Vector3d v = Vector3d::Zero();
		if (!this->m_pGeometry->Traits().HasTrait(Tag_Quat_X))
			this->m_pGeometry->Traits().AddTrait(Tag_Quat_X, Vector3d::Zero().eval());

		if (!this->m_pGeometry->Traits().HasTrait(Tag_Quat_V))
			this->m_pGeometry->Traits().AddTrait(Tag_Quat_V, Vector3d::Zero().eval());

		if (!this->m_pGeometry->Traits().HasTrait(Tag_Quat_0))
			this->m_pGeometry->Traits().AddTrait(Tag_Quat_0, Vector3d::Zero().eval());

		// Cached rotation

		if (!this->m_pGeometry->Traits().HasTrait(Tag_Rotat_X))
			this->m_pGeometry->Traits().AddTrait(Tag_Rotat_X, Matrix3d::Identity().eval());
	}

	VectorXd KEleRigidBody3D::GetPositionX() const
	{
		VectorXd vx(6);
		vx.segment(0, 3) = this->m_pGeometry->Traits().Vector3d(Tag_Position_X);
		vx.segment(3, 3) = this->m_pGeometry->Traits().Vector3d(Tag_Quat_X);
		return vx;
	}

	VectorXd KEleRigidBody3D::GetPosition0() const
	{
		VectorXd vx(6);
		vx.segment(0, 3) = this->m_pGeometry->Traits().Vector3d(Tag_Position_0);
		vx.segment(3, 3) = this->m_pGeometry->Traits().Vector3d(Tag_Quat_0);
		return vx;
	}

	VectorXd KEleRigidBody3D::GetVelocity() const
	{
		VectorXd vv(6);
		vv.segment(0, 3) = this->m_pGeometry->Traits().Vector3d(Tag_Velocity);
		vv.segment(3, 3) = this->m_pGeometry->Traits().Vector3d(Tag_Quat_V);
		return vv;
	}

	void KEleRigidBody3D::SetPositionX(const VectorXd& vx)
	{
		this->m_pGeometry->Traits().Vector3d(Tag_Position_X) = vx.segment(0, 3);
		this->m_pGeometry->Traits().Vector3d(Tag_Quat_X) = vx.segment(3, 3);
	}

	void KEleRigidBody3D::SetPosition0(const VectorXd& vx)
	{
		this->m_pGeometry->Traits().Vector3d(Tag_Position_X) = vx.segment(0, 3);
		this->m_pGeometry->Traits().Vector3d(Tag_Quat_0) = vx.segment(3, 3);
	}

	void KEleRigidBody3D::SetVelocity(const VectorXd& vv)
	{
		this->m_pGeometry->Traits().Vector3d(Tag_Velocity) = vv.segment(0, 3);
		this->m_pGeometry->Traits().Vector3d(Tag_Quat_V) = vv.segment(3, 3);
	}

	int KEleRigidBody3D::GetFullStateSize() const { return 21; }

	VectorXd KEleRigidBody3D::GetFullState() const
	{
		VectorXd vs(21);
		vs.segment(0, 6) = this->GetPositionX();
		vs.segment(6, 6) = this->GetVelocity();
		Matrix3d mR0 = this->GetCachedRotation();
		vs.segment(12, 3) = mR0.col(0);
		vs.segment(15, 3) = mR0.col(1);
		vs.segment(18, 3) = mR0.col(2);
		return vs;
	}

	void KEleRigidBody3D::SetFullState(const VectorXd& vs)
	{
		VectorXd vx = vs.segment(0, 6);
		VectorXd vv = vs.segment(6, 6);
		Matrix3d mR0;
		mR0.col(0) = vs.segment(12, 3);
		mR0.col(1) = vs.segment(15, 3);
		mR0.col(2) = vs.segment(18, 3);
		this->SetPositionX(vx);
		this->SetVelocity(vv);
		this->SetCachedRotation(mR0);
	}

	bool KEleRigidBody3D::UpdateKinematics()
	{
		Matrix3d mR = this->ComputeFullRotation();

		this->m_pGeometry->Traits().Vector3d(Tag_Quat_X) *= 0.0; // Reset rotation
		this->m_pGeometry->Traits().Matrix3d(Tag_Rotat_X) = mR; // Reset cached

		//MatrixXd mN;
		//this->m_pGeometry->GetNodesTrait(mN, Tag_Position_0);

		//const Vector3d& v0 = this->m_pGeometry->Traits().Vector3d(Tag_Position_0);
		//const Vector3d& vx = this->m_pGeometry->Traits().Vector3d(Tag_Position_X);

		//// Translate to local coordinates
		//mN.rowwise() -= v0.transpose();

		//// Rotate around centroid
		//mN = mN * mR.transpose();

		//// Translate to global coordinates
		//mN.rowwise() += vx.transpose();

		//this->m_pGeometry->SetNodesTrait(mN, Tag_Position_X);

		return true;
	}

	Matrix3d KEleRigidBody3D::ComputeFullRotation() const
	{
		const Vector3d& vaa = this->m_pGeometry->Traits().Vector3d(Tag_Quat_X);
		const MatrixXd& mR0 = this->m_pGeometry->Traits().Matrix3d(Tag_Rotat_X);

		return GeometryUtils::rotationAxisAngleToMatrix(vaa)*mR0;
	}

	Matrix3d KEleRigidBody3D::GetCachedRotation() const
	{
		return this->m_pGeometry->Traits().Matrix3d(Tag_Rotat_X);
	}

	void KEleRigidBody3D::SetCachedRotation(const Matrix3d& mR)
	{
		this->m_pGeometry->Traits().Matrix3d(Tag_Rotat_X) = mR;
	}

}