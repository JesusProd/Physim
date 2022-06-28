//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC MAdrid
//
//==========================================================

#include <PhySim/Kinematics/KMapRB2Particle3D.h>

#include <PhySim/Utils/MathUtils.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	KMapRB2Particle3D::KMapRB2Particle3D(Simulable* pModel, PtrS<KEleRigidBody3D>& pIn, const vector<PtrS<KinematicsEle>>& vOut)
	{
		this->Init(pModel, pIn, vOut);
	}

	void KMapRB2Particle3D::Init(Simulable* pModel, PtrS<KEleRigidBody3D>& pIn, const vector<PtrS<KinematicsEle>>& vOut)
	{
		vector<PtrS<KinematicsEle>> vIn;
		this->m_pIn = pIn;
		vIn.push_back(pIn);

		KinematicsMap::Init(pModel, vIn, vOut);

		this->m_isLinear = false;
	}

	void KMapRB2Particle3D::MapValue(int idxIn, const VectorXd& vpIn, int idxOut, VectorXd& vpOut)
	{
		Vector3d c0 = m_pIn->GetPosition0().segment(0, 3); // Rest centroid
		Vector3d cx = m_pIn->GetPositionX().segment(0, 3); // Defo. centroid
		Vector3d vt = cx - c0;

		Matrix3d mR = m_pIn->ComputeFullRotation();

		vpOut = mR * (this->m_vOut[idxOut]->m_pEle->GetPosition0() - c0) + c0 + vt;
	}

	void KMapRB2Particle3D::UpdateMapPartials()
	{
		Vector3d c0 = m_pIn->GetPosition0().segment(0, 3); // Rest centroid
		Matrix3d mR = m_pIn->ComputeFullRotation(); // GetCachedRotation();

		Matrix3d mI = Matrix3d::Identity();

		for (int idxOut = 0; idxOut < (int)this->m_vOut.size(); ++idxOut)
		{
			
			Vector3d posOfThisEle = this->m_vOut[idxOut]->m_pEle->GetPosition0();
			Vector3d dNodeC0 = (this->m_vOut[idxOut]->m_pEle->GetPosition0() - c0);
			Vector3d vr = mR * (this->m_vOut[idxOut]->m_pEle->GetPosition0() - c0);
			Matrix3d mR;
			MathUtils::crossMatrix(vr, mR);
			this->m_vDoutDin[idxOut][0].resize(3, 6);
			this->m_vDoutDin[idxOut][0].block(0, 0, 3, 3) = mI;
			this->m_vDoutDin[idxOut][0].block(0, 3, 3, 3) = -mR;
		}

		this->m_isDirty = false;
	}


}