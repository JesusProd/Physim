//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC MAdrid
//
//==========================================================

#include <PhySim/Kinematics/KMapIdentity.h>


namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	KMapIdentity::KMapIdentity(Simulable* pModel, PtrS<KinematicsEle>& pEleIn, PtrS<KinematicsEle>& pEleOut)
	{
		this->Init(pModel, pEleIn, pEleOut);
	}

	void KMapIdentity::Init(Simulable* pModel, PtrS<KinematicsEle>& pEleIn, PtrS<KinematicsEle>& pEleOut)
	{
		assert(pEleIn->NumDim() == pEleOut->NumDim());

		vector<PtrS<KinematicsEle>> vIn;
		vector<PtrS<KinematicsEle>> vOut;
		vIn.push_back(pEleIn);
		vOut.push_back(pEleOut);
		KinematicsMap::Init(pModel, vIn, vOut);

		this->m_isLinear = true;
	}

	void KMapIdentity::PropagateFixation()
	{
		if (this->m_vOut[0]->m_pEle->AnyFixed())
			this->m_vIn[0]->m_pEle->SetFixed(this->m_vOut[0]->m_pEle->GetFixed());
	}

	void KMapIdentity::UpdateMapPartials()
	{
		this->m_vDoutDin[0][0] = MatrixXd::Identity(this->m_vOut[0]->m_numDim, 
													this->m_vIn[0]->m_numDim);
		this->m_isDirty = false;
	}

	void KMapIdentity::MapValue(int idxIn, const VectorXd& vpIn, int idxOut, VectorXd& vpOut)
	{
		vpOut = vpIn;
	}

	void KMapIdentity::MapDerivatives(int owner, int idxOut, const MatrixXd& mBIn, vector<KinematicsEle*>& vpKOut, vector<MatrixXd>& vmBOut)
	{
		// Just pass the message, identity mapping: same derivative
		m_vIn[0]->m_pEle->MapDerivatives(owner, mBIn, vpKOut, vmBOut);
	}

}

