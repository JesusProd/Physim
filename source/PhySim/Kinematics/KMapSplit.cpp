//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC MAdrid
//
//==========================================================

#include <PhySim/Kinematics/KMapSplit.h>


namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	KMapSplit::KMapSplit(Simulable* pModel, PtrS<KinematicsEle>& pEleIn, vector<PtrS<KinematicsEle>>& vpEleOut)
	{
		this->Init(pModel, pEleIn, vpEleOut);
	}

	void KMapSplit::Init(Simulable* pModel, PtrS<KinematicsEle>& pEleIn, vector<PtrS<KinematicsEle>>& vpEleOut)
	{
		vector<PtrS<KinematicsEle>> vIn;

		vIn.push_back(pEleIn);

		for (int i = 0; i < (int)vpEleOut.size(); ++i)		
			assert(pEleIn->NumDim() == vpEleOut[i]->NumDim());

		KinematicsMap::Init(pModel, vIn, vpEleOut);

		this->m_isLinear = false;
	}

	void KMapSplit::UpdateMapPartials()
	{
		for (size_t iOut = 0; iOut < m_vOut.size(); ++iOut)
			this->m_vDoutDin[iOut][0] = MatrixXd::Identity(this->m_vOut[iOut]->m_numDim,
														   this->m_vIn[0]->m_numDim);
		this->m_isDirty = false;
	}

	void KMapSplit::MapValue(int idxIn, const VectorXd& vpIn, int idxOut, VectorXd& vpOut)
	{
		vpOut = vpIn;
	}

	void KMapSplit::MapDerivatives(int owner, int idxOut, const MatrixXd& mBIn, vector<KinematicsEle*>& vpKOut, vector<MatrixXd>& vmBOut)
	{
		// Just pass the message, split mapping: same block
		for (int i = 0; i < (int) this->m_vIn.size(); ++i)
			m_vIn[i]->m_pEle->MapDerivatives(owner, mBIn, vpKOut, vmBOut);
	}
}

