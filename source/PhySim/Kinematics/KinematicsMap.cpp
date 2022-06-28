//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC MAdrid
//
//==========================================================

#include <PhySim/Kinematics/KinematicsMap.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	//////////////////////////////////////////////////// KinematicsMap ////////////////////////////////////////////////////

	KinematicsMap::KinematicsMap()
	{
		this->m_isInit = false;
	}

	KinematicsMap::KinematicsMap(Simulable* pModel, const vector<PtrS<KinematicsEle>>& vIn, const vector<PtrS<KinematicsEle>>& vOut)
	{
		this->Init(pModel, vIn, vOut);
	}

	void KinematicsMap::Init(Simulable* pModel, const vector<PtrS<KinematicsEle>>& vIn, const vector<PtrS<KinematicsEle>>& vOut)
	{
		int numIn = (int)vIn.size();
		int numOut = (int)vOut.size();

		this->m_vIn.clear();
		this->m_vOut.clear();

		// Initialize Out connections

		int countOut = 0;
		for (int idx = 0; idx < (int)vOut.size(); ++idx)
		{
			PtrS<KMappingPort> pPort = make_shared<KMappingPort>();
			
			// Connect port to map
			m_vOut.push_back(pPort);
			pPort->m_pMap.reset(this);

			// Connect port to element
			vOut[idx]->PortsIn().push_back(pPort);
			pPort->m_pEle = vOut[idx];

			// Configure port
			pPort->m_index = idx;
			pPort->m_offset = countOut;
			pPort->m_numDim = vOut[idx]->NumDim();
			countOut += vOut[idx]->NumDim();

			// A mapped element is not active
			vOut[idx]->Active() = false;
		}

		// Initialize In connections

		int countIn = 0;
		for (int idx = 0; idx < (int)vIn.size(); ++idx)
		{
			PtrS<KMappingPort> pPort = make_shared<KMappingPort>();

			// Connect port to map
			m_vIn.push_back(pPort);
			pPort->m_pMap.reset(this);

			// Connect port to element
			vIn[idx]->PortsOut().push_back(pPort);
			pPort->m_pEle = vIn[idx];

			// Configure port
			pPort->m_index = idx;
			pPort->m_offset = countIn;
			pPort->m_numDim = vIn[idx]->NumDim();
			countIn += vIn[idx]->NumDim();
		}

		this->m_isLinear = true;

		// Initialize mapping matrix blocks

		this->m_vDoutDin.resize(vOut.size());
		for (size_t iOut = 0; iOut < vOut.size(); ++iOut)
		{
			this->m_vDoutDin[iOut].resize(vIn.size());
			for (size_t iIn = 0; iIn < vIn.size(); ++iIn)
			{
				this->m_vDoutDin[iOut][iIn] = MatrixXd::Zero(vOut[iOut]->NumDim(),
															 vIn[iIn]->NumDim());
			}
		}

		this->UpdateMapPartials();

		this->m_isInit = true;

		this->m_pModel = pModel;
	}

	const MatrixXd& KinematicsMap::GetJacobianBlock(int iOut, int iIn)
	{
		if (this->m_isDirty)
			this->UpdateMapPartials();

		assert(iOut >= 0 && iOut < this->m_vOut.size());
		assert(iIn >= 0 && iIn < this->m_vIn.size());

		return this->m_vDoutDin[iOut][iIn];
	}

	//void KinematicsMap::MapKinematics()
	//{
	//	this->MapPositionsX();
	//	this->MapVelocity();

	//	for (int idxOut = 0; idxOut < (int)this->m_vOut.size(); ++idxOut)
	//	{
	//		this->m_vOut[idxOut]->UpdateKinematics();
	//	}
	//}

	void KinematicsMap::MapPositionsX()
	{
		if (!this->m_isLinear)
			this->m_isDirty = true;

		for (int idxIn = 0; idxIn < (int)this->m_vIn.size(); ++idxIn)
			for (int idxOut = 0; idxOut < (int)this->m_vOut.size(); ++idxOut)
			{

				VectorXd vxOut;
				VectorXd vxIn = m_vIn[idxIn]->m_pEle->GetPositionX();
				this->MapValue(idxIn, vxIn, idxOut, vxOut);
				this->m_vOut[idxOut]->m_pEle->SetPositionX(vxOut);
			}
	}

	void KinematicsMap::MapPositions0()
	{
		if (!this->m_isLinear)
			this->m_isDirty = true;

		for (int idxIn = 0; idxIn < (int)this->m_vIn.size(); ++idxIn)
			for (int idxOut = 0; idxOut < (int)this->m_vOut.size(); ++idxOut)
			{

				VectorXd vxOut;
				VectorXd vxIn = m_vIn[idxIn]->m_pEle->GetPosition0();
				this->MapValue(idxIn, vxIn, idxOut, vxOut);
				this->m_vOut[idxOut]->m_pEle->SetPosition0(vxOut);
			}
	}

	void KinematicsMap::MapVelocity()
	{
		if (!this->m_isLinear)
			this->m_isDirty = true;

		for (int idxIn = 0; idxIn < (int)this->m_vIn.size(); ++idxIn)
			for (int idxOut = 0; idxOut < (int)this->m_vOut.size(); ++idxOut)
			{

				VectorXd vxOut;
				VectorXd vxIn = m_vIn[idxIn]->m_pEle->GetVelocity();
				this->MapValue(idxIn, vxIn, idxOut, vxOut);
				this->m_vOut[idxOut]->m_pEle->SetVelocity(vxOut);
			}
	}

	void KinematicsMap::UpdateKinematics()
	{
		if (!this->m_isLinear)
			this->m_isDirty = true;

		for (int idxOut = 0; idxOut < (int)this->m_vOut.size(); ++idxOut)
		{
			this->m_vOut[idxOut]->m_pEle->UpdateKinematics();
		}
	}

	void KinematicsMap::PropagateFixation()
	{
		bool anyFixed = false;

		for (auto pPortOut : m_vOut)
			anyFixed |= pPortOut->m_pEle->AnyFixed();

		if (anyFixed)
			for (auto pPortIn : m_vIn)
				pPortIn->m_pEle->SetFixed(true);
	}

	void KinematicsMap::GatherMapChain(int owner, int idxOut, vector<vector<PtrS<KMappingBlock>>>& vpmBlocks, vector<KinematicsEle*>& vpKinem)
	{
		vector<PtrS<KMappingBlock>> vprefix;
		if (!vpmBlocks.empty())
			vprefix = vpmBlocks.back();

		for (int i = 0; i < (int)m_vIn.size(); ++i)
		{
			if (i != 0)
			{
				// Split the mapping: when several Kinematic elements map into
				// an element, map chain must be splitted. The concatenation of
				// all elements through the map is inserted as a prefix

				vpmBlocks.push_back(vprefix);
			}

			// Concatenate this mapping to the list of applied mappings
			
			PtrS<KMappingBlock> pBlock;

			if (this->m_isLinear)
			{
				pBlock.reset(new KMappingBlock_Constant(m_vDoutDin[idxOut][i]));
			}
			else
			{
				pBlock.reset(new KMappingBlock_Variable(idxOut, i, this));
			}

			vpmBlocks.back().push_back(pBlock);

			// Pass message to master elements to gather further mappings

			m_vIn[i]->m_pEle->GatherMapChain(owner, vpmBlocks, vpKinem);
		}
	}

	void KinematicsMap::MapDerivatives(int owner, int idxOut, const MatrixXd& mBIn, vector<KinematicsEle*>& vpKOut, vector<MatrixXd>& vmBOut)
	{
		if (this->m_isDirty)
			this->UpdateMapPartials();

		// For each input to this kinematics map we map the input block and forward the signal

		for (int i = 0; i < (int)m_vIn.size(); ++i)
		{
			assert(mBIn.cols() == this->m_vDoutDin[idxOut][i].rows());

			m_vIn[i]->m_pEle->MapDerivatives(owner, mBIn*(this->m_vDoutDin[idxOut][i]), vpKOut, vmBOut);
		}
	}
}

