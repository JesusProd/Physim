//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Utils/DerivativeAssembler.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

void DerivativeAssembler::InitializeMappings(const vector<IDoFSet*>& vDoF,
                                             int layer) {
  this->m_vpMapping.clear();

  // Gather mappings chains: for each DoF, we obtain a list of active DoF
  // the input DoF depends on, together with a list of mappings chains to
  // apply to each derivative block

  for (size_t iDoF = 0; iDoF < vDoF.size(); ++iDoF) {
    KinematicsEle* pEle_i = static_cast<KinematicsEle*>(vDoF[iDoF]);

    // Create and initialize mapping
    this->m_vpMapping.push_back(DoFMapping());
    this->m_vpMapping.back().mappedElement = pEle_i;   // Mapped element
    this->m_vpMapping.back().mappingElements.clear();  // Mapping elements
    this->m_vpMapping.back().mappingMatrices.clear();  // Mapping matrices
    this->m_vpMapping.back().mappingChains.push_back(
        vector<PtrS<KMappingBlock>>());  // Mapping chain

    // Gather active elements and chain

    pEle_i->GatherMapChain(layer, this->m_vpMapping.back().mappingChains,
                           this->m_vpMapping.back().mappingElements);

    assert(this->m_vpMapping.back().mappingElements.size() == 1);
  }

  // Optimize mappings chains: some of these mappings chains might be optimized.
  // 1) Sequences of constant mappings can be collapsed into one.
  // 2) Identity mappings can be removed from the mappings chains.
  // 3) Any other?

  // TODO

  this->m_initializedMapping = layer;
}

void DerivativeAssembler::PreprocessMappings(const vector<IDoFSet*>& vDoF,
                                             int layer) {
  // Precompute gathering if possible
  if (m_initializedMapping != layer)
    InitializeMappings(vDoF, layer);

  for (size_t iDoF = 0; iDoF < this->m_vpMapping.size(); ++iDoF) {
    KinematicsEle* pMappedEle = static_cast<KinematicsEle*>(vDoF[iDoF]);

    auto& vmappingChains = this->m_vpMapping[iDoF].mappingChains;
    auto& vmappingElements = this->m_vpMapping[iDoF].mappingElements;

    this->m_vpMapping[iDoF].mappingMatrices.resize(vmappingElements.size());

    // Evaluate mapping matrices

    for (size_t iEle = 0; iEle < vmappingElements.size(); ++iEle) {
      auto pMappingEle = vmappingElements[iEle];

      // The resulting map should have dimensions sizeof(mappedDoF) x
      // sizeof(activeDoF) but initially it has dimensions sizeof(mappedDoF) x
      // sizeof(mappedDoF). These might change due to the concatenation of
      // transformations

      MatrixXd mMapping =
          MatrixXd::Identity(vDoF[iDoF]->NumDim(), vDoF[iDoF]->NumDim());

      for (size_t iMap = 0; iMap < vmappingChains[iEle].size(); ++iMap)
        mMapping *= vmappingChains[iEle][iMap]->GetBlock();  // Apply map!

      this->m_vpMapping[iDoF].mappingMatrices[iEle] = mMapping;
    }
  }
}

void DerivativeAssembler::AssemblePreprocessedGradient(
    const VectorXd& vgradient,
    AVectorXd& vglobalGradient) {
  int localOffset = 0;

  for (int iDoF = 0; iDoF < (int)m_vpMapping.size(); ++iDoF) {
    const VectorXd& vgradient_i = vgradient.segment(
        localOffset, this->m_vpMapping[iDoF].mappedElement->NumDim());

    auto& vmappingElements = this->m_vpMapping[iDoF].mappingElements;
    auto& vmappingMatrices = this->m_vpMapping[iDoF].mappingMatrices;

    for (size_t iEle = 0; iEle < vmappingElements.size(); ++iEle) {
      vglobalGradient.segment(vmappingElements[iEle]->Offset(),
                              vmappingElements[iEle]->NumDim()) +=
          vmappingMatrices[iEle].transpose() * vgradient_i;
    }

    localOffset += this->m_vpMapping[iDoF].mappedElement->NumDim();
  }
}

void DerivativeAssembler::AssemblePreprocessedHessian(
    const MatrixXd& mHessian,
    AMatrixSd& mglobalHessian) {
  int blockCount = 0;

  int localOffset_i = 0;
  int localOffset_j = 0;

  for (size_t iDoF = 0; iDoF < this->m_vpMapping.size(); ++iDoF) {
    KinematicsEle* pEle_i = this->m_vpMapping[iDoF].mappedElement;

    for (size_t jDoF = 0; jDoF <= iDoF; ++jDoF) {
      KinematicsEle* pEle_j = this->m_vpMapping[jDoF].mappedElement;

      // Obtain local block of the Hessian
      const MatrixXd& mHij = mHessian.block(localOffset_i, localOffset_j,
                                            pEle_i->NumDim(), pEle_j->NumDim());

      // Obtain mapping elements and matrices
      auto& mappingMatrices_i = this->m_vpMapping[iDoF].mappingMatrices;
      auto& mappingMatrices_j = this->m_vpMapping[jDoF].mappingMatrices;
      auto& mappingElements_i = this->m_vpMapping[iDoF].mappingElements;
      auto& mappingElements_j = this->m_vpMapping[jDoF].mappingElements;

      for (size_t iMap = 0; iMap < mappingElements_i.size(); ++iMap)
        for (size_t jMap = 0; jMap < mappingElements_j.size(); ++jMap) {
          MatrixXd mMapped = mappingMatrices_i[iMap].transpose() * mHij *
                             mappingMatrices_j[jMap];

          if (this->m_isDynamic)
            mglobalHessian.AddDynamicBlock(mappingElements_i[iMap]->Offset(),
                                           mappingElements_j[jMap]->Offset(),
                                           mMapped);
          else {
            if (!mglobalHessian.IsFast())
              mglobalHessian.AddStaticBlock(mappingElements_i[iMap]->Offset(),
                                            mappingElements_j[jMap]->Offset(),
                                            mMapped);
            else {
              if (this->m_vhessBlocksRef.size() <= blockCount)
                this->m_vhessBlocksRef.push_back(mglobalHessian.CreateBlock(
                    mappingElements_i[iMap]->Offset(),
                    mappingElements_j[jMap]->Offset(), mMapped.rows(),
                    mMapped.cols()));

              mglobalHessian.AddStaticBlock(this->m_vhessBlocksRef[blockCount],
                                            mMapped);
            }
          }

          blockCount++;
        }

      localOffset_j += pEle_j->NumDim();
    }

    localOffset_j = 0;
    localOffset_i += pEle_i->NumDim();
  }
}

void DerivativeAssembler::PropagateGradient(const VectorXd& vgradient,
                                            const vector<IDoFSet*>& vDoF,
                                            int layer) {
  int localDoFOffset = 0;

  size_t numDoFSet = vDoF.size();

  // Propagation

  this->m_vKinesBack.clear();
  this->m_vBlocksGrad.clear();

  for (size_t i = 0; i < numDoFSet; ++i) {
    VectorXd vgi = vgradient.segment(localDoFOffset, vDoF[i]->NumDim());

    KinematicsEle* pEle_i = static_cast<KinematicsEle*>(vDoF[i]);

    pEle_i->MapDerivatives(layer, vgi.transpose(), m_vKinesBack, m_vBlocksGrad);

    localDoFOffset += vDoF[i]->NumDim();
  }

  assert(m_vBlocksGrad.size() == m_vKinesBack.size());
}

void DerivativeAssembler::AssembleGradient(AVectorXd& vglobalGradient) {
  // Summation

  for (int b = 0; b < (int)m_vBlocksGrad.size(); ++b)
    vglobalGradient.segment(m_vKinesBack[b]->Offset(),
                            m_vKinesBack[b]->NumDim()) +=
        m_vBlocksGrad[b].transpose();
}

void DerivativeAssembler::PropagateHessian(const MatrixXd& mHessian,
                                           const vector<IDoFSet*>& vDoF,
                                           int layer) {
  int localOffset_i = 0;
  int localOffset_j = 0;

  size_t numDoFSet = vDoF.size();

  // Propagation

  this->m_vBlocksHess.clear();
  this->m_vKinesFront.clear();
  this->m_vKinesBack.clear();

  for (size_t i = 0; i < numDoFSet; ++i) {
    for (size_t j = 0; j <= i; ++j) {
      MatrixXd mHkl = mHessian.block(localOffset_i, localOffset_j,
                                     vDoF[i]->NumDim(), vDoF[j]->NumDim());
      KinematicsEle* pEle_i = static_cast<KinematicsEle*>(vDoF[i]);
      KinematicsEle* pEle_j = static_cast<KinematicsEle*>(vDoF[j]);

      this->m_vKinesTemp.clear();
      this->m_vBlocksTemp.clear();

      pEle_j->MapDerivatives(layer, mHkl, m_vKinesTemp, m_vBlocksTemp);

      for (int blockIdxBack = 0; blockIdxBack < (int)m_vBlocksTemp.size();
           ++blockIdxBack) {
        int countPre = m_vBlocksHess.size();
        pEle_i->MapDerivatives(layer, m_vBlocksTemp[blockIdxBack].transpose(),
                               m_vKinesFront, m_vBlocksHess);
        int countPos = m_vBlocksHess.size();

        for (int blockIdx = 0; blockIdx < (countPos - countPre); ++blockIdx)
          m_vKinesBack.push_back(m_vKinesTemp[blockIdxBack]);  // Multimapped
      }

      int numBlocks = (int)m_vBlocksHess.size();
      assert(m_vKinesFront.size() == numBlocks);
      assert(m_vKinesBack.size() == numBlocks);

      localOffset_j += vDoF[j]->NumDim();

      if (!(i == j) && m_vBlocksHess.back().rows() == 6 &&
          m_vBlocksHess.back().cols() == 6 &&
          (m_vKinesFront.back()->Offset() == m_vKinesBack.back()->Offset())) {
        MatrixXd mToAdd(6, 6), mToTranspose(6, 6);
        mToAdd = m_vBlocksHess.back();
        mToTranspose = mToAdd.transpose();

        m_vBlocksHess[numBlocks - 1] = mToAdd + mToTranspose;
      }
    }

    localOffset_j = 0;
    localOffset_i += vDoF[i]->NumDim();
  }

  int numBlocks = (int)m_vBlocksHess.size();
  assert(m_vKinesFront.size() == numBlocks);
  assert(m_vKinesBack.size() == numBlocks);
}

void DerivativeAssembler::AssembleHessian(AMatrixSd& mglobalHessian) {
  int blockCount = 0;

  // Summation

  for (int b = 0; b < (int)m_vBlocksHess.size(); ++b) {
    m_vBlocksHess[b].transposeInPlace();

    assert(m_vKinesFront[b]->NumDim() == m_vBlocksHess[b].rows());
    assert(m_vKinesBack[b]->NumDim() == m_vBlocksHess[b].cols());

    if (this->m_isDynamic)
      mglobalHessian.AddDynamicBlock(m_vKinesFront[b]->Offset(),
                                     m_vKinesBack[b]->Offset(),
                                     m_vBlocksHess[b]);
    else {
      if (!mglobalHessian.IsFast()) {
        mglobalHessian.AddStaticBlock(m_vKinesFront[b]->Offset(),
                                      m_vKinesBack[b]->Offset(),
                                      m_vBlocksHess[b]);  //, true
      } else {
        if (this->m_vhessBlocksRef.size() <= blockCount)
          this->m_vhessBlocksRef.push_back(mglobalHessian.CreateBlock(
              m_vKinesFront[b]->Offset(), m_vKinesBack[b]->Offset(),
              m_vBlocksHess[b].rows(), m_vBlocksHess[b].cols()));

        mglobalHessian.AddStaticBlock(this->m_vhessBlocksRef[blockCount],
                                      m_vBlocksHess[b]);
      }
    }

    blockCount++;
  }
}

void DerivativeAssembler::PropagateAndAssembleGradient(
    const VectorXd& vgradient,
    const vector<IDoFSet*>& vDoF,
    AVectorXd& vglobalGradient) {
  int localDoFOffset = 0;

  size_t numDoFSet = vDoF.size();

  // Propagation

  this->m_vMappingKines.clear();
  this->m_vMappingBlocks.clear();

  for (size_t i = 0; i < numDoFSet; ++i) {
    VectorXd vgi = vgradient.segment(localDoFOffset, vDoF[i]->NumDim());

    KinematicsEle* pEle_i = static_cast<KinematicsEle*>(vDoF[i]);

    pEle_i->MapDerivatives(vglobalGradient.Layer(), vgi.transpose(),
                           m_vMappingKines, m_vMappingBlocks);

    localDoFOffset += vDoF[i]->NumDim();
  }

  assert(m_vMappingBlocks.size() == m_vMappingKines.size());

  // Summation

  for (int b = 0; b < (int)m_vMappingBlocks.size(); ++b)
    vglobalGradient.segment(m_vMappingKines[b]->Offset(),
                            m_vMappingKines[b]->NumDim()) +=
        m_vMappingBlocks[b].transpose();
}

void DerivativeAssembler::PropagateAndAssembleHessian(
    const MatrixXd& mHessian,
    const vector<IDoFSet*>& vDoF,
    AMatrixSd& mglobalHessian) {
  int blockCount = 0;

  int localOffset_i = 0;
  int localOffset_j = 0;

  size_t numDoFSet = vDoF.size();

  // Propagation

  this->m_vBlocksHess.clear();
  this->m_vKinesFront.clear();
  this->m_vMappingKines.clear();

  for (size_t i = 0; i < numDoFSet; ++i) {
    for (size_t j = 0; j <= i; ++j) {
      MatrixXd mHkl = mHessian.block(localOffset_i, localOffset_j,
                                     vDoF[i]->NumDim(), vDoF[j]->NumDim());
      KinematicsEle* pEle_i = static_cast<KinematicsEle*>(vDoF[i]);
      KinematicsEle* pEle_j = static_cast<KinematicsEle*>(vDoF[j]);

      this->m_vKinesTemp.clear();
      this->m_vBlocksTemp.clear();

      pEle_j->MapDerivatives(mglobalHessian.Layer(), mHkl, m_vKinesTemp,
                             m_vBlocksTemp);

      for (int blockIdxBack = 0; blockIdxBack < (int)m_vBlocksTemp.size();
           ++blockIdxBack) {
        int countPre = m_vBlocksHess.size();
        pEle_i->MapDerivatives(mglobalHessian.Layer(),
                               m_vBlocksTemp[blockIdxBack].transpose(),
                               m_vKinesFront, m_vBlocksHess);
        int countPos = m_vBlocksHess.size();

        for (int blockIdx = 0; blockIdx < (countPos - countPre); ++blockIdx)
          m_vMappingKines.push_back(m_vKinesTemp[blockIdxBack]);  // Multimapped
      }

      int numBlocks = (int)m_vBlocksHess.size();
      assert(m_vKinesFront.size() == numBlocks);
      assert(m_vMappingKines.size() == numBlocks);

      localOffset_j += vDoF[j]->NumDim();
    }

    localOffset_j = 0;
    localOffset_i += vDoF[i]->NumDim();
  }

  int numBlocks = (int)m_vBlocksHess.size();
  assert(m_vKinesFront.size() == numBlocks);
  assert(m_vMappingKines.size() == numBlocks);

  // Summation

  for (int b = 0; b < (int)m_vBlocksHess.size(); ++b) {
    m_vBlocksHess[b].transposeInPlace();

    assert(m_vKinesFront[b]->NumDim() == m_vBlocksHess[b].rows());
    assert(m_vMappingKines[b]->NumDim() == m_vBlocksHess[b].cols());

    if (this->m_isDynamic)
      mglobalHessian.AddDynamicBlock(m_vKinesFront[b]->Offset(),
                                     m_vMappingKines[b]->Offset(),
                                     m_vBlocksHess[b]);
    else {
      if (!mglobalHessian.IsFast())
        mglobalHessian.AddStaticBlock(m_vKinesFront[b]->Offset(),
                                      m_vMappingKines[b]->Offset(),
                                      m_vBlocksHess[b]);
      else {
        if (this->m_vhessBlocksRef.size() <= blockCount)
          this->m_vhessBlocksRef.push_back(mglobalHessian.CreateBlock(
              m_vKinesFront[b]->Offset(), m_vMappingKines[b]->Offset(),
              m_vBlocksHess[b].rows(), m_vBlocksHess[b].cols()));

        mglobalHessian.AddStaticBlock(this->m_vhessBlocksRef[blockCount],
                                      m_vBlocksHess[b]);
      }
    }

    blockCount++;
  }
}

// void DerivativeAssembler::AssembleGradient(const VectorXd& vgradient, const
// vector<IDoFSet*>& vDoF, AVectorXd& vglobalGradient)
//{
//	int localDoFOffset = 0;

//	size_t numDoFSet = vDoF.size();

//	for (size_t i = 0; i < numDoFSet; ++i)
//	{
//		VectorXd vgi = vgradient.segment(localDoFOffset,
// vDoF[i]->NumDim());

//		vector<KinematicsEle*> vpEle;

//		this->m_vgradBlocks.clear();

//		KinematicsEle* pEle_i = static_cast<KinematicsEle*>(vDoF[i]);

//		if (pEle_i->HasMappingIn() &&
// pEle_i->ConnectionIn()->m_pMap->GetModel()->ID() == vglobalGradient.Layer())
//		{
//			pEle_i->ConnectionIn()->MapDerivatives(vglobalGradient.Layer(),
// vgi.transpose(), vpEle, m_vgradBlocks, true);
//		}
//		else
//		{
//			vpEle.push_back(pEle_i);
//			m_vgradBlocks.push_back(vgi.transpose());
//		}

//		int numBlocks = (int)m_vgradBlocks.size();
//		assert(vpEle.size() == numBlocks);

//		for (int b = 0; b < numBlocks; ++b)
//			vglobalGradient.segment(vpEle[b]->Offset(),
// vpEle[b]->NumDim()) += m_vgradBlocks[b].transpose();

//		localDoFOffset += vDoF[i]->NumDim();
//	}
//}

// void DerivativeAssembler::AssembleHessian(const MatrixXd& mHessian, const
// vector<IDoFSet*>& vDoF, AMatrixSd& mglobalHessian)
//{
//	int blockCount = 0;

//	int localOffset_i = 0;
//	int localOffset_j = 0;

//	size_t numDoFSet = vDoF.size();
//	for (size_t i = 0; i < numDoFSet; ++i)
//	{
//		for (size_t j = 0; j <= i; ++j)
//		{
//			MatrixXd mHkl = mHessian.block(localOffset_i,
// localOffset_j, vDoF[i]->NumDim(), vDoF[j]->NumDim());
// KinematicsEle* pEle_i = static_cast<KinematicsEle*>(vDoF[i]);
// KinematicsEle* pEle_j = static_cast<KinematicsEle*>(vDoF[j]);

//			//vector<MatrixXd> vallBlocks;
//			//vallBlocks.push_back(mHkl);

//			this->m_vBlocksFront.clear();
//			this->m_vBlocksBack.clear();

//			vector<KinematicsEle*> vpBackEle;
//			vector<KinematicsEle*> vpFrontEle;

//			// Map back

//			if (pEle_j->HasMappingIn() &&
// pEle_j->ConnectionIn()->m_pMap->GetModel()->ID() == mglobalHessian.Layer())
//			{
//				pEle_j->ConnectionIn()->MapDerivatives(mglobalHessian.Layer(),
// mHkl, vpBackEle, m_vBlocksBack, true);
//			}
//			else
//			{
//				m_vBlocksBack.push_back(mHkl);
//				vpBackEle.push_back(pEle_j); // Multiple times
//			}

//			// Map front

//			if (pEle_i->HasMappingIn() &&
// pEle_i->ConnectionIn()->m_pMap->GetModel()->ID() == mglobalHessian.Layer())
//			{
//				for (int blockIdx = 0; blockIdx <
//(int)m_vBlocksBack.size(); ++blockIdx)
//					pEle_i->ConnectionIn()->MapDerivatives(mglobalHessian.Layer(),
// m_vBlocksBack[blockIdx], vpFrontEle, m_vBlocksFront, false);
//			}
//			else
//			{

//				for (int k = 0; k < (int)m_vBlocksBack.size();
//++k)
//				{
//					m_vBlocksFront.push_back(m_vBlocksBack[k]);
//					vpFrontEle.push_back(pEle_i); //
// Multiple times
//				}
//			}

//			int numBlocks = (int)m_vBlocksFront.size();
//			assert(vpFrontEle.size() == numBlocks);
//			assert(vpBackEle.size() == numBlocks);

//			// Assemble all blocks

//			for (int b = 0; b < (int)m_vBlocksFront.size(); ++b)
//			{
//				assert(vpFrontEle[b]->NumDim() ==
// m_vBlocksFront[b].rows());
// assert(vpBackEle[b]->NumDim()
// == m_vBlocksFront[b].cols());

//				if (this->m_isDynamic)
//					mglobalHessian.AddDynamicBlock(vpFrontEle[b]->Offset(),
// vpBackEle[b]->Offset(), m_vBlocksFront[b]); 				else
//				{
//					if (!mglobalHessian.IsFast())
//						mglobalHessian.AddStaticBlock(vpFrontEle[b]->Offset(),
// vpBackEle[b]->Offset(), m_vBlocksFront[b]); else
//					{
//						if
//(this->m_vhessBlocksRef.size()
//<=
// blockCount)
// this->m_vhessBlocksRef.push_back(
// mglobalHessian.CreateBlock(
// vpFrontEle[b]->Offset(),
// vpBackEle[b]->Offset(),
//									m_vBlocksFront[b].rows(),
//									m_vBlocksFront[b].cols()));
//

//						mglobalHessian.AddStaticBlock(this->m_vhessBlocksRef[blockCount],
// m_vBlocksFront[b]);
//					}
//				}

//				blockCount++;
//			}

//			localOffset_j += vDoF[j]->NumDim();
//		}

//		localOffset_j = 0;
//		localOffset_i += vDoF[i]->NumDim();
//	}
//}

void DerivativeAssembler::AssembleJacobian(const MatrixXd& mJacobian,
                                           const vector<IDoFSet*>& vDoF,
                                           AMatrixSd& mglobalJacobian) {
  // TODO
}

}  // namespace PhySim
