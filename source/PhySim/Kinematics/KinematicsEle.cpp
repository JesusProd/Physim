//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Kinematics/KinematicsEle.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

const MatrixXd& KMappingBlock_Variable::GetBlock() {
  return map->GetJacobianBlock(iOut, iIn);
}

const MatrixXd& KMappingBlock_Constant::GetBlock() {
  return this->mBlock;
}

void KinematicsEle::SetFixed(bool fix) {
  DoFSet::SetFixed(fix);

  this->m_pModel->DirtyFixed();
}

void KinematicsEle::SetFixed(const bVector& vfix) {
  DoFSet::SetFixed(vfix);

  this->m_pModel->DirtyFixed();
}

void KinematicsEle::GatherMapChain(
    int owner,
    vector<vector<PtrS<KMappingBlock>>>& vpmBlocks,
    vector<KinematicsEle*>& vpKinem) {
  if (owner == this->Layer() && this->Active()) {
    vpKinem.push_back(this);
  } else {
    if (this->HasMappingIn()) {
      assert(PortsIn().size() == 1 &&
             "Number of inputs of an element should be mostly 1");
      PortsIn()[0]->m_pMap->GatherMapChain(owner, PortsIn()[0]->m_index,
                                           vpmBlocks, vpKinem);
    }
  }
}

void KinematicsEle::MapDerivatives(int owner,
                                   const MatrixXd& mBIn,
                                   vector<KinematicsEle*>& vpKOut,
                                   vector<MatrixXd>& vmBOut) {
  if (owner == this->Layer() && this->Active()) {
    vmBOut.push_back(mBIn);
    vpKOut.push_back(this);
  } else {
    if (this->HasMappingIn()) {
      assert(PortsIn().size() == 1 && "Number of inputs should be mostly 1");
      this->PortsIn()[0]->m_pMap->MapDerivatives(owner, PortsIn()[0]->m_index,
                                                 mBIn, vpKOut, vmBOut);
    }
  }
}

}  // namespace PhySim