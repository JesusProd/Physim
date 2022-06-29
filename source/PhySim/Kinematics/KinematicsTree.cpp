//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Kinematics/KinematicsTree.h>

#include <PhySim/Utils/Utils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

KinematicsTree::KinematicsTree(Simulable* pModel,
                               vector<PtrS<KinematicsEle>>& vproots) {
  this->m_pModel = pModel;
  this->m_numActiveDOF = -1;
  this->m_vproots = vproots;
}

void KinematicsTree::Init() {
  this->m_numActiveDOF = 0;

  // Set active DoF

  int countDoF = 0;

  size_t numRoots = m_vproots.size();
  for (size_t i = 0; i < numRoots; ++i) {
    if (!this->m_vproots[i]->Active())
      continue;  // Ignore inactive

    this->m_vproots[i]->Index() = countDoF++;
    this->m_vproots[i]->Offset() = this->m_numActiveDOF;
    this->m_numActiveDOF += this->m_vproots[i]->NumDim();
  }

  // Traverse tree: this piece of code alrady leaves mappings
  // sorted in a consistent order so that position/velocity
  // updates respect kinematic tree dependencies

  this->m_vallEle.clear();
  this->m_vallMap.clear();

  vector<PtrS<KinematicsEle>> vnextEles = this->m_vproots;
  vector<PtrS<KinematicsMap>> vnextMaps;

  while (!vnextEles.empty() || !vnextMaps.empty()) {
    // Elements layer

    while (!vnextEles.empty()) {
      for (auto pPort : vnextEles[0]->PortsOut())
        vnextMaps.push_back(pPort->m_pMap);

      m_vallEle.push_back(vnextEles[0]);
      vnextEles.erase(vnextEles.begin());
    }

    // Mapping layer

    while (!vnextMaps.empty()) {
      for (auto pPort : vnextMaps[0]->PortsOut())
        vnextEles.push_back(pPort->m_pEle);

      m_vallMap.push_back(vnextMaps[0]);
      vnextMaps.erase(vnextMaps.begin());
    }
  }

  // Initialize all elements
  for (auto pEle : m_vallEle)
    pEle->Initialize();
}

void KinematicsTree::GetFullState(VectorXd& vs) const {
  vector<Real> vstate;

  int count = 0;

  for (int i = 0; i < (int)this->m_vallEle.size(); ++i) {
    PtrS<KinematicsEle> pKine = m_vallEle[i];
    VectorXd vsi = pKine->GetFullState();
    for (auto j = 0; j < vsi.size(); ++j)
      vstate.push_back(vsi(j));
  }

  vs = Utils::toEigen(vstate);
}

void KinematicsTree::GetPositionX(VectorXd& vx) const {
  vx.resize(this->m_numActiveDOF);

  size_t numRoots = this->m_vproots.size();
  for (size_t i = 0; i < numRoots; ++i) {
    if (!this->m_vproots[i]->Active())
      continue;  // Ignore, not DOF

    vx.segment(this->m_vproots[i]->Offset(), this->m_vproots[i]->NumDim()) =
        this->m_vproots[i]->GetPositionX();
  }
}

void KinematicsTree::GetPosition0(VectorXd& v0) const {
  v0.resize(this->m_numActiveDOF);

  size_t numRoots = this->m_vproots.size();
  for (size_t i = 0; i < numRoots; ++i) {
    if (!this->m_vproots[i]->Active())
      continue;  // Ignore, not DOF

    v0.segment(this->m_vproots[i]->Offset(), this->m_vproots[i]->NumDim()) =
        this->m_vproots[i]->GetPosition0();
  }
}

void KinematicsTree::GetVelocity(VectorXd& vv) const {
  vv.resize(this->m_numActiveDOF);

  size_t numRoots = this->m_vproots.size();
  for (size_t i = 0; i < numRoots; ++i) {
    if (!this->m_vproots[i]->Active())
      continue;  // Ignore, not DOF

    vv.segment(this->m_vproots[i]->Offset(), this->m_vproots[i]->NumDim()) =
        this->m_vproots[i]->GetVelocity();
  }
}

// void KinematicsTree::UpdateMapPartials()
//{
//	for (PtrS<KinematicsMap> pMap : this->m_vallMap)
//		pMap->UpdateMapPartials();
//}

void KinematicsTree::SetFullState(const VectorXd& vs) {
  int count = 0;

  for (int i = 0; i < (int)this->m_vallEle.size(); ++i) {
    PtrS<KinematicsEle> pKine = m_vallEle[i];
    int N = pKine->GetFullStateSize();
    VectorXd vsi = vs.segment(count, N);
    pKine->SetFullState(vsi);
    count += N;
  }

  for (PtrS<KinematicsMap> pMap : this->m_vallMap)
    pMap->UpdateMapPartials();
}

void KinematicsTree::SetPositionX(const VectorXd& vx) {
  size_t numRoots = this->m_vproots.size();
  for (size_t i = 0; i < numRoots; ++i) {
    if (!this->m_vproots[i]->Active())
      continue;  // Ignore, not DOF

    this->m_vproots[i]->SetPositionX(
        vx.segment(this->m_vproots[i]->Offset(), this->m_vproots[i]->NumDim()));
  }

  this->MapPositionX();
}

void KinematicsTree::SetPosition0(const VectorXd& v0) {
  size_t numRoots = this->m_vproots.size();
  for (size_t i = 0; i < numRoots; ++i) {
    if (!this->m_vproots[i]->Active())
      continue;  // Ignore, not DOF

    this->m_vproots[i]->SetPosition0(
        v0.segment(this->m_vproots[i]->Offset(), this->m_vproots[i]->NumDim()));
  }

  for (PtrS<KinematicsMap> pMap : this->m_vallMap)
    pMap->MapPositions0();  // Already consistent order
}

void KinematicsTree::SetVelocity(const VectorXd& vv) {
  size_t numRoots = this->m_vproots.size();
  for (size_t i = 0; i < numRoots; ++i) {
    if (!this->m_vproots[i]->Active())
      continue;  // Ignore, not DOF

    this->m_vproots[i]->SetVelocity(
        vv.segment(this->m_vproots[i]->Offset(), this->m_vproots[i]->NumDim()));
  }

  for (PtrS<KinematicsMap> pMap : this->m_vallMap)
    pMap->MapVelocity();  // Already consistent order
}

void KinematicsTree::MapPositionX() {
  for (PtrS<KinematicsMap> pMap : this->m_vallMap)
    pMap->MapPositionsX();  // Already consistent order
}

void KinematicsTree::MapPosition0() {
  for (PtrS<KinematicsMap> pMap : this->m_vallMap)
    pMap->MapPositions0();  // Already consistent order
}

void KinematicsTree::MapVelocity() {
  for (PtrS<KinematicsMap> pMap : this->m_vallMap)
    pMap->MapVelocity();  // Already consistent order
}

void KinematicsTree::UpdateKinematics() {
  size_t numRoots = m_vproots.size();
  for (size_t i = 0; i < numRoots; ++i) {
    this->m_vproots[i]->UpdateKinematics();
  }

  for (PtrS<KinematicsMap> pMap : this->m_vallMap)
    pMap->UpdateKinematics();  // Already consistent order
}

void KinematicsTree::PropagateFixation() {
  for (int i = (int)m_vallMap.size() - 1; i >= 0; --i)
    m_vallMap[i]->PropagateFixation();  // Inverse
}

}  // namespace PhySim