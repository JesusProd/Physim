//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>

#include <PhySim/Kinematics/KinematicsEle.h>
#include <PhySim/Kinematics/KinematicsMap.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class KinematicsEle;
class KinematicsMap;
class Simulable;

class KinematicsTree {
 protected:
  Simulable* m_pModel;

  int m_numActiveDOF;
  vector<PtrS<KinematicsEle>> m_vproots;
  vector<PtrS<KinematicsEle>> m_vallEle;
  vector<PtrS<KinematicsMap>> m_vallMap;

 public:
  KinematicsTree(Simulable* pModel, vector<PtrS<KinematicsEle>>& vproots);

  virtual void Init();

  virtual const vector<PtrS<KinematicsEle>>& RootElements() {
    return this->m_vproots;
  }
  virtual const vector<PtrS<KinematicsEle>>& AllElements() {
    return this->m_vallEle;
  }
  virtual const vector<PtrS<KinematicsMap>>& AllMappings() {
    return this->m_vallMap;
  }

  virtual int GetNumActiveDoF() const { return this->m_numActiveDOF; }

  virtual void GetFullState(VectorXd& vs) const;
  virtual void SetFullState(const VectorXd& vs);

  virtual void GetPositionX(VectorXd& vx) const;
  virtual void GetPosition0(VectorXd& v0) const;
  virtual void GetVelocity(VectorXd& vv) const;

  virtual void SetPositionX(const VectorXd& vx);
  virtual void SetPosition0(const VectorXd& v0);
  virtual void SetVelocity(const VectorXd& vv);

  // virtual void UpdateMapPartials();

  virtual void MapPositionX();
  virtual void MapPosition0();
  virtual void MapVelocity();
  virtual void UpdateKinematics();
  virtual void PropagateFixation();
};

}  // namespace PhySim