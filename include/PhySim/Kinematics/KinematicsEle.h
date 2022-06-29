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

#include <PhySim/Physics/DoFSet.h>

#include <PhySim/Physics/Simulables/Simulable.h>

#include <PhySim/Kinematics/KinematicsMap.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Geometry;
class Simulable;
class KinematicsMap;

struct KMappingBlock {
  virtual bool IsConstant() const = 0;
  virtual const MatrixXd& GetBlock() = 0;
};

struct KMappingBlock_Constant : public KMappingBlock {
  MatrixXd mBlock;

  KMappingBlock_Constant(const MatrixXd& mBlock) : KMappingBlock() {
    this->mBlock = mBlock;
  }

  virtual bool IsConstant() const override { return true; }

  virtual const MatrixXd& GetBlock() override;
};

struct KMappingBlock_Variable : public KMappingBlock {
  int iOut;
  int iIn;
  KinematicsMap* map;
  KMappingBlock_Variable(int iOut, int iIn, KinematicsMap* map)
      : KMappingBlock() {
    this->iOut = iOut;
    this->iIn = iIn;
    this->map = map;
  }

  virtual bool IsConstant() const override { return false; }

  virtual const MatrixXd& GetBlock() override;
};

struct KMappingPort {
 public:
  PtrS<KinematicsEle> m_pEle = nullptr;
  PtrS<KinematicsMap> m_pMap = nullptr;
  int m_index = -1;
  int m_offset = -1;
  int m_numDim = -1;
};

class KinematicsEle : public DoFSet {
 public:
  KinematicsEle() : DoFSet() { this->Initialize(); }

  KinematicsEle(Simulable* pModel) : DoFSet(pModel) { this->Initialize(); }

  KinematicsEle(Simulable* pModel, Geometry* pGeom, int numDim)
      : DoFSet(pModel, pGeom, numDim) {
    this->Initialize();
  }

  virtual PtrS<KinematicsEle> Clone() = 0;

  virtual void Initialize() {
    // Nothing to do here...
  }

  virtual bool HasMappingIn() const { return !m_vpConnIn.empty(); }
  virtual bool HasMappingOut() const { return !m_vpConnOut.empty(); }
  virtual vector<PtrS<KMappingPort>>& PortsIn() { return this->m_vpConnIn; }
  virtual vector<PtrS<KMappingPort>>& PortsOut() { return this->m_vpConnOut; }

  inline virtual int GetFullStateSize() const = 0;

  virtual VectorXd GetFullState() const = 0;
  virtual VectorXd GetPositionX() const = 0;
  virtual VectorXd GetPosition0() const = 0;
  virtual VectorXd GetVelocity() const = 0;

  virtual void SetFullState(const VectorXd& vs) = 0;
  virtual void SetPositionX(const VectorXd& vp) = 0;
  virtual void SetPosition0(const VectorXd& vp) = 0;
  virtual void SetVelocity(const VectorXd& vp) = 0;

  virtual bool UpdateKinematics() { return false; }

  ///////////////////// DoFSet /////////////////////

  VectorXd GetValue() const override { return this->GetPositionX(); }

  void SetValue(const VectorXd& vv) override { this->SetPositionX(vv); }

  virtual void SetFixed(bool fix) override;

  virtual void SetFixed(const bVector& vfix) override;

  ///////////////////// DoFSet /////////////////////

  void GatherMapChain(int owner,
                      vector<vector<PtrS<KMappingBlock>>>& vpmBlocks,
                      vector<KinematicsEle*>& vpKinem);

  void MapDerivatives(int owner,
                      const MatrixXd& mBIn,
                      vector<KinematicsEle*>& vpKOut,
                      vector<MatrixXd>& vmBOut);

 protected:
  vector<PtrS<KMappingPort>> m_vpConnIn;
  vector<PtrS<KMappingPort>> m_vpConnOut;
};

}  // namespace PhySim
