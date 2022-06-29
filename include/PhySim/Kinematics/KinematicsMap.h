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

#include <PhySim/Physics/Simulables/Simulable.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class KinematicsEle;
struct KMappingPort;
struct KMappingBlock;

class Simulable;

class KinematicsMap {
 protected:
  vector<PtrS<KMappingPort>> m_vIn;
  vector<PtrS<KMappingPort>> m_vOut;
  vector<vector<MatrixXd>> m_vDoutDin;
  // vector<vector<PtrS<KMappingBlock>>> m_vDoutDinNew;
  bool m_isLinear = false;
  bool m_isDirty = true;
  bool m_isInit = false;
  Simulable* m_pModel;

 public:
  KinematicsMap();
  KinematicsMap(Simulable* pModel,
                const vector<PtrS<KinematicsEle>>& vIn,
                const vector<PtrS<KinematicsEle>>& vOut);
  virtual void Init(Simulable* pModel,
                    const vector<PtrS<KinematicsEle>>& vIn,
                    const vector<PtrS<KinematicsEle>>& vOut);

  inline virtual Simulable* GetModel() const { return this->m_pModel; }
  inline virtual void SetModel(Simulable* pM) { this->m_pModel = pM; }

  virtual int GetNumIn() const { return (int)this->m_vIn.size(); }
  virtual int GetNumOut() const { return (int)this->m_vOut.size(); }

  virtual vector<PtrS<KMappingPort>> PortsIn() { return m_vIn; }
  virtual vector<PtrS<KMappingPort>> PortsOut() { return m_vOut; }

  virtual void MapPositionsX();
  virtual void MapPositions0();
  virtual void MapVelocity();

  virtual void UpdateKinematics();

  virtual void PropagateFixation();

  virtual const MatrixXd& GetJacobianBlock(int iOut, int iIn);

  virtual void GatherMapChain(int owner,
                              int idxOut,
                              vector<vector<PtrS<KMappingBlock>>>& vpmBlocks,
                              vector<KinematicsEle*>& vpKinem);

  virtual void MapDerivatives(int owner,
                              int idxOut,
                              const MatrixXd& mDeri,
                              vector<KinematicsEle*>& vpEles,
                              vector<MatrixXd>& vmBlocks);

  virtual void MapValue(int idxIn,
                        const VectorXd& vpIn,
                        int idxOut,
                        VectorXd& vpOut) = 0;

  virtual void UpdateMapPartials() = 0;
};

}  // namespace PhySim
