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

#include <PhySim/Kinematics/KMapLinear.h>

#include <PhySim/Physics/Simulables/Simulable.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class KMapIdentity : public KinematicsMap {
 public:
  KMapIdentity() {}
  KMapIdentity(Simulable* pModel,
               PtrS<KinematicsEle>& pEleIn,
               PtrS<KinematicsEle>& vpEleOut);
  virtual void Init(Simulable* pModel,
                    PtrS<KinematicsEle>& pEleIn,
                    PtrS<KinematicsEle>& vpEleOut);

  void PropagateFixation() override;

  virtual void UpdateMapPartials() override;
  virtual void MapValue(int idxIn,
                        const VectorXd& vpIn,
                        int idxOut,
                        VectorXd& vpOut) override;
  virtual void MapDerivatives(int owner,
                              int idxOut,
                              const MatrixXd& mBIn,
                              vector<KinematicsEle*>& vpKOut,
                              vector<MatrixXd>& vmBOut) override;
};

}  // namespace PhySim
