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

#include <PhySim/Kinematics/KinematicsMap.h>

#include <PhySim/Kinematics/KEleParticle3D.h>
#include <PhySim/Kinematics/KEleRigidBody3D.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class KMapRB2Particle3D : public KinematicsMap {
 protected:
  PtrS<KEleRigidBody3D> m_pIn;

 public:
  KMapRB2Particle3D() {}
  KMapRB2Particle3D(Simulable* pModel,
                    PtrS<KEleRigidBody3D>& pIn,
                    const vector<PtrS<KinematicsEle>>& vOut);
  virtual void Init(Simulable* pModel,
                    PtrS<KEleRigidBody3D>& pIn,
                    const vector<PtrS<KinematicsEle>>& vOut);
  virtual void MapValue(int idxOut,
                        const VectorXd& vpIn,
                        int idxIn,
                        VectorXd& vpOut) override;
  virtual void UpdateMapPartials() override;
};

}  // namespace PhySim
