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

namespace PhySim {
using namespace std;
using namespace Eigen;

class KEleParticle3D : public KinematicsEle {
 public:
  KEleParticle3D(Simulable* pModel, Geometry* pGeom)
      : KinematicsEle(pModel, pGeom, 3) {
    // Nothing to do here...
  }

  ///////////////////// KinematicsEle /////////////////////

  virtual PtrS<KinematicsEle> Clone() override;

  virtual void Initialize() override;

  virtual int GetFullStateSize() const;

  virtual VectorXd GetFullState() const override;
  virtual VectorXd GetPositionX() const override;
  virtual VectorXd GetPosition0() const override;
  virtual VectorXd GetVelocity() const override;

  virtual void SetFullState(const VectorXd& vs);
  virtual void SetPositionX(const VectorXd& vp);
  virtual void SetPosition0(const VectorXd& vp);
  virtual void SetVelocity(const VectorXd& vp);

  ///////////////////// KinematicsEle /////////////////////
};

}  // namespace PhySim
