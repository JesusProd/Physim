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

class KEleRigidBody3D : public KinematicsEle {
 public:
  KEleRigidBody3D(Simulable* pModel, Geometry* pGeom)
      : KinematicsEle(pModel, pGeom, 6) {
    // Nothing to do here...
  }

  ///////////////////// KinematicsEle /////////////////////

  virtual PtrS<KinematicsEle> Clone() override;

  virtual void Initialize() override;

  virtual int GetFullStateSize() const override;

  virtual VectorXd GetFullState() const override;
  virtual VectorXd GetPositionX() const override;
  virtual VectorXd GetPosition0() const override;
  virtual VectorXd GetVelocity() const override;

  virtual void SetFullState(const VectorXd& vs) override;
  virtual void SetPositionX(const VectorXd& vx) override;
  virtual void SetPosition0(const VectorXd& vx) override;
  virtual void SetVelocity(const VectorXd& vv) override;

  virtual bool UpdateKinematics() override;

  ///////////////////// KinematicsEle /////////////////////

  virtual Matrix3d ComputeFullRotation() const;

  virtual Matrix3d GetCachedRotation() const;
  virtual void SetCachedRotation(const Matrix3d& mR);
};

}  // namespace PhySim