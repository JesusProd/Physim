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

#include <PhySim/Geometry/Meshes/Mesh_DER.h>
#include <PhySim/Kinematics/KinematicsEle.h>
#include <PhySim/Physics/DoFSet.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class KEleDERConnRot : public KinematicsEle {
 protected:
  Mesh_DER* m_pMeshDER;

 public:
  inline KEleDERConnRot(Simulable* pModel, Poly* pGeom)
      : KinematicsEle(pModel, pGeom, 3) {
    this->m_pMeshDER = static_cast<Mesh_DER*>(pGeom->GetMesh());
  }

  ///////////////////// KinematicsEle /////////////////////

  virtual PtrS<KinematicsEle> Clone() override {
    throw PhySim::exception("Not implemented");
  }

  void Initialize() override;

  virtual int GetFullStateSize() const override;

  virtual VectorXd GetFullState() const override;
  virtual VectorXd GetPositionX() const override;
  virtual VectorXd GetVelocity() const override;
  virtual VectorXd GetPosition0() const override;

  virtual void SetFullState(const VectorXd& vs) override;
  virtual void SetPositionX(const VectorXd& vp) override;
  virtual void SetVelocity(const VectorXd& vp) override;
  virtual void SetPosition0(const VectorXd& vp) override;

  virtual bool UpdateKinematics();

  ///////////////////// KinematicsEle /////////////////////
};

class KEleDERMatTwist : public KinematicsEle {
 protected:
  Mesh_DER* m_pMeshDER;

 public:
  inline KEleDERMatTwist(Simulable* pModel, Poly* pGeom)
      : KinematicsEle(pModel, pGeom, 1) {
    this->m_pMeshDER = static_cast<Mesh_DER*>(pGeom->GetMesh());
  }

  ///////////////////// KinematicsEle /////////////////////

  virtual PtrS<KinematicsEle> Clone() override {
    throw PhySim::exception("Not implemented");
  }

  virtual void Initialize() override;

  virtual int GetFullStateSize() const override;

  virtual VectorXd GetFullState() const override;
  virtual VectorXd GetPositionX() const override;
  virtual VectorXd GetPosition0() const override;
  virtual VectorXd GetVelocity() const override;

  virtual void SetFullState(const VectorXd& vs) override;
  virtual void SetPositionX(const VectorXd& vp) override;
  virtual void SetPosition0(const VectorXd& vp) override;
  virtual void SetVelocity(const VectorXd& vp) override;

  virtual bool UpdateKinematics();

  ///////////////////// KinematicsEle /////////////////////
};

class KEleDERRefTwist : public KinematicsEle {
 protected:
  Mesh_DER* m_pMeshDER;

 public:
  KEleDERRefTwist(Simulable* pModel, Poly* pGeom)
      : KinematicsEle(pModel, pGeom, 1) {
    this->m_pMeshDER = static_cast<Mesh_DER*>(pGeom->GetMesh());

    this->Active() = false;
  }

  ///////////////////// KinematicsEle /////////////////////

  virtual PtrS<KinematicsEle> Clone() override {
    throw PhySim::exception("Not implemented");
  }

  virtual void Initialize() override;

  virtual int GetFullStateSize() const override;

  virtual VectorXd GetFullState() const override;
  virtual VectorXd GetPositionX() const override;
  virtual VectorXd GetPosition0() const override;
  virtual VectorXd GetVelocity() const override;

  virtual void SetFullState(const VectorXd& vs) override;
  virtual void SetPositionX(const VectorXd& vp) override;
  virtual void SetPosition0(const VectorXd& vp) override;
  virtual void SetVelocity(const VectorXd& vp) override;

  virtual bool UpdateKinematics();

  ///////////////////// KinematicsEle /////////////////////
};

}  // namespace PhySim