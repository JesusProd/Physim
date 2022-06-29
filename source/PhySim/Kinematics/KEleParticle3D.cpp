//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC MAdrid
//
//==========================================================

#include <PhySim/Kinematics/KEleParticle3D.h>

#include <PhySim/Geometry/Geometry.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

PtrS<KinematicsEle> KEleParticle3D::Clone() {
  return PtrS<KinematicsEle>(
      new KEleParticle3D(this->m_pModel, this->m_pGeometry));
}

void KEleParticle3D::Initialize() {
  if (!this->m_pGeometry->Traits().HasTrait(Tag::Tag_Position_X))
    this->m_pGeometry->Traits().AddTrait(Tag::Tag_Position_X, Vector3d::Zero());

  if (!this->m_pGeometry->Traits().HasTrait(Tag::Tag_Velocity))
    this->m_pGeometry->Traits().AddTrait(Tag::Tag_Velocity, Vector3d::Zero());

  if (!this->m_pGeometry->Traits().HasTrait(Tag::Tag_Position_0))
    this->m_pGeometry->Traits().AddTrait(
        Tag::Tag_Position_0, this->m_pGeometry->Traits().Vector3d(Tag::Tag_Position_X));
}

int KEleParticle3D::GetFullStateSize() const {
  return 6;
};

VectorXd KEleParticle3D::GetFullState() const {
  VectorXd vs(6);
  vs.segment(0, 3) = this->GetPositionX();
  vs.segment(3, 3) = this->GetVelocity();
  return vs;
}

VectorXd KEleParticle3D::GetPositionX() const {
  return VectorXd(this->m_pGeometry->Traits().Vector3d(Tag::Tag_Position_X));
}

VectorXd KEleParticle3D::GetPosition0() const {
  return VectorXd(this->m_pGeometry->Traits().Vector3d(Tag::Tag_Position_0));
}

VectorXd KEleParticle3D::GetVelocity() const {
  return this->m_pGeometry->Traits().Vector3d(Tag::Tag_Velocity);
}

void KEleParticle3D::SetFullState(const VectorXd& vs) {
  this->SetPositionX(vs.segment(0, 3));
  this->SetVelocity(vs.segment(3, 3));
}

void KEleParticle3D::SetPositionX(const VectorXd& vp) {
  this->m_pGeometry->Traits().Vector3d(Tag::Tag_Position_X) = vp;
}

void KEleParticle3D::SetPosition0(const VectorXd& vp) {
  this->m_pGeometry->Traits().Vector3d(Tag::Tag_Position_0) = vp;
}

void KEleParticle3D::SetVelocity(const VectorXd& vp) {
  this->m_pGeometry->Traits().Vector3d(Tag::Tag_Velocity) = vp;
}

}  // namespace PhySim