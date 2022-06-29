//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC MAdrid
//
//==========================================================

#include <PhySim/Kinematics/KEleDER.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

void KEleDERMatTwist::Initialize() {
  if (!this->m_pGeometry->Traits().HasTrait(Tag::Tag_Angle_X))
    this->m_pGeometry->Traits().AddTrait(Tag::Tag_Angle_X, 0.0);

  if (!this->m_pGeometry->Traits().HasTrait(Tag::Tag_Angle_0))
    this->m_pGeometry->Traits().AddTrait(Tag::Tag_Angle_0, 0.0);

  if (!this->m_pGeometry->Traits().HasTrait(Tag::Tag_Angle_V))
    this->m_pGeometry->Traits().AddTrait(Tag::Tag_Angle_V, 0.0);

  if (!this->m_pGeometry->Traits().HasTrait(Tag::Tag_Frame_X))
    this->m_pGeometry->Traits().AddTrait(Tag::Tag_Frame_X, Frame3d());

  if (!this->m_pGeometry->Traits().HasTrait(Tag::Tag_Frame_0))
    this->m_pGeometry->Traits().AddTrait(Tag::Tag_Frame_0, Frame3d());
}

int KEleDERMatTwist::GetFullStateSize() const {
  return 11;
}

VectorXd KEleDERMatTwist::GetFullState() const {
  VectorXd vs(11);
  vs(0) = this->GetPositionX()[0];
  vs(1) = this->GetVelocity()[0];

  Frame3d F = this->m_pGeometry->Traits().Frame3d(Tag::Tag_Frame_X);

  vs.segment(2, 3) = F.tan;
  vs.segment(5, 3) = F.bin;
  vs.segment(8, 3) = F.nor;

  return vs;
}

VectorXd KEleDERMatTwist::GetPositionX() const {
  VectorXd vx(1);
  vx[0] = this->m_pGeometry->Traits().Double(Tag::Tag_Angle_X);
  return vx;
}

VectorXd KEleDERMatTwist::GetPosition0() const {
  VectorXd vx(1);
  vx[0] = this->m_pGeometry->Traits().Double(Tag::Tag_Angle_0);
  return vx;
}

VectorXd KEleDERMatTwist::GetVelocity() const {
  VectorXd vx(1);
  vx[0] = this->m_pGeometry->Traits().Double(Tag::Tag_Angle_V);
  return vx;
}

void KEleDERMatTwist::SetFullState(const VectorXd& vs) {
  this->SetPositionX(vs.segment(0, 1));
  this->SetVelocity(vs.segment(1, 1));

  Frame3d F;

  F.tan = vs.segment(2, 3);
  F.bin = vs.segment(5, 3);
  F.nor = vs.segment(8, 3);

  this->m_pGeometry->Traits().Frame3d(Tag::Tag_Frame_X) = F;
}

void KEleDERMatTwist::SetPositionX(const VectorXd& vp) {
  this->m_pGeometry->Traits().Double(Tag::Tag_Angle_X) = vp[0];
}

void KEleDERMatTwist::SetPosition0(const VectorXd& vp) {
  this->m_pGeometry->Traits().Double(Tag::Tag_Angle_0) = vp[0];
}

void KEleDERMatTwist::SetVelocity(const VectorXd& vp) {
  this->m_pGeometry->Traits().Double(Tag::Tag_Angle_V) = vp[0];
}

bool KEleDERMatTwist::UpdateKinematics() {
  this->m_pMeshDER->AdaptReferenceFrame(this->m_pGeometry->ID(), Tag::Tag_Frame_X);

  return true;
}

void KEleDERConnRot::Initialize() {
  if (!this->m_pGeometry->Traits().HasTrait(Tag::Tag_Euler_X))
    this->m_pGeometry->Traits().AddTrait(Tag::Tag_Euler_X, Vector3d::Zero());

  if (!this->m_pGeometry->Traits().HasTrait(Tag::Tag_Euler_V))
    this->m_pGeometry->Traits().AddTrait(Tag::Tag_Euler_V, Vector3d::Zero());

  if (!this->m_pGeometry->Traits().HasTrait(Tag::Tag_Rotat_X))
    this->m_pGeometry->Traits().AddTrait(Tag::Tag_Rotat_X, Matrix3d::Identity());
}

int KEleDERConnRot::GetFullStateSize() const {
  return 15;
}

VectorXd KEleDERConnRot::GetFullState() const {
  VectorXd vs(15);
  vs.segment(0, 3) = this->GetPositionX();
  vs.segment(3, 3) = this->GetVelocity();

  Matrix3d R = this->m_pGeometry->Traits().Matrix3d(Tag::Tag_Rotat_X);

  vs.segment(6, 3) = R.col(0);
  vs.segment(9, 3) = R.col(1);
  vs.segment(12, 3) = R.col(2);

  return vs;
}

VectorXd KEleDERConnRot::GetPositionX() const {
  return this->m_pGeometry->Traits().Vector3d(Tag::Tag_Euler_X);
}

VectorXd KEleDERConnRot::GetVelocity() const {
  return this->m_pGeometry->Traits().Vector3d(Tag::Tag_Euler_V);
}

VectorXd KEleDERConnRot::GetPosition0() const {
  return VectorXd::Zero(3);  // Always 0
}

void KEleDERConnRot::SetFullState(const VectorXd& vs) {
  this->SetPositionX(vs.segment(0, 3));
  this->SetVelocity(vs.segment(3, 3));

  Matrix3d R;

  R.col(0) = vs.segment(6, 3);
  R.col(1) = vs.segment(9, 3);
  R.col(2) = vs.segment(12, 3);

  this->m_pGeometry->Traits().Matrix3d(Tag::Tag_Rotat_X) = R;
}

void KEleDERConnRot::SetPositionX(const VectorXd& vp) {
  this->m_pGeometry->Traits().Vector3d(Tag::Tag_Euler_X) = vp;
}

void KEleDERConnRot::SetVelocity(const VectorXd& vp) {
  this->m_pGeometry->Traits().Vector3d(Tag::Tag_Euler_V) = vp;
}

void KEleDERConnRot::SetPosition0(const VectorXd& vp) {
  // Constant: Always 0
}

bool KEleDERConnRot::UpdateKinematics() {
  int connID = m_pMeshDER->GetNodeConnection(m_pGeometry->ID());
  this->m_pMeshDER->UpdateConnectionRotation(connID);

  return true;
}

void KEleDERRefTwist::Initialize() {
  if (!this->m_pGeometry->Traits().HasTrait(Tag::Tag_Twist_X))
    this->m_pGeometry->Traits().AddTrait(Tag::Tag_Twist_X, 0.0);
}

int KEleDERRefTwist::GetFullStateSize() const {
  return 1;
}

VectorXd KEleDERRefTwist::GetFullState() const {
  return this->GetPositionX();
}

VectorXd KEleDERRefTwist::GetPositionX() const {
  VectorXd vx(1);
  vx[0] = this->m_pGeometry->Traits().Double(Tag::Tag_Twist_X);
  return vx;
}

VectorXd KEleDERRefTwist::GetPosition0() const {
  return VectorXd::Zero(1);
}

VectorXd KEleDERRefTwist::GetVelocity() const {
  return VectorXd::Zero(1);
}

void KEleDERRefTwist::SetFullState(const VectorXd& vs) {
  this->SetPositionX(vs);
}

void KEleDERRefTwist::SetPositionX(const VectorXd& vp) {
  this->m_pGeometry->Traits().Double(Tag::Tag_Twist_X) = vp(0);
}

void KEleDERRefTwist::SetPosition0(const VectorXd& vp) {
  // Constant
}

void KEleDERRefTwist::SetVelocity(const VectorXd& vp) {
  // Constant
}

bool KEleDERRefTwist::UpdateKinematics() {
  this->m_pMeshDER->ComputeReferenceTwist(this->m_pGeometry->ID());

  return true;
}

}  // namespace PhySim