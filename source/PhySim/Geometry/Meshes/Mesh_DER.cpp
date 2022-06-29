//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, IST Austria
//
//==========================================================

#include <PhySim/Geometry/Meshes/Mesh_DER.h>
#include <PhySim/Geometry/Meshes/Rod.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Utils/Auto/computeReferenceTwistChange.h>
#include <PhySim/Utils/Auto/rodriguesRotation.h>
#include <PhySim/Utils/GeometryUtils.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Mesh_DER::Mesh_DER() : Mesh_Frame() {
  // Nothing to do here...
}

Mesh_DER::Mesh_DER(const Mesh_DER& toCopy) : Mesh_Frame(toCopy) {
  this->UpdateMetadata();
}

Mesh_DER::Mesh_DER(const vector<PtrS<Rod>>& vrods, Tag trait, Real tol) {
  this->Init(vrods, trait, tol);
}

Mesh_DER::Mesh_DER(const MatrixXd& mV,
                   const MatrixXi& mE,
                   const vector<Frame3d>& vF,
                   const vector<Vector2d>& vr) {
  this->Init(mV, mE, vF, vr);

  this->UpdateMetadata();
}

void Mesh_DER::Init(const vector<PtrS<Rod>>& vrods, Tag trait, Real tol) {
  vector<PtrS<Mesh_Frame>> vbaseMeshes(vrods.size());
  for (int i = 0; i < (int)vrods.size(); ++i)
    vbaseMeshes[i] = dynamic_pointer_cast<Mesh_Frame>(vrods[i]);

  PtrS<Mesh_Frame> pTempMesh(new Mesh_Frame());

  Mesh_Frame::Merge(pTempMesh, vbaseMeshes, trait, tol);

  MatrixXd mV;
  MatrixXi mE;
  vector<Frame3d> vF;
  pTempMesh->GetNodesTrait(mV, pTempMesh->NodeTraits()[0]);
  pTempMesh->GetElemsTrait(vF, pTempMesh->ElemTraits()[0]);
  pTempMesh->GetElemMatrix(mE);

  this->Init(mV, mE, vF);

  // Copy node traits if they exist

  if (pTempMesh->HasNodeTraits(Tag::Tag_Position_0)) {
    MatrixXd mV;
    pTempMesh->GetNodesTrait(mV, Tag::Tag_Position_0);
    this->SetNodesTrait(mV, Tag::Tag_Position_0);
  }

  if (pTempMesh->HasNodeTraits(Tag::Tag_Position_X)) {
    MatrixXd mV;
    pTempMesh->GetNodesTrait(mV, Tag::Tag_Position_X);
    this->SetNodesTrait(mV, Tag::Tag_Position_X);
  }

  if (pTempMesh->HasNodeTraits(Tag::Tag_Velocity)) {
    MatrixXd mV;
    pTempMesh->GetNodesTrait(mV, Tag::Tag_Velocity);
    this->SetNodesTrait(mV, Tag::Tag_Velocity);
  }

  if (pTempMesh->HasElemTraits(Tag::Tag_Frame_X)) {
    vector<Frame3d> vF;
    pTempMesh->GetElemsTrait(vF, Tag::Tag_Frame_X);
    this->SetElemsTrait(vF, Tag::Tag_Frame_X);
  }

  if (pTempMesh->HasElemTraits(Tag::Tag_Frame_0)) {
    vector<Frame3d> vF;
    pTempMesh->GetElemsTrait(vF, Tag::Tag_Frame_0);
    this->SetElemsTrait(vF, Tag::Tag_Frame_0);
  }
}

void Mesh_DER::Init(const MatrixXd& mV,
                    const MatrixXi& mE,
                    const vector<Frame3d>& vF,
                    const vector<Vector2d>& vr) {
  vector<Tag> vnodeTraits;
  vnodeTraits.push_back(Tag::Tag_Position_0);
  vnodeTraits.push_back(Tag::Tag_Position_X);
  vnodeTraits.push_back(Tag::Tag_Velocity);

  vector<Tag> vframeTraits;
  vframeTraits.push_back(Tag::Tag_Frame_0);
  vframeTraits.push_back(Tag::Tag_Frame_X);

  Mesh_Frame::Init(mV, mE, vF, vnodeTraits, vframeTraits);

  // Add material angles at edges

  for (int i = 0; i < (int)m_velems.size(); ++i) {
    m_velems[i]->Traits().AddTrait<Real>(Tag::Tag_Angle_0, 0.0);
    m_velems[i]->Traits().AddTrait<Real>(Tag::Tag_Angle_X, 0.0);
    m_velems[i]->Traits().AddTrait<Real>(Tag::Tag_Angle_V, 0.0);
  }

  // Add reference twist at nodes

  for (int i = 0; i < (int)m_vnodes.size(); ++i) {
    m_vnodes[i]->Traits().AddTrait<Real>(Tag::Tag_Twist_X, 0.0);
  }

  // Add edge radius at edges

  if (vr.size() == 1) {
    for (int i = 0; i < (int)m_velems.size(); ++i) {
      m_velems[i]->Traits().AddTrait<Real>(Tag::Tag_Size_0, vr[0].x());
      m_velems[i]->Traits().AddTrait<Real>(Tag::Tag_Size_1, vr[0].y());
    }
  } else if (vr.size() == m_velems.size()) {
    for (int i = 0; i < (int)m_velems.size(); ++i) {
      m_velems[i]->Traits().AddTrait<Real>(Tag::Tag_Size_0, vr[i].x());
      m_velems[i]->Traits().AddTrait<Real>(Tag::Tag_Size_1, vr[i].y());
    }
  } else {
    for (int i = 0; i < (int)m_velems.size(); ++i) {
      m_velems[i]->Traits().AddTrait<Real>(Tag::Tag_Size_0, 0.01);
      m_velems[i]->Traits().AddTrait<Real>(Tag::Tag_Size_1, 0.01);
    }
  }

  // Add connections data

  for (int i = 0; i < (int)m_vconns.size(); ++i) {
    m_vconns[i]->m_center->Traits().AddTrait<Matrix3d>(Tag::Tag_Rotat_X,
                                                       Matrix3d::Identity());
    m_vconns[i]->m_center->Traits().AddTrait<Vector3d>(Tag::Tag_Euler_X,
                                                       Vector3d(0.0, 0.0, 0.0));
    m_vconns[i]->m_center->Traits().AddTrait<Vector3d>(Tag::Tag_Euler_V,
                                                       Vector3d(0.0, 0.0, 0.0));

    for (int j = 0; j < (int)m_vconns[i]->m_vedges.size(); ++j) {
      m_vconns[i]->m_vedges[j]->Traits().AddTrait<double>(Tag::Tag_Twist_X,
                                                          0.0);
    }
  }

  this->SetRefFrames_TwistFreeRods(Tag::Tag_Frame_0);
  this->SetRefFrames_TwistFreeRods(Tag::Tag_Frame_X);

  this->UpdateMetadata();
}

Mesh_DER::~Mesh_DER(void) {
  this->FreeInternal();

#ifndef NDEBUG
  IOUtils::logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Mesh_DER");
#endif
}

Frame3d Mesh_DER::ComputeMatFrame(int i, Tag s) {
  Frame3d refF;
  Real twist;

  if (s == Tag::Tag_Position_0 || s == Tag::Tag_Frame_0 || s == Tag::Tag_Angle_0) {
    refF = this->RefFrame(i, Tag::Tag_Frame_0);
    twist = this->MatAngle(i, Tag::Tag_Angle_0);
  }

  if (s == Tag::Tag_Position_X || s == Tag::Tag_Frame_X || s == Tag::Tag_Angle_X) {
    refF = this->RefFrame(i, Tag::Tag_Frame_X);
    twist = this->MatAngle(i, Tag::Tag_Angle_X);
  }

  Frame3d matF = refF;
  rodriguesRotation(refF.tan.data(), twist, refF.nor.data(), matF.nor.data());
  rodriguesRotation(refF.tan.data(), twist, refF.bin.data(), matF.bin.data());
  return matF;
}

void Mesh_DER::ComputeMatFrameVector(vector<Frame3d>& vF, Tag s) {
  vF.resize(this->NumElems());
  for (int i = 0; i < this->NumElems(); ++i)
    vF[i] = this->ComputeMatFrame(i, s);
}

void Mesh_DER::ComputeMatFrameArrows(MatrixXd& m0,
                                     MatrixXd& mN,
                                     MatrixXd& mB,
                                     Tag s) {
  vector<Frame3d> vF;
  this->ComputeMatFrameVector(vF, s);
  m0.resize(vF.size(), 3);
  mN.resize(vF.size(), 3);
  mB.resize(vF.size(), 3);
  for (int i = 0; i < this->NumEdges(); ++i) {
    Vector3d vedgeCen;

    if (s == Tag::Tag_Position_0 || s == Tag::Tag_Frame_0 || s == Tag::Tag_Angle_0)
      vedgeCen = this->m_vedges[i]->Centroid(Tag::Tag_Position_0);

    if (s == Tag::Tag_Position_X || s == Tag::Tag_Frame_X || s == Tag::Tag_Angle_X)
      vedgeCen = this->m_vedges[i]->Centroid(Tag::Tag_Position_X);

    m0.row(i) = vedgeCen;
    mN.row(i) = vF[i].nor;
    mB.row(i) = vF[i].bin;
  }
}

Real& Mesh_DER::MatAngle(int i, Tag s) {
  if (s == Tag::Tag_Position_X || s == Tag::Tag_Angle_X)
    return this->GetEdge(i)->Traits().Double(Tag::Tag_Angle_X);

  if (s == Tag::Tag_Position_0 || s == Tag::Tag_Angle_0)
    return this->GetEdge(i)->Traits().Double(Tag::Tag_Angle_0);

  throw PhySim::exception(
      "Invalid Tag, expected: PosX, Pos0, Angle_X, Angle_0");
}

Frame3d& Mesh_DER::RefFrame(int i, Tag s) {
  if (s == Tag::Tag_Position_X || s == Tag::Tag_Frame_X)
    return this->GetEdge(i)->Traits().Frame3d(Tag::Tag_Frame_X);

  if (s == Tag::Tag_Position_0 || s == Tag::Tag_Frame_0)
    return this->GetEdge(i)->Traits().Frame3d(Tag::Tag_Frame_0);

  throw PhySim::exception(
      "Invalid Tag, expected: PosX, Pos0, Frame_X, Frame_0");
}

Real& Mesh_DER::RefTwist(int i) {
  return this->Nodes()[i]->Traits().Double(Tag::Tag_Twist_X);
}

Vector3d& Mesh_DER::RotEuler(int i) {
  return this->Connections()[i]->m_center->Traits().Vector3d(Tag::Tag_Euler_X);
}

Matrix3d& Mesh_DER::RotMatrix(int i) {
  return this->Connections()[i]->m_center->Traits().Matrix3d(Tag::Tag_Rotat_X);
}

void Mesh_DER::GetMatAngleVector(vector<Real>& va, Tag s) const {
  va.resize(this->NumElems());
  for (int i = 0; i < this->NumElems(); ++i)
    va[i] = this->Elems()[i]->Traits().Double(s);
}

void Mesh_DER::GetRefFrameVector(vector<Frame3d>& vF, Tag s) const {
  vF.resize(this->NumElems());
  for (int i = 0; i < this->NumElems(); ++i)
    vF[i] = this->Elems()[i]->Traits().Frame3d(s);
}

void Mesh_DER::GetRefTwistVector(vector<Real>& vt) const {
  vt.resize(this->NumNodes());
  for (int i = 0; i < this->NumNodes(); ++i)
    vt[i] = this->Nodes()[i]->Traits().Double(Tag::Tag_Twist_X);
}

void Mesh_DER::GetRotEulerVector(vector<Vector3d>& vR) const {
  vR.resize(this->NumConns());
  for (int i = 0; i < this->NumConns(); ++i)
    vR[i] = this->Connections()[i]->m_center->Traits().Vector3d(Tag::Tag_Euler_X);
}

void Mesh_DER::GetRotMatrixVector(vector<Matrix3d>& vR) const {
  vR.resize(this->NumConns());
  for (int i = 0; i < this->NumConns(); ++i)
    vR[i] = this->Connections()[i]->m_center->Traits().Matrix3d(Tag::Tag_Rotat_X);
}

void Mesh_DER::SetMatAngleVector(const vector<Real>& va, Tag s) {
  assert((int)va.size() == this->NumElems());
  for (int i = 0; i < this->NumElems(); ++i)
    this->Elems()[i]->Traits().Double(s) = va[i];
}

void Mesh_DER::SetRefFrameVector(const vector<Frame3d>& vF, Tag s) {
  assert((int)vF.size() == this->NumElems());
  for (int i = 0; i < this->NumElems(); ++i)
    this->Elems()[i]->Traits().Frame3d(s) = vF[i];
}

void Mesh_DER::SetRefTwistVector(const vector<Real>& vt) {
  assert((int)vt.size() == this->NumNodes());
  for (int i = 0; i < this->NumNodes(); ++i)
    this->Nodes()[i]->Traits().Double(Tag::Tag_Twist_X) = vt[i];
}

void Mesh_DER::SetRotEulerVector(const vector<Vector3d>& vR) {
  assert((int)vR.size() == this->NumConns());
  for (int i = 0; i < this->NumConns(); ++i)
    this->Elems()[i]->Traits().Vector3d(Tag::Tag_Euler_X) = vR[i];
}

void Mesh_DER::SetRotMatrixVector(const vector<Matrix3d>& vR) {
  assert((int)vR.size() == this->NumConns());
  for (int i = 0; i < this->NumConns(); ++i)
    this->Elems()[i]->Traits().Matrix3d(Tag::Tag_Rotat_X) = vR[i];
}

void Mesh_DER::AdaptReferenceFrame(int i, Tag s) {
  Edge* pEdge = this->m_vedges[i];

  if (s == Tag::Tag_Position_X || s == Tag::Tag_Frame_X) {
    pEdge->Traits().Frame3d(Tag::Tag_Frame_X) =
        GeometryUtils::parallelTransport(
            pEdge->Traits().Frame3d(Tag::Tag_Frame_X),
            pEdge->Tangent(Tag::Tag_Position_X));
    return;
  }

  if (s == Tag::Tag_Position_0 || s == Tag::Tag_Frame_0) {
    pEdge->Traits().Frame3d(Tag::Tag_Frame_0) =
        GeometryUtils::parallelTransport(
            pEdge->Traits().Frame3d(Tag::Tag_Frame_0),
            pEdge->Tangent(Tag::Tag_Position_0));
    return;
  }

  throw PhySim::exception(
      "Invalid Tag, expected: PosX, Pos0, Frame_X, Frame_0");
}

void Mesh_DER::ComputeReferenceTwist(int i) {
  Node* pNode = this->m_vnodes[i];

  vector<Edge*> vprevEdges = PrevEdges(pNode);
  vector<Edge*> vnextEdges = NextEdges(pNode);

  if (vprevEdges.size() != 1)
    return;  // Extreme
  if (vnextEdges.size() != 1)
    return;  // Extreme

  Real& rtX = pNode->Traits().Double(Tag::Tag_Twist_X);
  Frame3d& Fprev = vprevEdges[0]->Traits().Frame3d(Tag::Tag_Frame_X);
  Frame3d& Fnext = vnextEdges[0]->Traits().Frame3d(Tag::Tag_Frame_X);
  rtX += computeReferenceTwistChange(Fprev.tan.data(), Fprev.nor.data(),
                                     Fnext.tan.data(), Fnext.nor.data(), rtX);
}

void Mesh_DER::UpdateConnectionRotation(int i) {
  Connection* pConn = this->m_vconns[i];

  Matrix3d& mstart = pConn->m_center->Traits().Matrix3d(Tag::Tag_Rotat_X);
  Vector3d& veuler = pConn->m_center->Traits().Vector3d(Tag::Tag_Euler_X);

  Matrix3d mrotat = mstart;
  GeometryUtils::eulerRotation(veuler.data(), mstart.col(0).data(),
                               mrotat.col(0).data());
  GeometryUtils::eulerRotation(veuler.data(), mstart.col(1).data(),
                               mrotat.col(1).data());
  GeometryUtils::eulerRotation(veuler.data(), mstart.col(2).data(),
                               mrotat.col(2).data());
  mstart = mrotat;
  veuler.setZero();

  for (int i = 0; i < (int)pConn->m_vedges.size(); ++i) {
    Real& rtX = pConn->m_vedges[i]->Traits().Double(Tag::Tag_Twist_X);
    Frame3d& F0 = pConn->m_vedges[i]->Traits().Frame3d(Tag::Tag_Frame_0);
    Frame3d& FX = pConn->m_vedges[i]->Traits().Frame3d(Tag::Tag_Frame_X);
    Frame3d FR0;
    FR0.tan = mrotat * F0.tan;
    FR0.nor = mrotat * F0.nor;
    FR0.bin = mrotat * F0.bin;
    rtX += computeReferenceTwistChange(FR0.tan.data(), FR0.nor.data(),
                                       FX.tan.data(), FX.nor.data(), rtX);
  }
}

void Mesh_DER::AdaptReferenceFrames(Tag s) {
  int N = (int)m_vedges.size();
  for (int i = 0; i < N; ++i)
    AdaptReferenceFrame(i, s);
}

void Mesh_DER::ComputeReferenceTwists() {
  int N = (int)m_vnodes.size();
  for (int i = 0; i < N; ++i)
    ComputeReferenceTwist(i);
}

void Mesh_DER::UpdateConnectionRotations() {
  int N = (int)m_vconns.size();
  for (int i = 0; i < N; ++i)
    UpdateConnectionRotation(i);
}

void Mesh_DER::FreeMetadata() {
  // Not implemented
}

void Mesh_DER::UpdateMetadata() {
  this->FreeMetadata();

  // Not implemented
}

}  // namespace PhySim