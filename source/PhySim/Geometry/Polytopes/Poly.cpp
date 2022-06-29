//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Poly.h>

#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Embedding.h>
#include <PhySim/Geometry/Polytopes/Face.h>
#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/ShapeFunctions.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Poly::Poly() {
  this->m_pMesh = NULL;
  this->m_ID = -1;
}

Poly::Poly(int ID) {
  this->m_ID = ID;
  this->m_pMesh = NULL;
}

Poly::Poly(const Poly& toCopy) {
  this->m_ID = toCopy.m_ID;
  this->m_vnodes = toCopy.m_vnodes;
  this->m_vedges = toCopy.m_vedges;
  this->m_vfaces = toCopy.m_vfaces;
  this->m_vcells = toCopy.m_vcells;

  this->m_pSFunction = toCopy.m_pSFunction;

  this->m_pMesh = toCopy.m_pMesh;

  this->Traits().CloneTraits(toCopy.Traits());
}

Poly::~Poly() {
  this->m_pMesh = NULL;
}

Real Poly::VolumeSpace(Tag s) const {
  throw exception("Not implemented");

  return -1;
}

// void Poly::GetNodesTrait(MatrixXd& mN, Tag s) const
//{
//	int N = (int) this->m_vnodes.size();

//	mN.resize(N,3);
//	for (int i = 0; i < N; ++i)
//		mN.row(i) = m_vnodes[i]->Traits().Vector3d(s);
//}

// void Poly::GetElemsTrait(MatrixXd& mN, Tag s) const
//{
//	mN.resize(1, 3);

//	mN.row(0) = this->Traits().Vector3d(s);
//}

// void Poly::SetNodesTrait(const MatrixXd& mN, Tag s)
//{
//	int N = (int) this->m_vnodes.size();

//	assert(mN.rows() == N);
//	assert(mN.cols() == 3);

//	for (int i = 0; i < N; ++i)
//	{
//		if (!m_vnodes[i]->Traits().HasTrait(s))
//			m_vnodes[i]->Traits().AddTrait(s, Vector3d(mN.row(i)));
//		else m_vnodes[i]->Traits().Vector3d(s) = Vector3d(mN.row(i));
//	}
//}

// void Poly::SetElemsTrait(const MatrixXd& mN, Tag s)
//{
//	assert(mN.rows() == 1);
//	assert(mN.cols() == 3);

//	if (!this->Traits().HasTrait(s))
//			this->Traits().AddTrait(s, Vector3d(mN.row(0)));
//	else this->Traits().Vector3d(s) = Vector3d(mN.row(0));
//}

// void Poly::DelNodesTrait(Tag s)
//{
//	int numNodes = (int) this->m_vnodes.size();

//	for (int i = 0; i < numNodes; ++i)
//		m_vnodes[i]->Traits().RemTrait(s);
//}

// void Poly::DelElemsTrait(Tag s)
//{
//	this->Traits().RemTrait(s);
//}

Vector3d Poly::Centroid(Tag s) const {
  Vector3d vsum = Vector3d::Zero();

  size_t numNode = this->m_vnodes.size();
  for (size_t i = 0; i < numNode; ++i)
    vsum += this->m_vnodes[i]->Traits().Vector3d(s);

  return (1.0 / numNode) * vsum;
}

Matrix3d Poly::Rotation(Tag f, Tag t) const {
  throw exception("Not implemented");

  return Matrix3d();
}

void Poly::MassProperties(Tag s,
                          Real rho,
                          Real& mass,
                          Vector3d& vcom,
                          Matrix3d& mI) const {
  throw exception("Not implemented");
}

MatrixXd Poly::DeformationGradient(const VectorXd& vpoint, Tag f, Tag t) const {
  PtrS<IShapeFunction::IDeformationData> pData =
      this->m_pSFunction->CreateDeformationData();
  pData->SetPoly(this);
  pData->SetTag0(f);
  pData->SetTagX(t);
  vector<VectorXd> vvp;
  vvp.push_back(vpoint);
  pData->SetPoints(vvp);
  this->m_pSFunction->InitDeformationAtSamples(pData);
  this->m_pSFunction->UpdateDeformationAtSamples(pData);
  return pData->Get_F()[0];
}

VectorXd Poly::InterpolateValue(const VectorXd& vpoint, Tag trait) const {
  MatrixXd mP(1, vpoint.size());
  MatrixXd mI(1, vpoint.size());

  mP.row(0) = vpoint;

  this->m_pSFunction->InterpolateValue(this, trait, mP, mI);

  return mI.row(0);
}

VectorXd Poly::InterpolateDeformation(const VectorXd& vpoint,
                                      Tag trait0,
                                      Tag traitX) const {
  MatrixXd mP(1, vpoint.size());
  MatrixXd mI(1, vpoint.size());

  mP.row(0) = vpoint;

  this->m_pSFunction->InterpolateDeformation(this, trait0, traitX, mP, mI);

  return mI.row(0);
}

void Poly::TransformNat2Iso(VectorXd& vb, MatrixXd& mA, Tag s) const {
  throw exception("Not implemented");
}

void Poly::TransformIso2Nat(VectorXd& vb, MatrixXd& mA, Tag s) const {
  throw exception("Not implemented");
}

PtrS<Embedding> Poly::ComputeEmbedding(const Vector3d& vp, Tag s) {
  Node* pNode = NULL;
  VectorXd vb;
  MatrixXd mA;
  TransformNat2Iso(vb, mA, s);
  VectorXd viso = mA * vp + vb;
  return make_shared<Embedding>(this, viso);
}

PtrS<Embedding> Poly::ComputeProjection(const Vector3d& vx, Tag s) {
  PtrS<Embedding> pe = this->ComputeEmbedding(vx, s);

  if (pe->Valid()) {
    return pe;
  } else {
    Vector3d minPoint;
    Real minDist2 = HUGE_VAL;

    for (int i = 0; i < this->NumNodes(); ++i) {
      const Vector3d& vnode = this->m_vnodes[i]->Traits().Vector3d(s);
      Real dist2 = (vx - vnode).squaredNorm();
      if (dist2 < minDist2) {
        minDist2 = dist2;
        minPoint = vnode;
      }
    }

    return this->ComputeEmbedding(minPoint, s);
  }
}

bool Poly::IsValidParametric(const VectorXd& vp) const {
  return this->m_pSFunction->IsValidParametric(vp);
}

void Poly::InitSubelementPositions(Tag s) {
  throw exception("Not implemented");
}

}  // namespace PhySim