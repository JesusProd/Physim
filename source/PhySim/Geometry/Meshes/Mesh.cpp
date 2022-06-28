//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Meshes/Mesh.h>

#include <PhySim/Geometry/Polytopes/Cell_Hexa.h>
#include <PhySim/Geometry/Polytopes/Cell_Tetra.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Face_Quad.h>
#include <PhySim/Geometry/Polytopes/Face_Tri.h>

#include <PhySim/Geometry/Polytopes/Embedding.h>

#include <PhySim/Utils/MeshUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Mesh::Mesh() {
  this->Init(MatrixXd(), MatrixXi());
}

Mesh::Mesh(const Mesh& toCopy) {
  MatrixXi mE;
  MatrixXd mV;
  toCopy.GetElemMatrix(mE);
  toCopy.GetNodesTrait(mV, toCopy.NodeTraits().front());
  Init(mV, mE, toCopy.MeshType(), toCopy.NodeTraits());

  this->CloneTraits(toCopy);
}

Mesh::Mesh(int numN,
           const MatrixXi& mE,
           Discretization D,
           const vector<Tag>& vnTraits) {
  this->Init(numN, mE, D, vnTraits);
}

Mesh::Mesh(const MatrixXd& mV,
           const MatrixXi& mE,
           Discretization D,
           const vector<Tag>& vnTraits) {
  this->Init(mV, mE, D, vnTraits);
}

void Mesh::Init(int numN,
                const MatrixXi& mE,
                Discretization D,
                const vector<Tag>& vnTraits) {
  this->Init(MatrixXd::Zero(numN, 3), mE, D, vnTraits);
}

void Mesh::Init(const MatrixXd& mV,
                const MatrixXi& mE,
                Discretization D,
                const vector<Tag>& vnTraits) {
  this->FreeInternal();

  this->m_meshType = D;

  int numN = (int)mV.rows();
  int numE = (int)mE.rows();

  this->m_vnodes.reserve(numN);
  this->m_velems.reserve(numE);

  this->m_vnodeTraits = vnTraits;
  if (this->m_vnodeTraits.empty())
    this->m_vnodeTraits.push_back(Tag_Position_X);

  // Initialize nodes

  for (int i = 0; i < numN; ++i) {
    this->m_vnodes.push_back(new Node(i, mV.row(i), this->m_vnodeTraits));

    this->m_vnodes.back()->SetMesh(this);
  }

  // Initialize elements

  for (int i = 0; i < numE; ++i) {
    vector<Node*> vnodes(mE.cols());
    for (int j = 0; j < mE.cols(); ++j)
      vnodes[j] = this->m_vnodes[mE(i, j)];

    switch (D) {
      case Discretization_Nodes:
        this->m_dimBasis = 0;
        throw exception("Not implemented");
        break;
      case Discretization_Edge2:
      case Discretization_Edge3:
        this->m_dimBasis = 1;
        this->m_velems.push_back(new Edge(i, vnodes));
        break;
      case Discretization_Tri3:
        this->m_dimBasis = 2;
        this->m_velems.push_back(new Face_Tri3(i, vnodes));
        break;
      case Discretization_Tri6:
        this->m_dimBasis = 2;
        this->m_velems.push_back(new Face_Tri6(i, vnodes));
        break;
      case Discretization_Quad4:
        this->m_dimBasis = 2;
        this->m_velems.push_back(new Face_Quad4(i, vnodes));
        break;
      case Discretization_Quad8:
        this->m_dimBasis = 2;
        this->m_velems.push_back(new Face_Quad8(i, vnodes));
        break;
      case Discretization_Tet4:
        this->m_dimBasis = 3;
        this->m_velems.push_back(new Cell_Tetra4(i, vnodes));
        break;
      case Discretization_Tet10:
        this->m_dimBasis = 3;
        this->m_velems.push_back(new Cell_Tetra10(i, vnodes));
        break;
      case Discretization_Hex8:
        this->m_dimBasis = 3;
        this->m_velems.push_back(new Cell_Hexa8(i, vnodes));
        break;
    }

    this->m_velems.back()->SetMesh(this);
  }
  this->m_dimSpace = 3;
  this->UpdateMetadata();

  this->m_ID;
}

Mesh::~Mesh(void) {
  this->FreeInternal();

#ifndef NDEBUG
  logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Mesh");
#endif
}

void Mesh::Merge(PtrS<Mesh>& pMesh,
                 const vector<PtrS<Mesh>>& vmeshes,
                 Tag trait,
                 Real tol) {
  assert(pMesh != NULL);

  // Build a mesh from the combination of a sequence of meshs
  // with potentially common nodes. Elements remain the same.

  int Nm = (int)vmeshes.size();

  vector<MatrixXd> vmV(Nm);
  vector<MatrixXi> vmE(Nm);
  vector<VectorXi> vmaps;
  MatrixXd mV;
  MatrixXi mE;

  for (int i = 0; i < Nm; ++i) {
    vmeshes[i]->GetNodesTrait(vmV[i], trait);
    vmeshes[i]->GetElemMatrix(vmE[i]);
  }

  // Merge nodes/edges

  MeshUtils::mergeDuplicatedNodes(vmV, vmE, mV, mE, vmaps);

  // Initialize new mesh

  pMesh->Init(mV, mE, vmeshes.front()->MeshType(),
              vmeshes.front()->NodeTraits());

  // Copy traits

  vector<TraitSet> vnodeTraits;
  vector<TraitSet> velemTraits;
  vnodeTraits.resize(mV.rows());
  velemTraits.reserve(mE.rows());

  for (int i = 0; i < Nm; ++i) {
    for (int j = 0; j < vmeshes[i]->NumNodes(); ++j) {
      vnodeTraits[vmaps[i](j)] = vmeshes[i]->Nodes()[j]->Traits();
    }

    for (int j = 0; j < vmeshes[i]->NumElems(); ++j) {
      velemTraits.push_back(vmeshes[i]->Elems()[j]->Traits());
    }
  }

  pMesh->SetNodesTrait(vnodeTraits);
  pMesh->SetElemsTrait(velemTraits);

  // Copy tags

  for (int i = 0; i < (int)vmeshes[0]->NodeTraits().size(); ++i)
    if (!pMesh->HasNodeTraits(vmeshes[0]->NodeTraits()[i]))
      pMesh->m_vnodeTraits.push_back(vmeshes[0]->NodeTraits()[i]);

  for (int i = 0; i < (int)vmeshes[0]->ElemTraits().size(); ++i)
    if (!pMesh->HasElemTraits(vmeshes[0]->ElemTraits()[i]))
      pMesh->m_velemTraits.push_back(vmeshes[0]->ElemTraits()[i]);
}

void Mesh::FreeInternal() {
  for (int i = 0; i < this->m_vnodes.size(); ++i)
    delete this->m_vnodes[i];

  for (int i = 0; i < this->m_velems.size(); ++i)
    delete this->m_velems[i];

  this->m_vnodes.clear();
  this->m_velems.clear();

  this->FreeMetadata();
}

void Mesh::CloneTraits(const Mesh& toCopy) {
  // Copy node traits

  for (int i = 0; i < toCopy.NumNodes(); ++i) {
    this->m_vnodes[i]->Traits().CloneTraits(toCopy.m_vnodes[i]->Traits());
  }

  // Copy elem traits

  for (int i = 0; i < toCopy.NumElems(); ++i) {
    this->m_velems[i]->Traits().CloneTraits(toCopy.m_velems[i]->Traits());
  }

  // Copy mesh traits

  this->Traits().CloneTraits(toCopy.Traits());
}

void Mesh::GetElemMatrix(MatrixXi& mE) const {
  mE.resize(NumElems(), NumElemNodes());
  for (int i = 0; i < this->NumElems(); ++i)
    for (int j = 0; j < this->NumElemNodes(); ++j)
      mE(i, j) = this->m_velems[i]->Nodes()[j]->ID();
}

// void Mesh::GetNodesTrait(vector<TraitSet>& vtraits) const
//{
//	vtraits.resize(this->m_vnodes.size());

//	for (int i = 0; i < (int)this->m_vnodes.size(); ++i)
//		vtraits[i].CloneTraits(this->m_vnodes[i]->Traits());
//}

// void Mesh::SetNodesTrait(const vector<TraitSet>& vtraits)
//{
//	assert(vtraits.size() == m_vnodes.size());

//	for (int i = 0; i < (int)this->m_vnodes.size(); ++i)
//		this->m_vnodes[i]->Traits().CloneTraits(vtraits[i]);
//}

// void Mesh::GetElemsTrait(vector<TraitSet>& vtraits) const
//{
//	vtraits.resize(this->m_velems.size());

//	for (int i = 0; i < (int)this->m_velems.size(); ++i)
//		this->m_velems[i]->Traits().CloneTraits(vtraits[i]);
//}

// void Mesh::SetElemsTrait(const vector<TraitSet>& vtraits)
//{
//	assert(vtraits.size() == m_velems.size());

//	for (int i = 0; i < (int)this->m_velems.size(); ++i)
//		this->m_velems[i]->Traits().CloneTraits(vtraits[i]);
//}

// void Mesh::GetNodesTrait(MatrixXd& mN, Tag s) const
//{
//	int numNodes = (int) this->m_vnodes.size();

//	mN.resize(numNodes, 3);
//	for (int i = 0; i < numNodes; ++i)
//		mN.row(i) = m_vnodes[i]->Traits().Vector3d(s);
//}

// void Mesh::GetElemsTrait(MatrixXd& mN, Tag s) const
//{
//	int numElems = (int)this->m_velems.size();

//	mN.resize(numElems, 3);
//	for (int i = 0; i < numElems; ++i)
//		mN.row(i) = m_velems[i]->Traits().Vector3d(s);
//}

// void Mesh::SetNodesTrait(const MatrixXd& mN, Tag s)
//{
//	int numNodes = (int) this->m_vnodes.size();
//	assert(mN.rows() == numNodes && mN.cols() == 3);

//	for (int i = 0; i < numNodes; ++i)
//	{
//		if (!m_vnodes[i]->Traits().HasTrait(s))
//			m_vnodes[i]->Traits().AddTrait(s, Vector3d(mN.row(i)));
//		else m_vnodes[i]->Traits().Vector3d(s) = Vector3d(mN.row(i));
//	}
//}

// void Mesh::SetElemsTrait(const MatrixXd& mN, Tag s)
//{
//	int numElems = (int)this->m_velems.size();
//	assert(mN.rows() == numElems && mN.cols() == 3);

//	for (int i = 0; i < numElems; ++i)
//	{
//		if (!m_velems[i]->Traits().HasTrait(s))
//			m_velems[i]->Traits().AddTrait(s, Vector3d(mN.row(i)));
//		else m_velems[i]->Traits().Vector3d(s) = Vector3d(mN.row(i));
//	}
//}

// void Mesh::DelNodesTrait(Tag s)
//{
//	int numNodes = (int) this->m_vnodes.size();

//	for (int i = 0; i < numNodes; ++i)
//		m_vnodes[i]->Traits().RemTrait(s);
//}

// void Mesh::DelElemsTrait(Tag s)
//{
//	int numElems = (int)this->m_velems.size();

//	for (int i = 0; i < numElems; ++i)
//		m_velems[i]->Traits().RemTrait(s);
//}

Real Mesh::VolumeBasis(Tag s) const {
  Real volume = 0;
  for (int i = 0; i < this->m_velems.size(); ++i)
    volume += m_velems[i]->VolumeBasis(s);

  return volume;
}

Vector3d Mesh::Centroid(Tag s) const {
  Vector3d centroid;
  centroid.setZero();

  for (int i = 0; i < this->m_vnodes.size(); ++i)
    centroid += this->m_vnodes[i]->Traits().Vector3d(s);

  centroid /= (int)m_vnodes.size();

  return centroid;
}

Matrix3d Mesh::Rotation(Tag f, Tag t) const {
  throw exception("Not implemented");

  return Matrix3d::Identity();
}

// bool Mesh::HasNodeEmbedding() const
//{
//	return this->m_vnodes
//}

// vector<Embedding>& Mesh::NodeEmbedding()
//{
//	return this->m_vEmbedding;
//}

// const vector<Embedding>& Mesh::NodeEmbedding() const
//{
//	return this->m_vEmbedding;
//}

// void Mesh::InterpolateEmbeddedNodeTraits(Tag s)
//{
//	for (int i = 0; i < this->NumNodes(); ++i)
//		if (this->Nodes()[i]->Traits().HasTrait(Tag::Tag_Embedding_0))
//			this->Nodes()[i]->Traits().Vector3d(s) =
//this->Nodes()[i]->Trait<Embedding>(Tag::Tag_Embedding_0).InterpolateValue(s);
//}

// void Mesh::InterpolateEmbeddedDeformation(Tag s0, Tag sx)
//{
//	for (int i = 0; i < this->NumNodes(); ++i)
//		if (this->Nodes()[i]->Traits().HasTrait(Tag::Tag_Embedding_0))
//			this->Nodes()[i]->Traits().Vector3d(sx) =
//this->Nodes()[i]->Trait<Embedding>(Tag::Tag_Embedding_0).InterpolateDeformation(s0,
//sx);
//}

// PtrS<Embedding> Mesh::ComputeEmbedding(const Vector3d& vx, Tag s)
//{
//	PtrS<Embedding> embedding;

//	for (int j = 0; j < this->NumElems(); ++j)
//	{
//		embedding = this->Elems()[j]->ComputeEmbedding(vx, s);

//		if (embedding->Valid())
//		{
//			break;
//		}
//	}

//	return embedding;
//}

// PtrS<Embedding> Mesh::ComputeProjection(const Vector3d& vx, Tag s)
//{
//	PtrS<Embedding> projection;

//	Real minDist2 = HUGE_VAL;

//	for (size_t j = 0; j < this->NumElems(); ++j)
//	{
//		PtrS<Embedding> emb = Elems()[j]->ComputeProjection(vx, s);
//		Real dist2 = (vx - emb->InterpolateValue(s)).squaredNorm();
//		if (dist2 < minDist2)
//		{
//			minDist2 = dist2;
//			projection = emb;
//		}
//	}

//	return projection;
//}

// void Mesh::ComputeEmbedding(const MatrixXd& vpoints, const
// std::vector<Poly*>& velems, vector<PtrS<Embedding>>& vembed, Tag s) const
//{
//	assert(vpoints.rows() == velems.size() && "The number of points and
//elements mismatch!");

//	vembed.resize(vpoints.rows());

//	for (int i = 0; i < vpoints.rows(); ++i)
//	{
//		assert(velems[i] != nullptr && "Invalid element");
//		assert(velems[i]->GetMesh() == this && "The given element is owned
//by another mesh.");

//		vembed[i] = velems[i]->ComputeEmbedding(vpoints.row(i), s);
//		assert(vembed[i].Valid() && "The given point lies outside of the
//element.");
//	}
//}

// void Mesh::ComputeEmbedding(vector<PtrS<Embedding>>& vembed, const VectorXi&
// vsel)
//{
//	if (vsel.size() == 0)
//	{
//		vembed.resize(this->NumNodes());

//		for (size_t i = 0; i < this->NumNodes(); ++i)
//		{
//			vembed[i] = make_shared<Embedding>(this->Nodes()[i],
//VectorXd::Zero(1));
//		}
//	}
//	else
//	{
//		vembed.resize(vsel.size());

//		for (size_t i = 0; i < (int)vsel.size(); ++i)
//		{
//			vembed[i] = make_shared<Embedding>(this->Nodes()[vsel[i]],
//VectorXd::Zero(1));
//		}
//	}
//}

// void Mesh::ComputeProjection(const MatrixXd& vpoints,
// vector<PtrS<Embedding>>& vembed, Tag s)
//{
//	vembed.resize(vpoints.rows());

//	for (int i = 0; i < vpoints.rows(); ++i)
//	{
//		vembed[i] = this->ComputeProjection(vpoints.row(i), s);
//	}
//}

// void Mesh::ComputeEmbedding(const MatrixXd& vpoints, vector<PtrS<Embedding>>&
// vembed, Tag s)
//{
//	vembed.resize(vpoints.rows());

//	for (int i = 0; i < vpoints.rows(); ++i)
//	{
//		vembed[i] = this->ComputeEmbedding(vpoints.row(i), s);
//	}
//}

// void Mesh::EmbedNodes(vector<Node*>& vtoEmbed, const Tag& trait)
//{
//	vector<PtrS<Embedding>> vembed(vtoEmbed.size());

//	MatrixXd mPoints(vtoEmbed.size(), 3);
//	for (int i = 0; i < (int)vtoEmbed.size(); ++i)
//		mPoints.row(i) = vtoEmbed[i]->Traits().Vector3d(trait);

//	this->ComputeEmbedding(mPoints, vembed, trait);

//	for (int i = 0; i < (int)vtoEmbed.size(); ++i)
//		vtoEmbed[i]->Traits().AddTrait<Embedding>(Tag::Tag_Embedding_0,
//*vembed[i]);
//}

// void Mesh::EmbedNodes(vector<Node*>& vtoEmbed, const VectorXi& vsel)
//{
//	vector<PtrS<Embedding>> vembed(vtoEmbed.size());

//	this->ComputeEmbedding(vembed, vsel);

//	for (int i = 0; i < (int)vtoEmbed.size(); ++i)
//		vtoEmbed[i]->Traits().AddTrait<Embedding>(Tag::Tag_Embedding_0,
//*vembed[i]);
//}

// void Mesh::EmbedMesh(Mesh& mesh, const Tag& trait)
//{
//	this->EmbedNodes(mesh.Nodes(), trait);
//}

// void Mesh::EmbedMesh(Mesh& mesh, const VectorXi& vsel)
//{
//	this->EmbedNodes(mesh.Nodes(), vsel);
//}

// void Mesh::Scale(const VectorXd& vs, Tag s)
//{
//	this->Transform(vs.asDiagonal(), s);
//}

// void Mesh::Rotate(const VectorXd& vaxi, Real ang, Tag s)
//{
//	this->Transform(Utils::rotationAxisAngleToMatrix(vaxi*ang), s);
//}

// void Mesh::Translate(const VectorXd& vt, Tag s)
//{
//	MatrixXd mV;
//	this->GetNodesTrait(mV, s);
//	mV = mV.rowwise() + vt.transpose();
//	this->SetNodesTrait(mV, s);
//}

// void Mesh::Transform(const MatrixXd& mT, Tag s)
//{
//	MatrixXd mV;
//	this->GetNodesTrait(mV, s);
//	mV = (mT*mV.transpose()).transpose();;
//	this->SetNodesTrait(mV, s);
//}

void Mesh::UpdateMetadata() {
  this->FreeMetadata();
}

void Mesh::FreeMetadata() {
  // Nothing to do here...
}

void Mesh::MassProperties(Tag s,
                          Real rho,
                          Real& mass,
                          Vector3d& vcom,
                          Matrix3d& mI) const {
  throw PhySim::exception("Not implemented");
}

}  // namespace PhySim