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

#include <PhySim/Utils/IOUtils.h>
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
  IOUtils::logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Mesh");
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