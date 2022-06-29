//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/CommonIncludes.h>

#include <PhySim/Geometry/Geometry.h>
#include <PhySim/Geometry/Meshes/Mesh.h>
#include <PhySim/Geometry/Polytopes/Embedding.h>
#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/Poly.h>

#include <PhySim/Utils/GeometryUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

PtrS<Embedding> Geometry::ComputeEmbedding(const Vector3d& vx, Tag s) {
  PtrS<Embedding> embedding;

  for (int j = 0; j < this->NumElems(); ++j) {
    embedding = Elems()[j]->ComputeEmbedding(vx, s);

    if (embedding->Valid()) {
      break;
    }
  }

  return embedding;
}

PtrS<Embedding> Geometry::ComputeProjection(const Vector3d& vx, Tag s) {
  PtrS<Embedding> projection;

  Real minDist2 = HUGE_VAL;

  for (size_t j = 0; j < this->NumElems(); ++j) {
    PtrS<Embedding> emb = Elems()[j]->ComputeProjection(vx, s);
    Real dist2 = (vx - emb->InterpolateValue(s)).squaredNorm();
    if (dist2 < minDist2) {
      minDist2 = dist2;
      projection = emb;
    }
  }

  return projection;
}

void Geometry::InterpolateEmbeddedNodeTraits(Tag s) {
  for (int i = 0; i < this->NumNodes(); ++i)
    if (this->Nodes()[i]->Traits().HasTrait(Tag::Tag_Embedding_0))
      this->Nodes()[i]->Traits().Vector3d(s) =
          this->Nodes()[i]
              ->Trait<Embedding>(Tag::Tag_Embedding_0)
              .InterpolateValue(s);
}

void Geometry::InterpolateEmbeddedDeformation(Tag s0, Tag sx) {
  for (int i = 0; i < this->NumNodes(); ++i)
    if (this->Nodes()[i]->Traits().HasTrait(Tag::Tag_Embedding_0))
      this->Nodes()[i]->Traits().Vector3d(sx) =
          this->Nodes()[i]
              ->Trait<Embedding>(Tag::Tag_Embedding_0)
              .InterpolateDeformation(s0, sx);
}

void Geometry::ComputeEmbedding(vector<PtrS<Embedding>>& vembed,
                                const VectorXi& vsel) {
  if (vsel.size() == 0) {
    vembed.resize(this->NumNodes());

    for (size_t i = 0; i < this->NumNodes(); ++i) {
      vembed[i] = make_shared<Embedding>(this->Nodes()[i], VectorXd::Zero(1));
    }
  } else {
    vembed.resize(vsel.size());

    for (size_t i = 0; i < (int)vsel.size(); ++i) {
      vembed[i] =
          make_shared<Embedding>(this->Nodes()[vsel[i]], VectorXd::Zero(1));
    }
  }
}

void Geometry::ComputeProjection(const MatrixXd& vpoints,
                                 vector<PtrS<Embedding>>& vembed,
                                 Tag s) {
  vembed.resize(vpoints.rows());

  for (int i = 0; i < vpoints.rows(); ++i) {
    vembed[i] = this->ComputeProjection(vpoints.row(i), s);
  }
}

void Geometry::ComputeEmbedding(const MatrixXd& vpoints,
                                vector<PtrS<Embedding>>& vembed,
                                Tag s) {
  vembed.resize(vpoints.rows());

  for (int i = 0; i < vpoints.rows(); ++i) {
    vembed[i] = this->ComputeEmbedding(vpoints.row(i), s);
  }
}

void Geometry::EmbedNodes(vector<Node*>& vtoEmbed, const Tag& trait) {
  vector<PtrS<Embedding>> vembed(vtoEmbed.size());

  MatrixXd mPoints(vtoEmbed.size(), 3);
  for (int i = 0; i < (int)vtoEmbed.size(); ++i)
    mPoints.row(i) = vtoEmbed[i]->Traits().Vector3d(trait);

  this->ComputeEmbedding(mPoints, vembed, trait);

  for (int i = 0; i < (int)vtoEmbed.size(); ++i)
    vtoEmbed[i]->Traits().AddTrait<Embedding>(Tag::Tag_Embedding_0, *vembed[i]);
}

void Geometry::EmbedNodes(vector<Node*>& vtoEmbed, const VectorXi& vsel) {
  vector<PtrS<Embedding>> vembed(vtoEmbed.size());

  this->ComputeEmbedding(vembed, vsel);

  for (int i = 0; i < (int)vtoEmbed.size(); ++i)
    vtoEmbed[i]->Traits().AddTrait<Embedding>(Tag::Tag_Embedding_0, *vembed[i]);
}

void Geometry::EmbedMesh(Mesh& mesh, const Tag& trait) {
  this->EmbedNodes(mesh.Nodes(), trait);
}

void Geometry::EmbedMesh(Mesh& mesh, const VectorXi& vsel) {
  this->EmbedNodes(mesh.Nodes(), vsel);
}

void Geometry::Scale(const VectorXd& vs, Tag s) {
  this->Transform(vs.asDiagonal(), s);
}

void Geometry::RotateAxis(const VectorXd& vaxi, Tag s) {
  this->Transform(GeometryUtils::rotationAxisAngleToMatrix(vaxi), s);
}

void Geometry::RotateEuler(const VectorXd& veul, Tag s) {
  this->Transform(GeometryUtils::rotationEulerToMatrix(veul), s);
}

void Geometry::Translate(const VectorXd& vt, Tag s) {
  MatrixXd mV;
  this->GetNodesTrait(mV, s);
  mV = mV.rowwise() + vt.transpose();
  this->SetNodesTrait(mV, s);
}

void Geometry::Transform(const MatrixXd& mT, Tag s) {
  MatrixXd mV;
  this->GetNodesTrait(mV, s);
  mV = (mT * mV.transpose()).transpose();
  ;
  this->SetNodesTrait(mV, s);
}

void Geometry::GetNodesTrait(MatrixXd& mN, Tag s) const {
  int numNodes = (int)this->m_vnodes.size();

  mN.resize(numNodes, 3);
  for (int i = 0; i < numNodes; ++i)
    mN.row(i) = m_vnodes[i]->Traits().Vector3d(s);
}

void Geometry::GetElemsTrait(MatrixXd& mN, Tag s) const {
  int numElems = (int)this->m_velems.size();

  mN.resize(numElems, 3);
  for (int i = 0; i < numElems; ++i)
    mN.row(i) = m_velems[i]->Traits().Vector3d(s);
}

void Geometry::SetNodesTrait(const MatrixXd& mN, Tag s) {
  int numNodes = (int)this->m_vnodes.size();
  assert(mN.rows() == numNodes && mN.cols() == 3);

  for (int i = 0; i < numNodes; ++i) {
    if (!m_vnodes[i]->Traits().HasTrait(s))
      m_vnodes[i]->Traits().AddTrait(s, Vector3d(mN.row(i)));
    else
      m_vnodes[i]->Traits().Vector3d(s) = Vector3d(mN.row(i));
  }
}

void Geometry::SetElemsTrait(const MatrixXd& mN, Tag s) {
  int numElems = (int)this->m_velems.size();
  assert(mN.rows() == numElems && mN.cols() == 3);

  for (int i = 0; i < numElems; ++i) {
    if (!m_velems[i]->Traits().HasTrait(s))
      m_velems[i]->Traits().AddTrait(s, Vector3d(mN.row(i)));
    else
      m_velems[i]->Traits().Vector3d(s) = Vector3d(mN.row(i));
  }
}

void Geometry::DelNodesTrait(Tag s) {
  int numNodes = (int)this->m_vnodes.size();

  for (int i = 0; i < numNodes; ++i)
    m_vnodes[i]->Traits().RemTrait(s);
}

void Geometry::DelElemsTrait(Tag s) {
  int numElems = (int)this->m_velems.size();

  for (int i = 0; i < numElems; ++i)
    m_velems[i]->Traits().RemTrait(s);
}

template <class T>
void Geometry::GetNodesTrait(vector<T>& vvalues, Tag s) const {
  vvalues.resize(this->m_vnodes.size());

  for (int i = 0; i < (int)this->m_vnodes.size(); ++i)
    vvalues[i] = this->m_vnodes[i]->Traits().Trait<T>(s);
}

template <class T>
void Geometry::SetNodesTrait(const vector<T>& vvalues, Tag s) {
  assert(vvalues.size() == this->m_vnodes.size());

  int numNodes = (int)this->m_vnodes.size();

  for (int i = 0; i < numNodes; ++i) {
    if (!m_vnodes[i]->Traits().HasTrait(s))
      m_vnodes[i]->Traits().AddTrait(s, vvalues[i]);
    else
      m_vnodes[i]->Traits().Trait<T>(s) = vvalues[i];
  }
}

template <class T>
void Geometry::GetElemsTrait(vector<T>& vvalues, Tag s) const {
  vvalues.resize(this->m_velems.size());

  for (int i = 0; i < (int)this->m_velems.size(); ++i)
    vvalues[i] = this->m_velems[i]->Traits().Trait<T>(s);
}

template <class T>
void Geometry::SetElemsTrait(const vector<T>& vvalues, Tag s) {
  assert(vvalues.size() == this->m_velems.size());

  int numElems = (int)this->m_velems.size();

  for (int i = 0; i < numElems; ++i) {
    if (!m_velems[i]->Traits().HasTrait(s))
      m_velems[i]->Traits().AddTrait(s, vvalues[i]);
    else
      m_velems[i]->Traits().Trait<T>(s) = vvalues[i];
  }
}

void Geometry::GetNodesTrait(vector<TraitSet>& vtraits) const {
  vtraits.resize(this->m_vnodes.size());

  for (int i = 0; i < (int)this->m_vnodes.size(); ++i)
    vtraits[i].CloneTraits(this->m_vnodes[i]->Traits());
}

void Geometry::SetNodesTrait(const vector<TraitSet>& vtraits) {
  assert(vtraits.size() == m_vnodes.size());

  for (int i = 0; i < (int)this->m_vnodes.size(); ++i)
    this->m_vnodes[i]->Traits().CloneTraits(vtraits[i]);
}

void Geometry::GetElemsTrait(vector<TraitSet>& vtraits) const {
  vtraits.resize(this->m_velems.size());

  for (int i = 0; i < (int)this->m_velems.size(); ++i)
    this->m_velems[i]->Traits().CloneTraits(vtraits[i]);
}

void Geometry::SetElemsTrait(const vector<TraitSet>& vtraits) {
  assert(vtraits.size() == m_velems.size());

  for (int i = 0; i < (int)this->m_velems.size(); ++i)
    this->m_velems[i]->Traits().CloneTraits(vtraits[i]);
}

}  // namespace PhySim