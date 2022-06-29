//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/CommonIncludes.h>

#include <PhySim/Geometry/Meshes/Mesh.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Face.h>
#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Physics/DoFSet.h>
#include <PhySim/Physics/Simulables/Simulable_Mesh.h>

#pragma once

namespace PhySim {
using namespace std;
using namespace Eigen;

class GeometryFilter {
 public:
  GeometryFilter() {}
  virtual ~GeometryFilter() {}

  virtual string Name() const = 0;

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeometry) const = 0;

  virtual vector<Geometry*> Filter(vector<PtrS<Geometry>> vpGeometry) const;
};

class GeometryFilter_AllNode : public GeometryFilter {
 public:
  GeometryFilter_AllNode() {}
  virtual ~GeometryFilter_AllNode() {}

  virtual string Name() const override { return "AllNode"; }

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeom) const override;
};

class GeometryFilter_AllElem : public GeometryFilter {
 public:
  GeometryFilter_AllElem() {}

  virtual string Name() const override { return "AllElem"; }

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeom) const;
};

class GeometryFilter_IndexNode : public GeometryFilter {
 public:
  iVector m_vindices;

  GeometryFilter_IndexNode(const iVector& vindices) { m_vindices = vindices; }

  virtual ~GeometryFilter_IndexNode() {}

  virtual string Name() const override { return "IndexNode"; }

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeom) const override {
    vector<Geometry*> vfiltered;

    vfiltered.reserve(m_vindices.size());

    for (int i = 0; i < m_vindices.size(); ++i)
      vfiltered.push_back(pGeom->Nodes()[m_vindices[i]]);

    return vfiltered;
  }
};

class GeometryFilter_IndexElem : public GeometryFilter {
 public:
  iVector m_vindices;

  GeometryFilter_IndexElem(const iVector& vindices) { m_vindices = vindices; }

  virtual ~GeometryFilter_IndexElem() {}

  virtual string Name() const override { return "IndexElem"; }

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeom) const override {
    vector<Geometry*> vfiltered;

    PtrS<Mesh> pMesh = dynamic_pointer_cast<Mesh>(pGeom);
    if (pMesh == NULL)
      return vector<Geometry*>();

    vfiltered.reserve((int)m_vindices.size());

    for (int i = 0; i < m_vindices.size(); ++i)
      vfiltered.push_back(pMesh->Elems()[m_vindices[i]]);

    return vfiltered;
  }
};

class GeometryFilter_BoundaryNode : public GeometryFilter {
 public:
  static const string NAME;

  GeometryFilter_BoundaryNode() {
    // Nothing to do here...
  }

  virtual ~GeometryFilter_BoundaryNode() {}

  virtual string Name() const override { return "BoundaryNodes"; }

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeom) const override {
    vector<Geometry*> vfiltered;

    PtrS<Mesh> pMesh = dynamic_pointer_cast<Mesh>(pGeom);
    if (pMesh == NULL)
      return vector<Geometry*>();

    vector<Node*> vbnode;
    pMesh->GetBoundaryNodes(vbnode);
    vfiltered.reserve((int)vbnode.size());
    for (int i = 0; i < vbnode.size(); ++i)
      vfiltered.push_back(vbnode[i]);

    return vfiltered;
  }
};

class GeometryFilter_BoundaryEdge : public GeometryFilter {
 public:
  GeometryFilter_BoundaryEdge() {
    // Nothing to do here...
  }

  virtual ~GeometryFilter_BoundaryEdge() {}

  virtual string Name() const override { return "BoundaryEdges"; }

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeom) const override {
    vector<Geometry*> vfiltered;

    PtrS<Mesh> pMesh = dynamic_pointer_cast<Mesh>(pGeom);
    if (pMesh == NULL)
      return vector<Geometry*>();

    vector<Edge*> vbedge;
    pMesh->GetBoundaryEdges(vbedge);
    vfiltered.reserve((int)vbedge.size());
    for (int i = 0; i < vbedge.size(); ++i)
      vfiltered.push_back(vbedge[i]);

    return vfiltered;
  }
};

class GeometryFilter_BoundaryFace : public GeometryFilter {
 protected:
  bool m_useNodes;

 public:
  GeometryFilter_BoundaryFace(bool useNodes) { this->m_useNodes = useNodes; }

  virtual ~GeometryFilter_BoundaryFace() {}

  virtual string Name() const override { return "BoundaryFaces"; }

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeom) const override {
    vector<Geometry*> vfiltered;

    PtrS<Mesh> pMesh = dynamic_pointer_cast<Mesh>(pGeom);
    if (pMesh == NULL)
      return vector<Geometry*>();

    vector<Face*> vbface;
    pMesh->GetBoundaryFaces(vbface, m_useNodes);
    vfiltered.reserve((int)vbface.size());
    for (int i = 0; i < vbface.size(); ++i)
      vfiltered.push_back(vbface[i]);

    return vfiltered;
  }
};

class GeometryFilter_BoxNodes : public GeometryFilter {
 protected:
  VectorXd m_vminCorner;
  VectorXd m_vmaxCorner;
  Tag m_sema;

 public:
  GeometryFilter_BoxNodes(const VectorXd& vminCor,
                          const VectorXd& vmaxCor,
                          Tag sema) {
    this->m_vminCorner = vminCor;
    this->m_vmaxCorner = vmaxCor;
    this->m_sema = sema;
  }

  virtual ~GeometryFilter_BoxNodes() {}

  virtual string Name() const override { return "BoxNodes"; }

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeom) const override {
    vector<Geometry*> vfiltered;

    PtrS<Mesh> pMesh = dynamic_pointer_cast<Mesh>(pGeom);
    if (pMesh == NULL)
      return vector<Geometry*>();

    vfiltered.reserve(pGeom->NumNodes());
    for (int i = 0; i < pMesh->Nodes().size(); ++i) {
      Node* pNode = pMesh->Nodes()[i];
      const Vector3d& vx = pNode->Traits().Vector3d(this->m_sema);
      if (vx.x() >= this->m_vminCorner.x() &&
          vx.x() <= this->m_vmaxCorner.x() &&
          vx.y() >= this->m_vminCorner.y() &&
          vx.y() <= this->m_vmaxCorner.y() &&
          vx.z() >= this->m_vminCorner.z() && vx.z() <= this->m_vmaxCorner.z())
        vfiltered.push_back(pNode);
    }

    return vfiltered;
  }
};

class GeometryFilter_BoxElems : public GeometryFilter {
 protected:
  VectorXd m_vminCorner;
  VectorXd m_vmaxCorner;
  Tag m_sema;

 public:
  GeometryFilter_BoxElems(const VectorXd& vminCor,
                          const VectorXd& vmaxCor,
                          Tag sema) {
    this->m_vminCorner = vminCor;
    this->m_vmaxCorner = vmaxCor;
    this->m_sema = sema;
  }

  virtual ~GeometryFilter_BoxElems() {}

  virtual string Name() const override { return "BoxElems"; }

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeom) const override {
    vector<Geometry*> vfiltered;

    throw exception("Not implemented");

    return vfiltered;
  }
};

class GeometryFilter_SphereNodes : public GeometryFilter {
 protected:
  VectorXd m_center;
  Real m_radius;
  Tag m_sema;

 public:
  GeometryFilter_SphereNodes(const VectorXd& center, Real radius, Tag sema) {
    this->m_center = center;
    this->m_radius = radius;
    this->m_sema = sema;
  }

  virtual ~GeometryFilter_SphereNodes() {}

  virtual string Name() const override { return "SphereNodes"; }

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeom) const override {
    vector<Geometry*> vfiltered;

    PtrS<Mesh> pMesh = dynamic_pointer_cast<Mesh>(pGeom);
    if (pMesh == NULL)
      return vector<Geometry*>();

    vfiltered.reserve(pGeom->NumNodes());
    for (int i = 0; i < pMesh->Nodes().size(); ++i) {
      Node* pNode = pMesh->Nodes()[i];
      const Vector3d& vx = pNode->Traits().Vector3d(this->m_sema);
      if ((vx - this->m_center).norm() <= this->m_radius)
        vfiltered.push_back(pNode);
    }

    return vfiltered;
  }
};

class GeometryFilter_SphereElems : public GeometryFilter {
 protected:
  VectorXd m_center;
  Real m_radius;
  Tag m_sema;

 public:
  GeometryFilter_SphereElems(const VectorXd& center, Real radius, Tag sema) {
    this->m_center = center;
    this->m_radius = radius;
    this->m_sema = sema;
  }

  virtual ~GeometryFilter_SphereElems() {}

  virtual string Name() const override { return "SphereElems"; }

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeom) const override {
    vector<Geometry*> vfiltered;

    throw exception("Not implemented");

    return vfiltered;
  }
};

class GeometryFilter_PlaneDistNode : public GeometryFilter {
 public:
  Tag m_space;
  Vector3d m_vp;
  Vector3d m_vn;
  Vector2d m_vrange;

  GeometryFilter_PlaneDistNode(Tag space,
                               const Vector3d& vp,
                               const Vector3d& vn,
                               const Vector2d& vrange) {
    this->m_space = space;
    this->m_vp = vp;
    this->m_vn = vn;
    this->m_vrange = vrange;
  }

  virtual ~GeometryFilter_PlaneDistNode() {}

  virtual string Name() const override { return "CollNodePlane"; }

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeom) const override {
    vector<Geometry*> vfiltered;

    PtrS<Mesh> pMesh = dynamic_pointer_cast<Mesh>(pGeom);
    if (pMesh == NULL)
      return vector<Geometry*>();

    // Select nodes inside the [range] envelope

    for (int i = 0; i < pMesh->Nodes().size(); ++i) {
      Node* pNode = pMesh->Nodes()[i];
      const Vector3d& vx = pNode->Traits().Vector3d(this->m_space);
      Real distance = m_vn.dot((m_vn * m_vn.transpose()) * (vx - m_vp));
      if (distance >= m_vrange(0) && distance <= m_vrange(1))
        vfiltered.push_back(pNode);
    }

    return vfiltered;
  }
};

class GeometryFilter_HullNodes : public GeometryFilter {
 public:
  Tag m_space;
  MatrixXd m_vX;
  MatrixXi m_vF;
  MatrixXd m_vP;

  GeometryFilter_HullNodes(Tag space, const MatrixXd& vX, const MatrixXi& vF) {
    assert(vF.cols() == 3 && "Invalid number of columns in face matrix.");
    assert(vX.cols() == 3 && "Only 3D convex hull meshes are supported.");

    this->m_space = space;
    this->m_vX = vX;
    this->m_vF = vF;

    const int nFaces = m_vF.rows();
    const int nDims = m_vX.cols();

    // ENHANCEME: Sanitiy check to test the convexity of the given input mesh.
    // Two options:
    // A) Test all vertices and check if they lie behind the input plane (pretty
    //    much like the filter algorithm we implemented).
    //    Cost: O(X·F).
    // B) Build the convex hull of the mesh and see if any vertices were
    // dropped.
    //    Cost(X·log(X)).
    // Probably not worth implementing any of these though =)

    // Build per-face planes used to test points if the points are within the
    // convex hull.
    this->m_vP.resize(nFaces, 4);

    for (int i = 0; i < m_vF.rows(); ++i) {
      Vector3i f = m_vF.row(i).transpose();
      Vector3d a = m_vX.row(f[0]).transpose();
      Vector3d ba = m_vX.row(f[1]).transpose() - a;
      Vector3d ca = m_vX.row(f[2]).transpose() - a;
      Vector3d n = ba.cross(ca).normalized();

      this->m_vP.block(i, 0, 1, 3) = n.transpose();
      this->m_vP(i, 3) = -a.dot(n);
    }
  }

  virtual ~GeometryFilter_HullNodes() = default;

  virtual string Name() const override { return "ConvexHullNode"; }

  virtual vector<Geometry*> Filter(PtrS<Geometry> pGeom) const override {
    vector<Geometry*> vFiltered;
    // vector<vector<Geometry*>> vPerThreadFiltered(omp_get_max_threads());

    PtrS<Mesh> pMesh = dynamic_pointer_cast<Mesh>(pGeom);
    if (pMesh == nullptr)
      return vFiltered;

    vFiltered.reserve(pMesh->NumNodes());

    // A point is contained within a convex hull if it lies behind of all the
    // planes defined by the hull surface [i.e. inside = all(x^T N + d <= 0)].
    // This algorithm is pretty simple, and enables very easy parallelization,
    // although its wost case scenario could be pretty nasty [O(NM)]. However,
    // it allows an early bail-out, so the actual run-time cost will greatly
    // depend on both the complexity and coverage of the hull.

    // Filter all node points in parallel. To prevent thread synchronization
    // shenanigans, build filtered list on a per-thread basis and concatenate
    // at the end.
    //#pragma omp parallel
    {
      // vector<Geometry*>& vThreadFiltered =
      // vPerThreadFiltered[omp_get_thread_num()];

      //#pragma omp for
      for (int i = 0; i < pMesh->Nodes().size(); ++i) {
        Node* pNode = pMesh->Nodes()[i];
        const Vector3d& vx = pNode->Traits().Vector3d(this->m_space);
        bool insideHull = true;

        for (int j = 0; insideHull && j < m_vP.rows(); ++j) {
          Vector4d p = m_vP.row(j).transpose();
          insideHull = (vx.dot(p.segment(0, 3)) + p(3)) <= 0.0;
        }

        if (insideHull == true)
          vFiltered.push_back(pNode);
      }
    }

    // for (auto& vThreadFiltered : vPerThreadFiltered)
    //    vFiltered.insert(vFiltered.end(), vThreadFiltered.begin(),
    //    vThreadFiltered.end());

    // Done! Let's hope that wasn't too long =)
    return vFiltered;
  }
};
}  // namespace PhySim