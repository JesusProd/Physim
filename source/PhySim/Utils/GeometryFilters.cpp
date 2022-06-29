//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Utils/GeometryFilters.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

vector<Geometry*> GeometryFilter::Filter(
    vector<PtrS<Geometry>> vpGeometry) const {
  vector<Geometry*> vpResult;

  for (int i = 0; i < (int)vpGeometry.size(); ++i) {
    vector<Geometry*> vpResult_i = Filter(vpGeometry[i]);
    vpResult.reserve(vpResult.size() + vpResult_i.size());
    vpResult.insert(vpResult.end(), vpResult_i.begin(), vpResult_i.end());
  }

  return vpResult;
}

vector<Geometry*> GeometryFilter_AllNode::Filter(PtrS<Geometry> pGeom) const {
  vector<Geometry*> vfiltered;

  vfiltered.reserve(pGeom->NumNodes());
  for (int i = 0; i < pGeom->NumNodes(); ++i)
    vfiltered.push_back(pGeom->Nodes()[i]);

  return vfiltered;
}

vector<Geometry*> GeometryFilter_AllElem::Filter(PtrS<Geometry> pGeom) const {
  vector<Geometry*> vfiltered;

  PtrS<Mesh> pMesh = dynamic_pointer_cast<Mesh>(pGeom);
  if (pMesh == NULL)
    return vector<Geometry*>();

  vfiltered.reserve(pMesh->NumElems());
  for (int i = 0; i < pMesh->NumElems(); ++i)
    vfiltered.push_back(pMesh->Elems()[i]);

  return vfiltered;
}

// const string GeometryFilter_AllNode::NAME = "AllNodes";
// const string GeometryFilter_AllElem::NAME = "AllElems";
// const string GeometryFilter_IndexNode::NAME = "IndexNodes";
// const string GeometryFilter_IndexElem::NAME = "IndexElems";
// const string GeometryFilter_BoundaryNode::NAME = "BoundaryNode";
// const string GeometryFilter_BoundaryEdge::NAME = "BoundaryEdge";
// const string GeometryFilter_BoundaryFace::NAME = "BoundaryFace";
// const string GeometryFilter_BoxDoF::NAME = "Box";
// const string GeometryFilter_PlaneDistNode::NAME = "PlaneDistNode";
}  // namespace PhySim