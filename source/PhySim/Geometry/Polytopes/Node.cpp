//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Geometry/Polytopes/ShapeFunctions.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Node::Node(int ID, const Vector3d& vpos, const vector<Tag>& vtraits)
    : Poly(ID) {
  // Define the shape function of a node. This is just
  // a dummy shape function that returns the value of
  // the node. There is no interpolation happening.

  this->m_pSFunction = ShapeFunction_Node::Instance();

  // If there are not define traits, the position of
  // the node is stored in the Tag_Position_0 data layer.

  if (vtraits.empty()) {
    m_traits.AddTrait(Tag_Position_0, vpos);
  } else {
    for (int i = 0; i < (int)vtraits.size(); ++i)
      m_traits.AddTrait(vtraits[i], vpos);
  }

  // As a polytope, a node contains itself

  this->m_vnodes.push_back(this);

  // By default, it doesn't belong to a mesh

  this->m_pMesh = NULL;
}

Node::~Node(void) {
  // Nothing to do here...
}

}  // namespace PhySim