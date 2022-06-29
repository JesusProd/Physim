//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>

#include <PhySim/Geometry/Meshes/Mesh_Frame.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class Rod : public Mesh_Frame {
 public:
  Rod();

  Rod(const MatrixXd& mV,
      const vector<Frame3d>& vF = vector<Frame3d>(),
      const vector<Tag>& vnodeTraits = vector<Tag>(),
      const vector<Tag>& vframeTratis = vector<Tag>());

  void Init(const MatrixXd& mV,
            const vector<Frame3d>& vF = vector<Frame3d>(),
            const vector<Tag>& vnodeTraits = vector<Tag>(),
            const vector<Tag>& vframeTratis = vector<Tag>());

  virtual ~Rod(void);

  virtual Edge* HeadEdge();
  virtual Edge* TailEdge();
  virtual Node* HeadNode();
  virtual Node* TailNode();
};

}  // namespace PhySim
