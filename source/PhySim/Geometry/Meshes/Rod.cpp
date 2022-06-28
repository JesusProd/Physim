//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Meshes/Rod.h>

#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Node.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	Rod::Rod()
	{
		// Nothing to do here...
	}

	Rod::Rod(const MatrixXd& mV,
			 const vector<Frame3d>& vF,
			 const vector<Tag>& vnTraits,
			 const vector<Tag>& vfTraits)
	{
		this->Init(mV, vF, vnTraits, vfTraits);
	}

	void Rod::Init(const MatrixXd& mV,
				   const vector<Frame3d>& vF,
				   const vector<Tag>& vnTraits,
				   const vector<Tag>& vfTraits)
	{
		int numV = (int)mV.rows();
		MatrixXi mE(numV - 1, 2);
		for (int i = 0; i < numV - 1; ++i)
			mE.row(i) = Vector2i(i, i + 1);

		Mesh_Frame::Init(mV, mE, vF, vnTraits, vfTraits);
	}

	Edge* Rod::HeadEdge()
	{
		return this->Edges().back();
	}

	Edge* Rod::TailEdge()
	{
		return this->Edges().front();
	}

	Node* Rod::HeadNode()
	{
		return this->Nodes().back();
	}

	Node* Rod::TailNode()
	{
		return this->Nodes().front();
	}

	Rod::~Rod()
	{
#ifndef NDEBUG
		logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Rod");
#endif 
	}

}