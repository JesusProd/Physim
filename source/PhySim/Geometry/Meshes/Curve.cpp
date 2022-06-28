//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Meshes/Curve.h>

#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Node.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	Curve::Curve()
	{
		// Nothing to do here...
	}

	Curve::Curve(const MatrixXd& mV, const vector<Tag>& vs)
	{
		this->Init(mV, vs);
	}

	void Curve::Init(const MatrixXd& mV, const vector<Tag>& vs)
	{
		int numV = (int) mV.rows();
		MatrixXi mE(numV - 1, 2);
		for (int i = 0; i < numV - 1; ++i)
			mE.row(i) = Vector2i(i, i + 1);

		Mesh_Edge::Init(mV, mE, Discretization_Edge2, vs);
	}

	Curve::~Curve()
	{
		this->FreeInternal();

#ifndef NDEBUG
		logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Curve");
#endif 
	}

}