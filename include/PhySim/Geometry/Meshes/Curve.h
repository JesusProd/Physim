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


#include <PhySim/Geometry/Meshes/Mesh_Edge.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Curve : public Mesh_Edge
	{

	public:
		Curve();

		Curve(const MatrixXd& mV, const vector<Tag>& vs = vector<Tag>());

		void Init(const MatrixXd& mV, const vector<Tag>& vs = vector<Tag>());

		virtual ~Curve(void);

	};

}

