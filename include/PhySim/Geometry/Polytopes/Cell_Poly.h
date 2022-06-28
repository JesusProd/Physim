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


#include <PhySim/Geometry/Polytopes/Cell.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Cell_Poly : public Cell
	{

	public:
		Cell_Poly(int ID, const vector<Node*>& vnodes);

		virtual ~Cell_Poly(void);

		virtual Real VolumeSpace(Tag s) const override;

	};
}