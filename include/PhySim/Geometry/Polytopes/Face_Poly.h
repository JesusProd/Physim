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


#include <PhySim/Geometry/Polytopes/Face.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Face_Poly : public Face
	{

	public:
		Face_Poly(int ID, const vector<Node*>& vnodes);
		virtual ~Face_Poly(void);

		virtual Real VolumeSpace(Tag s) const override;

	};
}