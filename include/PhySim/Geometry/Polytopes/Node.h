//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC Madrid
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>


#include <PhySim/Geometry/Polytopes/Poly.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	/**
	* TODO
	**/
	class Node : public Poly
	{
	public:
		Node(int ID, const Vector3d& vpos = Vector3d::Zero(), const vector<Tag>& vtraits = vector<Tag>());
		virtual ~Node(void);

		inline int DimBasis() const override { return 0; }

		virtual Real VolumeSpace(Tag s) const override { return 0; }

		VectorXd InterpolateValue(const VectorXd& vp, Tag s) const override
		{
			return this->Traits().Vector3d(s);
		}

	};

}
