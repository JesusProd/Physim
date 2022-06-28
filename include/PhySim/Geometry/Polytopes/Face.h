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

	class Face : public Poly
	{

	public:
		Face(int ID, const vector<Node*>& vnodes);
		~Face(void);

		inline int DimBasis() const override { return 2; }

		virtual Real Area(Tag s) const { return this->VolumeSpace(s); }

		virtual Vector3d Normal(Tag s) const { throw new exception("Not implemented"); }

		/**
		* Computes the rigid transformation from natural coordinates
		* to the local tangential coordinates of this face, assuming
		* the face is planar.
		*/
		virtual void TransformNat2Bas(VectorXd& b, MatrixXd& A, Tag s) const;

		/**
		* Computes the rigid transformation from the local tangential
		* coordinates of this face, to the natural coordinates assuming
		* the face is planar.
		*/
		virtual void TransformBas2Nat(VectorXd& b, MatrixXd& A, Tag s) const;

	};
}
