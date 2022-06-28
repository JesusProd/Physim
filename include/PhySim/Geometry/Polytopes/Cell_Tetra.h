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


#include <PhySim/Geometry/Geometry.h>
#include <PhySim/Geometry/Polytopes/Cell.h>
#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/ShapeFunctions.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Cell_Tetra : public Cell
	{

	public:
		Cell_Tetra(int ID, const vector<Node*>& vnodes);

		virtual ~Cell_Tetra(void);

		virtual Real VolumeSpace(Tag s) const override;

		virtual bool IsValidParametric(const VectorXd& vp) const override;
		virtual void TransformNat2Iso(VectorXd& vb, MatrixXd& mA, Tag s) const override;
		virtual void TransformIso2Nat(VectorXd& vb, MatrixXd& mA, Tag s) const override;

	};

	class Cell_Tetra4 : public Cell_Tetra
	{

	public:
		Cell_Tetra4(int ID, const vector<Node*>& vnodes) : Cell_Tetra(ID, vnodes) 
		{
			this->m_pSFunction = ShapeFunction_Tet4::Instance();
		}

		virtual void InitSubelementPositions(Tag s) override { }

	};

	class Cell_Tetra10 : public Cell_Tetra
	{
	public:
		Cell_Tetra10(int ID, const vector<Node*>& vnodes) : Cell_Tetra(ID, vnodes)
		{
			this->m_pSFunction = ShapeFunction_Tet10::Instance();
		}

		virtual void InitSubelementPositions(Tag s) override
		{
			MatrixXd mN;
			this->GetNodesTrait(mN, s);
			this->m_vnodes[4]->Traits().Vector3d(s) = 0.5*(mN.row(0) + mN.row(1));
			this->m_vnodes[5]->Traits().Vector3d(s) = 0.5*(mN.row(1) + mN.row(2));
			this->m_vnodes[6]->Traits().Vector3d(s) = 0.5*(mN.row(0) + mN.row(2));
			this->m_vnodes[7]->Traits().Vector3d(s) = 0.5*(mN.row(0) + mN.row(3));
			this->m_vnodes[8]->Traits().Vector3d(s) = 0.5*(mN.row(1) + mN.row(3));
			this->m_vnodes[9]->Traits().Vector3d(s) = 0.5*(mN.row(2) + mN.row(3));
		}

	};
}