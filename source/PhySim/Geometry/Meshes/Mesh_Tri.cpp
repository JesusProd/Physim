//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Meshes/Mesh_Tri.h>
#include <PhySim/Geometry/Meshes/Curve.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Cell_Tetra.h>

#include <PhySim/Utils/GeometryUtils.h>

#include <igl/boundary_loop.h>
#include <igl/boundary_facets.h>
#include <igl/biharmonic_coordinates.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	Mesh_Tri::Mesh_Tri() : Mesh_Face()
	{
		this->UpdateMetadata();
	}

	Mesh_Tri::Mesh_Tri(const Mesh_Tri& toCopy) : Mesh_Face(toCopy)
	{
		this->UpdateMetadata();
	}

	Mesh_Tri::Mesh_Tri(const MatrixXd& mV, const MatrixXi& mF, Discretization D, const vector<Tag>& vnTraits) : Mesh_Face(mV, mF, D, vnTraits)
	{
		this->UpdateMetadata();
	}

	void Mesh_Tri::Init(const MatrixXd& mV, const MatrixXi& mF, Discretization D, const vector<Tag>& vnTraits)
	{
		Mesh_Face::Init(mV, mF, D, vnTraits);

		this->UpdateMetadata();
	}

	Mesh_Tri::~Mesh_Tri(void)
	{
		this->FreeInternal();
	
#ifndef NDEBUG
		logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Mesh_Tri");
#endif
	}

	void Mesh_Tri::GetBoundaryLoop(vector<Node*>& vbnode)
	{
		MatrixXi mF;
		VectorXi vb;
		this->GetElemMatrix(mF);
		igl::boundary_loop(mF, vb);

		vbnode.resize((int) vb.size());
		for (int i = 0; i < vb.size(); ++i)
			vbnode[i] = this->Nodes()[vb[i]];
	}

	void Mesh_Tri::GetBoundaryCurves(vector<vector<Node*>>& vC, Real angleTol, Tag s)
	{
		vector<Node*> vbloop;
		this->GetBoundaryLoop(vbloop);
		iVector vcorner;

		for (int i = 0; i < (int) vbloop.size(); ++i)
		{
			int prev = (i == 0)? (int)vbloop.size() - 1 : i - 1;
			int next = (i == (int)vbloop.size() - 1)? 0 : i + 1;
			Vector3d x1 = vbloop[i]->Traits().Vector3d(s);
			Vector3d x0 = vbloop[prev]->Traits().Vector3d(s);
			Vector3d x2 = vbloop[next]->Traits().Vector3d(s);
			Vector3d vt0 = (x1 - x0).normalized();
			Vector3d vt1 = (x2 - x1).normalized();
			if (acos(vt0.dot(vt1)) > angleTol)
				vcorner.push_back(i); // Add!
		}

		// No boundary: out
		if (vbloop.empty())
		{
			return;
		}

		// No corner: out
		if (vcorner.empty())
		{
			vC.push_back(vbloop);
			return; // Just one!
		}

		// Build curves
		
		int numAdded = 0;
		int k = vcorner.front();
		vC = vector<vector<Node*>>();

		vector<Node*>* vpCurCurve = NULL;
		while (numAdded != vbloop.size())
		{
			if (find(vcorner.begin(), vcorner.end(), k) != vcorner.end())
			{
				if (vpCurCurve != NULL) 
					vpCurCurve->push_back(vbloop[k]);

				if (vC.size() == vcorner.size())
				{
					vpCurCurve = &vC.front();
				}
				else
				{
					vC.push_back(vector<Node*>());
					vpCurCurve = &vC.back();
				}
			}

			vpCurCurve->push_back(vbloop[k]);
			
			k = (k + 1) % vbloop.size();
			numAdded = numAdded + 1;
		}

		vC.back().push_back(vC.front().front());
	}

	void Mesh_Tri::GetBoundaryNodes(vector<Node*>& vbnode)
	{
		this->GetBoundaryLoop(vbnode);
	}

	void Mesh_Tri::FreeMetadata()
	{
		// Nothing to do here...
	}

	void Mesh_Tri::UpdateMetadata()
	{
		this->FreeMetadata();

		// Nothing to do here...
	}

	void Mesh_Tri::SkinningBiharmonic(const VectorXi& vhandles, MatrixXd& mweights, Tag s)
	{
		MatrixXd mV;
		MatrixXi mE;
		this->GetElemMatrix(mE);
		this->GetNodesTrait(mV, s);
		vector<vector<int>> lhandles;
		igl::matrix_to_list(vhandles, lhandles);
		igl::biharmonic_coordinates(mV, mE, lhandles, mweights);
	}

	void Mesh_Tri::SkinningLaplacian(const VectorXi& vhandles, MatrixXd& mweights, Tag s)
	{
		// TODO
	}

	Real Mesh_Tri::VolumeSpace(Tag s) const
	{
		if (this->DimSpace() == 2)
		{
			return this->VolumeBasis(s);
		}

		if (this->DimSpace() == 3)
		{
			// For each quad, compute the volume of the two tetrahedra
			// associated with the two triangles composing the face

			Real volume = 0;

			for (int i = 0; i < (int)this->m_vfaces.size(); ++i)
			{
				Node* pNode0 = this->m_vfaces[i]->Nodes()[0];
				Node* pNode1 = this->m_vfaces[i]->Nodes()[1];
				Node* pNode2 = this->m_vfaces[i]->Nodes()[2];

				Matrix3d mV;
				mV.col(0) = pNode0->Traits().Vector3d(s);
				mV.col(1) = pNode1->Traits().Vector3d(s);
				mV.col(2) = pNode2->Traits().Vector3d(s);

				volume += (1.0 / 6.0)*mV.determinant();
			}

			return volume;
		}

		throw PhySim::exception("Unreachable section");
	}

	void Mesh_Tri::MassProperties(Tag s, Real rho, Real& mass, Vector3d& vcom, Matrix3d& mI) const
	{
		int numE = this->NumElems();

		Node ref(-1, Vector3d::Zero(), this->NodeTraits());

		////Matrix3d mC0 = Matrix3d::Ones()*(1.0/120.0);
		////mC0(0,0) = mC0(1,1) = mC0(2,2) = (1.0/60.0);

		mass = 0;
		vcom = Vector3d::Zero();
		//mI = Matrix3d::Zero();

		// Elements

		for (int i = 0; i < numE; ++i)
		{
			vector<Node*> vpos(4);
			vpos[0] = &ref;
			vpos[1] = this->m_velems[i]->Nodes()[0];
			vpos[2] = this->m_velems[i]->Nodes()[1];
			vpos[3] = this->m_velems[i]->Nodes()[2];
			Cell_Tetra tetra(-1, vpos);

			Vector3d x0 = vpos[0]->Trait<Vector3d>(Tag::Tag_Position_0);
			Vector3d x1 = vpos[1]->Trait<Vector3d>(Tag::Tag_Position_0);
			Vector3d x2 = vpos[2]->Trait<Vector3d>(Tag::Tag_Position_0);
			Vector3d x3 = vpos[3]->Trait<Vector3d>(Tag::Tag_Position_0);

			// Mass

			double m;

			mass += m = tetra.VolumeBasis(s) * rho;

			// COM

			vcom += tetra.Centroid(s) * m;
		}

		// Approximate moment-of-inertia with OOBB

		MatrixXd mV;
		this->GetNodesTrait(mV, s);

		MatrixXd mR;
		VectorXd vt;
		GeometryUtils::computeBestAxisAligment(mV, mR, vt);

		// Align mesh with axis

		MatrixXd mVAligned = mR * mV.transpose();

		// Compute axis aligned moment

		VectorXd vmin = mVAligned.rowwise().minCoeff();
		VectorXd vmax = mVAligned.rowwise().maxCoeff();
		VectorXd vran = vmax - vmin;
		Matrix3d mI0 = Matrix3d::Identity()*(mass/12);
		mI0(0, 0) *= (vran(1) * vran(1) + vran(2) * vran(2));
		mI0(1, 1) *= (vran(0) * vran(0) + vran(2) * vran(2));
		mI0(2, 2) *= (vran(1) * vran(1) + vran(0) * vran(0));

		// Rotate to original orientation

		mI = mR.transpose() * mI0 * mR;
	}

}