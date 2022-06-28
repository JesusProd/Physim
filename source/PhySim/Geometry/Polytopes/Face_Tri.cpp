//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Face_Tri.h>

#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Node.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	Face_Tri::Face_Tri(int id, const vector<Node*>& m_vnodes) : Face(id, m_vnodes)
	{
		m_vfaces.push_back(this);
	}

	Face_Tri::~Face_Tri(void) { }

	Vector3d Face_Tri::Normal(Tag s) const
	{
		Vector3d e0 = (m_vnodes[1]->Traits().Vector3d(s) - m_vnodes[0]->Traits().Vector3d(s));
		Vector3d e1 = (m_vnodes[2]->Traits().Vector3d(s) - m_vnodes[0]->Traits().Vector3d(s));
		return e0.cross(e1).normalized();
	}

	Real Face_Tri::VolumeSpace(Tag s) const
	{
		Vector3d e0 = (m_vnodes[1]->Traits().Vector3d(s) - m_vnodes[0]->Traits().Vector3d(s));
		Vector3d e1 = (m_vnodes[2]->Traits().Vector3d(s) - m_vnodes[0]->Traits().Vector3d(s));
		return 0.5*e0.cross(e1).norm();
	}

	void Face_Tri::TransformNat2Iso(VectorXd& b, MatrixXd& A, Tag s) const
	{
		MatrixXd mNB;
		mNB.resize(3, 3);
		Vector3d e0 = this->m_vnodes[1]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		Vector3d e1 = this->m_vnodes[2]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		mNB.col(0) = e0;
		mNB.col(1) = e1;
		mNB.col(2) = e0.cross(e1);
		A = mNB.inverse().block(0, 0, 2, 3);
		b = -A*m_vnodes[0]->Traits().Vector3d(s);
	}

	void Face_Tri::TransformIso2Nat(VectorXd& b, MatrixXd& A, Tag s) const
	{
		A.resize(3, 2);
		A.col(0) = this->m_vnodes[1]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		A.col(1) = this->m_vnodes[2]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		b = this->m_vnodes[0]->Traits().Vector3d(s);
	}

	void Face_Tri::TransformNat2Bas(VectorXd& b, MatrixXd& A, Tag s) const
	{
		Vector3d e0 = this->m_vnodes[1]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		Vector3d e1 = this->m_vnodes[2]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		Vector3d e2 = e0.cross(e1);
		e1 = e2.cross(e0);
		e0.normalize();
		e1.normalize();
		e2.normalize();
		Matrix3d T;
		T.col(0) = e0;
		T.col(1) = e1;
		T.col(2) = e2;

		assert((T*T.transpose() - Matrix3d::Identity()).norm() < 1e-6);


		A = T.inverse().block(0, 0, 2, 3);
		b = -(A*Centroid(s));
	}

	void Face_Tri::TransformBas2Nat(VectorXd& b, MatrixXd& A, Tag s) const
	{
		Vector3d e0 = this->m_vnodes[1]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		Vector3d e1 = this->m_vnodes[2]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		Vector3d e2 = e0.cross(e1);
		e1 = e2.cross(e0);
		e0.normalize();
		e1.normalize();
		e2.normalize();
		Matrix3d T;
		T.col(0) = e0;
		T.col(1) = e1;
		T.col(2) = e2;
		A = T;
		b = Centroid(s);
	}

}