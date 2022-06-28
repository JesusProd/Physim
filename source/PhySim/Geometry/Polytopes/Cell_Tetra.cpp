//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Polytopes/Cell_Tetra.h>

#include <PhySim/Geometry/Polytopes/Node.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	Cell_Tetra::Cell_Tetra(int id, const vector<Node*>& m_vnodes) : Cell(id, m_vnodes)
	{
		// Nothing to do here...
	}

	Cell_Tetra::~Cell_Tetra(void)
	{
		// Nothing to do here...
	}

	Real Cell_Tetra::VolumeSpace(Tag s) const
	{
		Matrix3d mV;
		mV.col(0) = this->m_vnodes[1]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		mV.col(1) = this->m_vnodes[2]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		mV.col(2) = this->m_vnodes[3]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		return (1.0/6.0)*mV.determinant();
	}

	bool Cell_Tetra::IsValidParametric(const VectorXd& vp) const
	{
		if (vp.size() != 3)
			return false;

		Real weightSum = 0;
		bool valid = true;

		for (int i = 0; i < vp.size(); ++i)
		{
			if (vp[i] < 0 - 1e-6 || vp[i] > 1 + 1e-6)
			{
				valid = false;
				break; // Found
			}

			weightSum += vp[i];
		}

		if (weightSum > 1 + 1e-6)
			valid = false;

		return valid;
	}

	void Cell_Tetra::TransformNat2Iso(VectorXd& b, MatrixXd& A, Tag s) const
	{
		MatrixXd mNB;
		mNB.resize(3, 3);
		mNB.col(0) = this->m_vnodes[1]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		mNB.col(1) = this->m_vnodes[2]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		mNB.col(2) = this->m_vnodes[3]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		A = mNB.inverse();
		b = -A*this->m_vnodes[0]->Traits().Vector3d(s);
	}

	void Cell_Tetra::TransformIso2Nat(VectorXd& b, MatrixXd& A, Tag s) const
	{
		A.resize(3, 3);
		A.col(0) = this->m_vnodes[1]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		A.col(1) = this->m_vnodes[2]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		A.col(2) = this->m_vnodes[3]->Traits().Vector3d(s) - this->m_vnodes[0]->Traits().Vector3d(s);
		b = this->m_vnodes[0]->Traits().Vector3d(s);
	}

}