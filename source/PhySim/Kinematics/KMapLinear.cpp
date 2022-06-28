//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, URJC MAdrid
//
//==========================================================

#include <PhySim/Kinematics/KMapLinear.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	KMapLinear::KMapLinear(Simulable* pModel, const vector<PtrS<KinematicsEle>>& vIn, const vector<PtrS<KinematicsEle>>& vOut)
	{
		this->Init(pModel, vIn, vOut);
	}

	void KMapLinear::Init(Simulable* pModel, const vector<PtrS<KinematicsEle>>& vIn, const vector<PtrS<KinematicsEle>>& vOut)
	{
		KinematicsMap::Init(pModel, vIn, vOut);
	}

	void KMapLinear::MapValue(int idxIn, const VectorXd& vpIn, int idxOut, VectorXd& vpOut)
	{
		if (this->m_isDirty)
			this->UpdateMapPartials();

		// For each input to this kinematics map we
		// map the value to the corresponding output

		VectorXd vpIn_i = vpIn.segment(
			this->m_vIn[idxIn]->m_offset,
			this->m_vIn[idxIn]->m_numDim);
		vpOut = this->m_vDoutDin[idxOut][idxIn] * vpIn_i;
	}
}

