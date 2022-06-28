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


#include <PhySim/Kinematics/KinematicsMap.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class KMapLinear : public KinematicsMap
	{
	public:

		KMapLinear() { }
		KMapLinear(Simulable* pModel, const vector<PtrS<KinematicsEle>>& vpEleIn, const vector<PtrS<KinematicsEle>>& vpEleOut);
		virtual void Init(Simulable* pModel, const vector<PtrS<KinematicsEle>>& vpEleIn, const vector<PtrS<KinematicsEle>>& vpEleOut);
		virtual void MapValue(int idxIn, const VectorXd& vpIn, int idxOut, VectorXd& vpOut) override;
		virtual void UpdateMapPartials() override = 0;

	};
}
