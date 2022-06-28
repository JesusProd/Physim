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
#include <PhySim/Kinematics/KEleParticle3D.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Embedding;

	class KMapEmbedding : public KinematicsMap
	{
	protected:
		PtrS<Embedding> m_pEmbedding;

	public:
		KMapEmbedding() { }
		KMapEmbedding(Simulable* pModel, PtrS<KEleParticle3D>& pParticle, PtrS<Embedding>& pEmbedding);
		virtual void Init(Simulable* pModel, PtrS<KEleParticle3D>& pParticle, PtrS<Embedding>& pEmbedding);
		virtual void MapValue(int idxIn, const VectorXd& vpIn, int idxOut, VectorXd& vpOut) override;
		virtual void UpdateMapPartials() override;
	};

}
