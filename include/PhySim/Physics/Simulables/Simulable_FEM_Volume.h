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


#include <PhySim/Physics/Simulables/Simulable_FEM.h>

#include <PhySim/Geometry/Meshes/Mesh_Cell.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	/**
	* Simulable_FEM_Volume
	*
	* TODO.
	*/
	class Simulable_FEM_Volume : public Simulable_FEM
	{
	public:
		virtual string GetName() const override { return "[FEM Volume]"; }

		virtual Mesh_Cell* MeshCell() { return static_cast<Mesh_Cell*>(this->m_pMesh.get()); }
	};
}
