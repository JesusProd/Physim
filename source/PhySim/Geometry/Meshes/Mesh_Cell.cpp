//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Meshes/Mesh_Cell.h>

#include <PhySim/Geometry/Polytopes/Cell_Tetra.h>
#include <PhySim/Geometry/Polytopes/Cell_Hexa.h>
#include <PhySim/Geometry/Polytopes/Node.h>
#include <PhySim/Geometry/Polytopes/Edge.h>
#include <PhySim/Geometry/Polytopes/Face.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	Mesh_Cell::Mesh_Cell()
	{
		// Noting to do here...
	}

	Mesh_Cell::Mesh_Cell(const Mesh_Cell& toCopy) : Mesh(toCopy)
	{
		this->UpdateMetadata();
	}

	Mesh_Cell::Mesh_Cell(const MatrixXd& mV, const MatrixXi& mF, Discretization D, const vector<Tag>& vnTraits) : Mesh(mV, mF, D, vnTraits)
 	{
		this->UpdateMetadata();
	}

	void Mesh_Cell::Init(const MatrixXd& mV, const MatrixXi& mF, Discretization D, const vector<Tag>& vnTraits)
	{
		Mesh::Init(mV, mF, D, vnTraits);

		this->UpdateMetadata();
	}

	Mesh_Cell::~Mesh_Cell(void)
	{
		this->FreeMetadata();

#ifndef NDEBUG
		logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Mesh_Cell");
#endif
	}

	void Mesh_Cell::FreeMetadata()
	{
		this->m_vcells.clear();
	}

	void Mesh_Cell::UpdateMetadata()
	{
		// Initialize mesh cells vector

		this->m_vfaces.reserve(this->NumElems());
		for (int i = 0; i < this->m_velems.size(); ++i)
			this->m_vcells.push_back(static_cast<Cell*>(m_velems[i]));
	}
}