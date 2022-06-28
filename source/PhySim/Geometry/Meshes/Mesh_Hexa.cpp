//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Geometry/Meshes/Mesh_Hexa.h>

#include <PhySim/Geometry/Polytopes/Cell_Hexa.h>
#include <PhySim/Geometry/Polytopes/Node.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	Mesh_Hexa::Mesh_Hexa() : Mesh_Cell()
	{
		this->UpdateMetadata();
	}

	Mesh_Hexa::Mesh_Hexa(const Mesh_Hexa& toCopy) : Mesh_Cell(toCopy)
	{
		this->UpdateMetadata();
	}

	Mesh_Hexa::Mesh_Hexa(const MatrixXd& mV, const MatrixXi& mC, Discretization D, const vector<Tag>& vnTraits) : Mesh_Cell(mV, mC, D, vnTraits)
	{
		this->UpdateMetadata();
	}

	void Mesh_Hexa::Init(const MatrixXd& mV, const MatrixXi& mC, Discretization D, const vector<Tag>& vnTraits)
	{
		Mesh::Init(mV, mC, D, vnTraits);
	}

	Mesh_Hexa::~Mesh_Hexa(void)
	{
		this->FreeMetadata();

#ifndef NDEBUG
		logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Mesh_Hexa");
#endif
	}

	void Mesh_Hexa::FreeMetadata()
	{
		// Not implemented
	}

	void Mesh_Hexa::UpdateMetadata()
	{
		this->FreeMetadata();
	}

}