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


#include <PhySim/Geometry/Meshes/Mesh_Cell.h>
#include <PhySim/Geometry/Polytopes/Cell_Hexa.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Mesh_Hexa : public Mesh_Cell
	{

	protected:

	public:
		Mesh_Hexa();
		Mesh_Hexa(const Mesh_Hexa& toCopy);
		Mesh_Hexa(const MatrixXd& mV, const MatrixXi& mC,
				  Discretization D = Discretization_Hex8,
				  const vector<Tag>& vs = vector<Tag>());

		void Init(const MatrixXd& mV, const MatrixXi& mF,
				  Discretization D = Discretization_Hex8,
				  const vector<Tag>& vs = vector<Tag>());

		virtual ~Mesh_Hexa(void);
		
		virtual Mesh* Clone() const override { return new Mesh_Hexa(*this); }

		virtual Cell_Hexa* GetHexa(int i) { return static_cast<Cell_Hexa*>(this->m_vcells[i]); }

	private:
		void UpdateMetadata();
		void FreeMetadata();

	};

}
