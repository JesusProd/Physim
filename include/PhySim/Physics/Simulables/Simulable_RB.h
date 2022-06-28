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

#include <PhySim/Physics/Simulables/Simulable.h>
#include <PhySim/Geometry/Meshes/Mesh.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	/**
	* Simulable_RB.
	*
	* TODO.
	*/
	class Simulable_RB : public Simulable
	{
	public:

		/**
		* TODO.
		*/
		struct Options
		{
			/**
			* The input meshes.
			*/
			vector<PtrS<Geometry>> m_vpGeometries;

			vector<PtrS<ParameterSet>> m_vpMaterials;

			MatrixXi m_couplings;

			vector<MatrixXd> m_params;

			std::string m_patientNumber;
			bool m_shoulderJoint;

			Options()
			{
				m_vpGeometries.clear();
			}

			virtual ~Options()
			{
				// Nothing to do here...
			}
		};

	protected:

		Options* m_pOptions;

	public:

		virtual string GetName() const override { return "RB System"; }

		virtual Simulable_RB::Options& SetupOptions();

		/**
		* Constructor.
		*/
		Simulable_RB();

		/**
		* Denstructor.
		*/
		virtual ~Simulable_RB();

		virtual void SetState(const VectorXd& vs) override;
		virtual void SetDOFVector(const VectorXd& vx, Tag s) override;

		virtual vector<PtrS<Geometry>> Geometries() const override { return this->m_pOptions->m_vpGeometries; }
		//virtual PtrS<Geometry> GetGeometry(int i) const { return this->m_pOptions->m_vpGeometries[i]; }
		//virtual PtrS<ParameterSet> Materials(int i) const { return this->m_pOptions->m_vpMaterials[i]; }

		//virtual KEleRigidBody3D* RigidBody(int i) { return static_cast<KEleRigidBody3D*>(this->m_pOptions->m_vpGeometries[i]->Traits().Kinematics(Tag_DOF_0)); }

	protected:

		virtual void InitInternal() override;
		virtual void FreeInternal() override;

	};
}