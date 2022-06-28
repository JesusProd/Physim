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


#include <PhySim/Physics/Simulables/Simulable_Mesh.h>

#include <PhySim/Geometry/Meshes/Mesh_DER.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class EnergyElement_DER_Stretch;
	class EnergyElement_DER_BendTwist;
	class EnergyElement_DER_Connection;

	/**
	* Simulable_DER.
	*
	* TODO.
	*/
	class Simulable_DER : public Simulable_Mesh
	{
	public:

		struct Options : public Simulable_Mesh::Options
		{
			Options()
			{
				// Nothing to do here...
			}
		};


	protected:

		Mesh_DER* m_pMeshDER;

		vector<EnergyElement_DER_Stretch*> m_venerEle_stretch;
		vector<EnergyElement_DER_BendTwist*> m_venerEle_bendTwist;
		vector<EnergyElement_DER_Connection*> m_venerEle_connection;

	public:

		virtual string GetName() const override { return "[DER System]"; }

		virtual Options& SetupOptions() override;

		virtual PtrS<IDomainDistribution<PtrS<ParameterSet>>>& MaterialDistribution() { return this->SetupOptions().m_pMatParams; }

		virtual Mesh_DER& GetMesh() { return *this->m_pMeshDER; }

		/**
		* Constructor.
		*/
		Simulable_DER();

		/**
		* Destructor.
		*/
		virtual ~Simulable_DER();

		virtual const vector<EnergyElement_DER_Stretch*>& GetEnergyElements_Stretch() const { return this->m_venerEle_stretch; }
		virtual const vector<EnergyElement_DER_BendTwist*>& GetEnergyElements_BendTwist() const { return this->m_venerEle_bendTwist; }
		virtual const vector<EnergyElement_DER_Connection*>& GetEnergyElements_Connection() const { return this->m_venerEle_connection; }


	protected:

		// Initialization
		virtual void FreeInternal() override;

		virtual void PreInit() override;
		virtual void CreateDiscretization(PtrS<Mesh>& pMesh) override;
		virtual void CreateKinematicsEle(vector<PtrS<KinematicsEle>>& vpEle) override;
		virtual void CreateEnergyElements(vector<IEnergyElement*>& vEnergies) override;
		virtual void CreateMassElements(vector<IMassElement*>& vMasses) override;

	};
}

