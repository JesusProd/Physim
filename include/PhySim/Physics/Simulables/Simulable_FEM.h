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

#include <PhySim/Physics/Elements/MaterialModels.h>
#include <PhySim/Geometry/Polytopes/Quadratures.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class IEnergyElement;
	class EnergyElement_FEM;
	class IShapeFunction;

	/**
	* FEM simulable model.
	*/
	class Simulable_FEM : public Simulable_Mesh
	{
	public:

		/**
		* FEM simulable model options.
		*/
		struct Options : public Simulable_Mesh::Options
		{
			/**
			* Domain distributions specifying the material model used at
			* each element of the discretization. If left unchanged, a uniform
			* distribution of the default material model associated with the
			* discretization element is used.
			*/
			PtrS<IDomainDistribution<PtrS<IMaterialModel>>> m_pMatModels;

			/**
			* Domain distributions specifying the quadrature used at
			* each element of the discretization. If left unchanged, a uniform
			* distribution of the default quadrature associated with the
			* discretization element is used.
			*/
			PtrS<IDomainDistribution<PtrS<IQuadrature>>> m_pQuadratures;
			
			/**
			* Domain distributions specifying the shape function used at
			* each element of the discretization. If left unchanged, a uniform
			* distribution of the default shape function associated with the
			* discretization element is used.
			*/
			PtrS<IDomainDistribution<PtrS<IShapeFunction>>> m_pShapeFunctions;

			Options()
			{
				// Nothing to do here...
			}
		};

	protected:

		vector<EnergyElement_FEM*> m_venergyEle_fem;

	public:

		virtual string GetName() const override { return "[FEM System]"; }

		virtual Simulable_FEM::Options& SetupOptions() override;

		/**
		* Constructor.
		*/
		Simulable_FEM();

		/**
		* Destructor.
		*/
		virtual ~Simulable_FEM();

		virtual PtrS<IDomainDistribution<PtrS<ParameterSet>>>& MatParamsDistribution() { return this->SetupOptions().m_pMatParams; }
		virtual PtrS<IDomainDistribution<PtrS<IMaterialModel>>>& MatModelsDistribution() { return this->SetupOptions().m_pMatModels; }
		virtual PtrS<IDomainDistribution<PtrS<IQuadrature>>>& QuadraturesDistribution() { return this->SetupOptions().m_pQuadratures; }
		virtual PtrS<IDomainDistribution<PtrS<IShapeFunction>>>& ShapeFunctionsDistribution() { return this->SetupOptions().m_pShapeFunctions; }

		virtual void GetRestStrains(vector<MatrixXd>& vE0) const;
		virtual void SetRestStrains(const vector<MatrixXd>& vE0);

		virtual const vector<EnergyElement_FEM*>& GetEnergyElements_FEM() const { return this->m_venergyEle_fem; }

	protected:

		virtual void PreInit() override;

		virtual void FreeInternal() override;

		virtual void CreateEnergyElements(vector<IEnergyElement*>& vEnergies) override;

	};
}

