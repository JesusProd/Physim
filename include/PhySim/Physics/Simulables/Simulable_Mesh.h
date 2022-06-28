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
#include <PhySim/Kinematics/KinematicsTree.h>

#include <PhySim/Utils/DomainDistribution.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	/**
	* Simulable_Mesh.
	*
	* TODO.
	*/
	class Simulable_Mesh : public Simulable
	{
	public:

		/**
		* TODO.
		*/
		struct Options
		{
			PtrS<Mesh> m_pMesh;

			/**
			* Domain distributions specifying the material parameters used at
			* each element of the discretization. If left unchanged, a uniform
			* distribution of the default material parameters associated with the
			* discretization element is used.
			*/
			PtrS<IDomainDistribution<PtrS<ParameterSet>>> m_pMatParams;

			Options()
			{
				m_pMesh.reset();
			}

			virtual ~Options()
			{
				// Nothing to do here...
			}
		};

	protected:

		Options* m_pOptions;
		PtrS<Mesh> m_pMesh;

	public:
		
		virtual string GetName() const override { return "Mesh-Based System"; }

		virtual Simulable_Mesh::Options& SetupOptions();

		/**
		* Constructor.
		*/
		Simulable_Mesh();

		/**
		* Denstructor.
		*/
		virtual ~Simulable_Mesh();

		//virtual Geometry* Geometry() const { return this->m_pMesh.get(); }

		virtual vector<PtrS<Geometry>> Geometries() const override;

		inline virtual Mesh& GetMesh() { return *this->m_pMesh; }
		inline virtual const vector<Node*>& GetNodes() const { return m_pMesh->Nodes(); }
		inline virtual const vector<Poly*>& GetElems() const { return m_pMesh->Elems(); }

		virtual void GetNodeDOF(MatrixXr& mV, Tag t = Tag::Tag_Position_X) const
		{
			VectorXr vx;
			this->GetDOFVector(vx, t);
			memcpy(mV.data(), vx.data(), sizeof(Real) * 3 * this->m_pMesh->NumNodes());
		};

		virtual void SetNodeDOF(const MatrixXr& mV, Tag t = Tag::Tag_Position_X)
		{
			assert(mV.cols() == 3);

			MatrixXd mVt = mV.transpose();
			VectorXr vx;
			this->GetDOFVector(vx, t);
			memcpy(vx.data(), mVt.data(), sizeof(Real) * 3 * this->m_pMesh->NumNodes());
			this->SetDOFVector(vx, t);
		}

		virtual void InitSubelementPositions(const MatrixXd& mV, Tag s);

		virtual vector<IDoFSet*> SelectDoF(const Vector3d& vboxMin, const Vector3d& vboxMax, Tag s) const override;
	
	protected:
		
		virtual void InitInternal() override;
		virtual void FreeInternal() override;

		virtual void PreInit();
		virtual void PostInit();

		virtual void CreateDiscretization(PtrS<Mesh>& pMesh);
		virtual void CreateKinematicsEle(vector<PtrS<KinematicsEle>>& vpEle);
		virtual void CreateEnergyElements(vector<IEnergyElement*>& vEnergies);
		virtual void CreateConstraintSets(vector<IConstraintSet*>& vConstraints);
		virtual void CreateMassElements(vector<IMassElement*>& vMasses);

		//virtual void CreateKinematicsEle(vector<KinematicsEle*>& vDoFs);

	};
}