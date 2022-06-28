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
	* Simulable.
	*
	* TODO.
	*/
	class Simulable_Composite : public Simulable
	{
	public:

		struct DoFCoupling
		{
			enum Type
			{
				HardFixed,
				SoftFixed,
				HardEmbedding,
				SoftEmbedding,
				RigidBody
			};

			Type m_type;
			MatrixXi m_mC;
		};

		/**
		* TODO.
		*/
		struct Options
		{
			vector<PtrS<Simulable>> m_vpSimulables;
			vector<DoFCoupling> m_vDoFCouplings;

			Options()
			{
				
			}

			virtual ~Options()
			{
				// Nothing to do here...
			}
		};

	protected:

		Options* m_pOptions;

	public:

		virtual string GetName() const override { return "[Composite]"; }

		virtual Simulable_Composite::Options& SetupOptions();

		/**
		* Constructor.
		*/
		Simulable_Composite();

		/**
		* Denstructor.
		*/
		virtual ~Simulable_Composite();

		virtual vector<IDoFSet*> SelectDoF(const Vector3d& vboxMin, const Vector3d& vboxMax, Tag s) const override;

		virtual void AddEnergy(Real& energy, bool addBC = true) override;

		virtual void AddGradient(AVectorXd& vgradient, bool addBC = true) override;

		virtual void AddHessian(AMatrixSd& mHessian, bool addBC = true) override;

		vector<PtrS<Geometry>> Geometries() const override;

		virtual void SetUseFDGradientAtElements(bool u);
		virtual void SetUseFDHessianAtElements(bool u);

		virtual void UpdateMechanics() override;
		//virtual void UpdateKinematics() override;

		virtual void ComputeAndStore_Fixed();
		//virtual void ComputeAndStore_Mass() override;
		//virtual void ComputeAndStore_Rest() override;
		virtual void ComputeAndStore_Energy(bool addBC = true) override;
		virtual void ComputeAndStore_Gradient(bool addBC = true) override;
		virtual void ComputeAndStore_Hessian(bool addBC = true) override;
		//virtual void ComputeAndStore_Constraint(bool addBC = true) override;
		//virtual void ComputeAndStore_Jacobian(bool addBC = true) override;

		virtual void DirtyMass();
		virtual void DirtyRest();
		//virtual void DirtyKinematics();
		virtual void DirtyMechanics();
		virtual void DirtyFixed();

		//virtual bool IsDirty_Kinematics() const;
		//virtual bool IsDirty_Mechanics() const;

		//virtual bool IsDirty_Energy() const;
		//virtual bool IsDirty_Gradient() const;
		//virtual bool IsDirty_Hessian() const;
		//virtual bool IsDirty_Rest() const;
		//virtual bool IsDirty_Mass() const;


		virtual bool IsDirty_Kinematics() const override;
		virtual bool IsDirty_Mechanics() const override;

		virtual bool IsDirty_Energy() const override;
		virtual bool IsDirty_Gradient() const override;
		virtual bool IsDirty_Hessian() const override;
		virtual bool IsDirty_Rest() const override;
		virtual bool IsDirty_Mass() const override;
		virtual bool IsDirty_Constraint() const override;
		virtual bool IsDirty_Jacobian() const override;
		virtual bool IsDirty_Fixed() const override;

		virtual bool BoundaryConditionsLoaded() override;

		virtual void ResetBoundaryConditions() override;

		virtual void FullBoundaryConditions() override;

		virtual void StepBoundaryConditions() override;

	protected:

		virtual void FreeInternal() override;
		virtual void InitInternal() override;

		virtual void addSoftFixedCoupling(DoFCoupling& coupling, vector<PtrS<KinematicsEle>>& vpMapped, vector<PtrS<KinematicsEle>>& vpAdded, vector<PtrS<EnergyElement>>& vpEnergy);
		virtual void addHardFixedCoupling(DoFCoupling& coupling, vector<PtrS<KinematicsEle>>& vpMapped, vector<PtrS<KinematicsEle>>& vpAdded, vector<PtrS<EnergyElement>>& vpEnergy);
		virtual void addSoftEmbeddingCoupling(DoFCoupling& coupling, vector<PtrS<KinematicsEle>>& vpMapped, vector<PtrS<KinematicsEle>>& vpAdded, vector<PtrS<EnergyElement>>& vpEnergy);
		virtual void addHardEmbeddingCoupling(DoFCoupling& coupling, vector<PtrS<KinematicsEle>>& vpMapped, vector<PtrS<KinematicsEle>>& vpAdded, vector<PtrS<EnergyElement>>& vpEnergy);
		virtual void addRigidBodyCoupling(DoFCoupling& coupling, vector<PtrS<KinematicsEle>>& vpMapped, vector<PtrS<KinematicsEle>>& vpAdded, vector<PtrS<EnergyElement>>& vpEnergy);

	};
}