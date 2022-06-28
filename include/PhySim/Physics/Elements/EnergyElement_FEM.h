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


#include <PhySim/Physics/Elements/EnergyElement.h>

#include <PhySim/Physics/Elements/MaterialModels.h>
#include <PhySim/Geometry/Polytopes/Quadratures.h>
#include <PhySim/Geometry/Polytopes/ShapeFunctions.h>

#include <PhySim/Kinematics/KinematicsEle.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Poly;
	class Simulable_FEM;

	class EnergyElement_FEM: public EnergyElement
	{
	protected:
		Poly* m_pElemPoly;

		VectorXd m_vintWei;
		MatrixXd m_mstrain0;

		PtrS<IShapeFunction::IDeformationData> m_pSampleData;

		Simulable_FEM* m_pSimulFEM;

	public:
		EnergyElement_FEM(Simulable_FEM* pSimul, Poly* pPoly);
		virtual ~EnergyElement_FEM(void);

		virtual void Init();

		virtual void UpdateKinematics() override;
		virtual void UpdateMechanics() override;
		virtual void ComputeAndStore_Energy_Internal() override;
		virtual void ComputeAndStore_Gradient_Internal() override;
		virtual void ComputeAndStore_Hessian_Internal() override;

		virtual const VectorXd& GetIntegrationWeights() const { return this->m_vintWei; }

		virtual MatrixXd ComputeRestStrain();

		inline virtual MatrixXd GetRestStrain() const { return this->m_mstrain0; }
		inline virtual void SetRestStrain(MatrixXd e0) { this->m_mstrain0 = e0; }

	};
}

