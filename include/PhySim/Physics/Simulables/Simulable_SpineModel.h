//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez - Christos Koutras, URJC Madrid
//
//==========================================================

#pragma once

#include <PhySim/CommonIncludes.h>
#include <PhySim/PhySimInterface.h>

#include <PhySim/Physics/Simulables/Simulable_Composite.h>
#include <PhySim/Physics/Simulables/Simulable_RB.h>
#include <PhySim/Physics/Simulables/Simulable_FEM.h>

#include <PhySim/Physics/Elements/EnergyElement_SixDJoint.h>

#include <PhySim/Kinematics/KEleRigidBody3D.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Simulable_SpineModel : public Simulable_Composite
	{
	private:
		PtrS<Simulable_RB> m_rigidBodyModel;
		PtrS<Simulable_FEM> m_softBodyModel;
		vector<PtrS<EnergyElement_SixDJoint>> m_rbEnergyElements;
		VectorXd m_defaultParams;

	public:

		virtual string GetName() const override { return "[Simulable_SpineModel]"; }

		virtual void Setup() override
		{
			Simulable_Composite::Setup();

			m_rigidBodyModel = dynamic_pointer_cast<Simulable_RB>(this->m_pOptions->m_vpSimulables[0]);
			m_softBodyModel = dynamic_pointer_cast<Simulable_FEM>(this->m_pOptions->m_vpSimulables[1]);  

			auto rbEnergyElements = m_rigidBodyModel->GetEnergyElements();
			this->m_rbEnergyElements.resize(rbEnergyElements.size());
			for (int i = 0; i < rbEnergyElements.size(); ++i)
			{
				this->m_rbEnergyElements[i] = dynamic_pointer_cast<EnergyElement_SixDJoint>(rbEnergyElements[i]);
			}
			
			int scale1(1e5), scale2(100);
			m_defaultParams.resize(26);
			double lamemi, lamel, young, poisson;
			young = 30000;
			poisson = 0.4;
			lamemi = 0.5 * young / (1 + poisson);
			lamel = young * poisson / ((1 + poisson) * (1 - 2 * poisson));
			m_defaultParams << 1.1 * scale1, 7.8 * scale1, 1. * scale1, 1.52 * scale2, 1.53 * scale2, 1.48 * scale2,  /// thoracic		
				2.45* scale1, 17.2* scale1, 2.45* scale1, 1.43* scale2, 4.98* scale2, 1.49* scale2,					///lumbar
				53.9 * 1000, 123000, 123000, 9.7, 6.87, 6.87,														/// vert rib
				51.85 * 1000, 8000, 8000, 9.7, 2.29, 2.29,														/// rib sternum
				lamemi, lamel;																						/// young, poisson
		}



		void SetParameters(const VectorXd& vp)
		{		
			VectorXd helpVecInitializeSpineJoints(6);		/// just to add to a vector the 4 parameters under optimization and initialize everything
			helpVecInitializeSpineJoints.setOnes();
			helpVecInitializeSpineJoints[0] = vp[0];
			helpVecInitializeSpineJoints[3] = vp[1];
			helpVecInitializeSpineJoints[4] = vp[2];
			helpVecInitializeSpineJoints[5] = vp[3];


			// vp[0:N] = Parameters of the rigid-body joints
			for (int i = 0; i < 17; i++) {
				MatrixXd K(6, 6);
				K.setZero();
				if (i < 11) {																		/// 11 because of number of thoracic joints
					K = (helpVecInitializeSpineJoints.cwiseProduct(m_defaultParams.segment(0, 6))).asDiagonal();
				}
				else if (i < 17) {																		//// nbVertVertJoints
					K = (helpVecInitializeSpineJoints.cwiseProduct(m_defaultParams.segment(6, 6))).asDiagonal();  /// same parameter but different initialization
				}
				//else if (i < 17 + 24 ) {																////nbVertVertJoints + nbVertRibJoints
				//	K = (vp.segment(6, 6).cwiseProduct(m_defaultParams.segment(12, 6))).asDiagonal();
				//}
				//else {
				//	K = (vp.segment(12, 6).cwiseProduct(m_defaultParams.segment(18, 6))).asDiagonal();
				//}
				m_rbEnergyElements[i]->SetParameters(K);
			}
			//this->DirtyMechanics();



			//PtrS<DomainDistribution_Constant<ParameterSet>> pParamsFem = dynamic_pointer_cast<DomainDistribution_Constant<ParameterSet>>(this->m_softBodyModel->SetupOptions().m_pMatParams);
			////PtrS<DomainDistribution_Constant<ParameterSet>> pParamsFem = dynamic_pointer_cast<DomainDistribution_Constant<ParameterSet>>(this->m_softBodyModel->SetupOptions().m_pMatParams);
			////pParamsFem->Value()
			//ParameterSet& paramsFem= pParamsFem->Value();
			//paramsFem[ParameterSet::Param_Young] = m_defaultParams[24] * vp[4];
			//paramsFem[ParameterSet::Param_Poisson] = m_defaultParams[25] * vp[5];

			////double poisson = paramsFem[ParameterSet::Param_Poisson];

			////////////////            This part works for normal simulation, normal I mean spine simulation, the full sim, for soft tissue params  ////////////////////
			PtrS<ParameterSet> paramsFem = this->m_softBodyModel->SetupOptions().m_pMatParams->GetValueAtDomainPoint(0);
			//PtrS<DomainDistribution_Constant<ParameterSet>> pParamsFem = static_pointer_cast<DomainDistribution_Constant<ParameterSet>>(paramsFem2);
			//ParameterSet& paramsFem = pParamsFem->Value();
			
			///////////// young poisson        ///////////
			//(*paramsFem)[ParameterSet::Param_Young] = m_defaultParams[24] * vp[4];
			//(*paramsFem)[ParameterSet::Param_Poisson] = m_defaultParams[25] * vp[5];
			//PtrS<ParameterSet> pMatParam = std::make_shared<ParameterSet>();
			//pMatParam->InitFromYoungPoisson(m_defaultParams[24] * vp[4], m_defaultParams[25] * vp[5], 1000);
			/////////////end  young poisson        ///////////

			(*paramsFem)[ParameterSet::Param_Lame2] = m_defaultParams[24] * vp[4];
			(*paramsFem)[ParameterSet::Param_Lame1] = m_defaultParams[25] * vp[5];
			PtrS<ParameterSet> pMatParam = std::make_shared<ParameterSet>();
			pMatParam->InitFromLameParameter(m_defaultParams[25] * vp[5], m_defaultParams[24] * vp[4], 1000);
			this->m_softBodyModel->SetupOptions().m_pMatParams.reset(new DomainDistribution_Constant<PtrS<ParameterSet>>(pMatParam));
			this->DirtyMechanics();
			////////////////            End this part works for normal simulation, normal I mean spine simulation, the full sim  ////////////////////


			//////////////////// un comment above for the normal version. Below simple example for simsceneSIMPLE ///////
			//for (int i = 0; i < m_rbEnergyElements.size(); i++) {
			//	MatrixXd K(6, 6);
			//	K.setZero();
			//	for (int j = 0; j < 6; j++) {
			//		K(j, j) = vp[0];
			//	}
			//	m_rbEnergyElements[i]->SetParameters(K);
			//PtrS<ParameterSet> paramsFem = this->m_softBodyModel->SetupOptions().m_pMatParams->GetValueAtDomainPoint(0);
			//(*paramsFem)[ParameterSet::Param_Young] =vp[1];
 		//	(*paramsFem)[ParameterSet::Param_Poisson] = vp[2];				
			//	
			//PtrS<ParameterSet> pMatParam = std::make_shared<ParameterSet>();
			//pMatParam->InitFromYoungPoisson(vp[1], vp[2], 1000);
			//this->m_softBodyModel->SetupOptions().m_pMatParams.reset(new DomainDistribution_Constant<PtrS<ParameterSet>>(pMatParam));
			//this->DirtyMechanics();
			//}
			//
			//////////////////// end un comment above for the normal version. Below simple example for simsceneSIMPLE ///////
		}

		void GetParameters(VectorXd& vp) const
		{
			VectorXd helpVecInitializeSpineJoints(6);		/// just to add to a vector the 4 parameters under optimization and initialize everything
			
			// because of the specific parametrization, I can consrtuct vp with only these values
			MatrixXd K = m_rbEnergyElements[0]->GetParameters();
			helpVecInitializeSpineJoints = K.diagonal().cwiseQuotient(m_defaultParams.segment(0, 6));
			
			//K = m_rbEnergyElements[18]->GetParameters();
			//vp.segment(6, 6) = K.diagonal().cwiseQuotient(m_defaultParams.segment(12, 6));

			//K = m_rbEnergyElements[42]->GetParameters();
			//vp.segment(12, 6) = K.diagonal().cwiseQuotient(m_defaultParams.segment(18, 6));
			

			vp[0] = helpVecInitializeSpineJoints[0];
			vp.segment(1, 3) = helpVecInitializeSpineJoints.segment(3, 3);
			PtrS<ParameterSet> paramsFem = this->m_softBodyModel->SetupOptions().m_pMatParams->GetValueAtDomainPoint(0);
			//PtrS<DomainDistribution_Constant<ParameterSet>> pParamsFem = static_pointer_cast<DomainDistribution_Constant<ParameterSet>>(paramsFem2);
			//ParameterSet& paramsFem = pParamsFem->Value();
			/////////// young poisson        ///////////
			//double young = (*paramsFem)[ParameterSet::Param_Young];
			//double poisson = (*paramsFem)[ParameterSet::Param_Poisson];
			//vp[4] = young / m_defaultParams[24];
			//vp[5] = poisson / m_defaultParams[25];
			/////////// end  young poisson        ///////////
			double lamemi = (*paramsFem)[ParameterSet::Param_Lame2];
			double lamel = (*paramsFem)[ParameterSet::Param_Lame1];
			vp[4] = lamemi / m_defaultParams[24];
			vp[5] = lamel / m_defaultParams[25];
			std::cout << "inside simulab le spine model, line 155 " << std::endl;

			////////////////////// un comment above for the normal version. Below simple exxample for simsceneSIMPLE ///////
			//MatrixXd K = m_rbEnergyElements[0]->GetParameters();
			//vp[0] = K(0, 0);
			//PtrS<ParameterSet> paramsFem = this->m_softBodyModel->SetupOptions().m_pMatParams->GetValueAtDomainPoint(0);
			//double young = (*paramsFem)[ParameterSet::Param_Young];
			//double poisson = (*paramsFem)[ParameterSet::Param_Poisson];
			//vp[1] = young ;
			//vp[2] = poisson ;
			//std::cout << "inside simulab le spine model, line 169" << std::endl;
			//////////////////// end un comment above for the normal version. Below simple example for simsceneSIMPLE ///////


		}
		
		void getRigidVectors(vector<Matrix3d> & rotMat, vector<Vector3d> & posVec, vector<int> &indces)
		{
			vector<PtrS<KinematicsEle>> eles = m_pTree->RootElements();
			for (int i = 0; i < eles.size(); i++) {
				KEleRigidBody3D* pEle = dynamic_cast<KEleRigidBody3D*>(m_pTree->RootElements()[i].get());

				if (pEle != NULL) {
					//, then pEle is a RigidBody you can extract from the the full rotation by calling

					rotMat.push_back(pEle->ComputeFullRotation());
					posVec.push_back(pEle->GetPositionX());
					indces.push_back(pEle->Index());					
				}
			}
		}


		void Compute_DFDp_FiniteDifferences(MatrixSd& DFdp, int nParams)
		{
			MatrixXd DFDp_dense;
			VectorXd vp;
			vp.resize(nParams);
			vp.setZero();
			this->GetParameters(vp);
			std::cout << " Parameters inside Compute_DFDp_FiniteDifferences, after getting them = " << vp << std::endl;
			AVectorXd fplus, fminus;
			this->GetGradient(fplus);
			std::cout << " parameters vp inside Compute_DFDp_FiniteDifferences " << vp << std::endl;
			DFDp_dense.resize(fplus.size(), vp.size());
			DFDp_dense.setZero();
			// FInite differences code
			
			for (int i = 0; i < vp.size(); i++) {
				//std::cout << " gradient inside Compute_DFDp_FiniteDifferences, before set parameters " << fplus.norm() << std::endl;
				double h = 1e-3 * vp[i];
				vp[i] += h;
				this->SetParameters(vp);	
				this->GetParameters(vp);
				std::cout << " Parameters inside Compute_DFDp_FiniteDifferences, after setting them for first time = " << vp << std::endl;
				this->GetGradient(fplus);
				//std::cout << " gradient inside Compute_DFDp_FiniteDifferences, fplus " << fplus.norm() << std::endl;
				this->FixVectorDoFs(fplus);    // this may not necessary
				std::cout << " gradient inside Compute_DFDp_FiniteDifferences, fplus " << fplus.norm() << std::endl;
				vp[i] -= 2 * h;
				this->SetParameters(vp);
				this->GetGradient(fminus);
				this->FixVectorDoFs(fminus); // this may not necessary
				std::cout << " gradient inside Compute_DFDp_FiniteDifferences, fminus " << fminus.norm() << std::endl;
				DFDp_dense.block(0, i, fplus.size(), 1) = -(fplus - fminus) / (2 * h);     
				vp[i] += h;
				this->SetParameters(vp);
				//std::cout<< "DFDp_dense.block(0, i="<< i <<" , 18 * 6, 1) " << DFDp_dense.block(0, i, 18 * 6, 1) << std::endl;
			}
			std::cout << "DFDp_dense.norm()" << DFDp_dense.norm() << std::endl;
			DFdp = DFDp_dense.sparseView(1e-12);
		}
	};
}