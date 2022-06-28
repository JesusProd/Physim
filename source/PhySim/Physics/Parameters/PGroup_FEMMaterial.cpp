////==========================================================
////
////	PhySim library. Generic library for physical simulation.
////
////	Authors:
////			Jesus Perez Rodriguez, jesusprod @ GitHub
////
////==========================================================
//
//#include <PhySim/Physics/Parameters/PGroup_FEMMaterial.h>
//
//#include <PhySim/Physics/Simulables/Simulable_FEM.h>
//#include <PhySim/Physics/Parameters/PDoFSet_Material.h>
//#include <PhySim/Physics/Parameters/PEnergy_FEMMaterial.h>
//
//
//namespace PhySim
//{
//	using namespace std;
//	using namespace Eigen;
//
//	PGroup_FEMMaterial::PGroup_FEMMaterial(Simulable_FEM* pModel, const vector<string>& vparams)
//	{
//		this->m_pModel = pModel;
//		this->m_vparams = vparams;
//	}
//
//	void PGroup_FEMMaterial::CreateParameterEnergies(vector<PtrS<ParameterEnergy>>& vpEne)
//	{
//
//	}
//
//	void PGroup_FEMMaterial::CreateParameterDoFSets(vector<PtrS<ParameterDoFSet>>& vpDoF)
//	{
//		// Collect the set of parameter sets: the number of parameters
//		// depends on the material distribution of the simulated object
//		// and the quadrature used (potentially a differet material per
//		// quadrature point might be used
//
//		set<pair<PtrS<ParameterSet>, PtrS<IMaterialModel>>> parameterSets;
//
//		PtrS<IDomainDistribution<PtrS<ParameterSet>>> pMatParamDist = this->m_pModel->SetupOptions().m_pMatParams;
//		PtrS<IDomainDistribution<PtrS<IMaterialModel>>> pMatModelDist = this->m_pModel->SetupOptions().m_pMatModels;
//		PtrS<IDomainDistribution<PtrS<IQuadrature>>> pQuadratureDist = this->m_pModel->SetupOptions().m_pQuadratures;
//
//		auto numEle = this->m_pModel->GetEnergyElements_FEM().size();
//
//		for (int i = 0; i < numEle; ++i)
//		{
//			PtrS<IQuadrature> pQuadrature = pQuadratureDist->GetValueAtDomainPoint(i);
//
//			const vector<VectorXd>& vpoints = pQuadrature->Points();
//			for (size_t j = 0; j < pQuadrature->Points().size(); ++j)
//			{
//				parameterSets.insert(pair<PtrS<ParameterSet>, PtrS<IMaterialModel>>(
//					pMatParamDist->GetValueAtDomainPoint(i, vpoints[j]), 
//					pMatModelDist->GetValueAtDomainPoint(i, vpoints[i])));
//			}
//		}
//
//		// Create a DoF for each parameter set collected. The parameter
//		// DoF encapsulates access to the specified values required by
//		// the material model if this is 
//
//		for (auto pPair : parameterSets)
//		{
//			vector<string> vselParams;
//
//			const vector<string>& vreqParams = pPair.second->GetRequiredParameters();
//			for (size_t i = 0; i < vreqParams.size(); ++i)
//			{
//				auto found = find(m_vparams.begin(), m_vparams.end(), vreqParams[i]);
//				if (found != m_vparams.end())
//					vselParams.push_back(vreqParams[i]);
//			}
//
//			PtrS<PDoFSet_Material> pParamDoF(new PDoFSet_Material(this->m_pModel, pPair.first, vselParams));
//
//			vpDoF.push_back(dynamic_pointer_cast<ParameterDoFSet>(pParamDoF));
//		}
//	}
//}
