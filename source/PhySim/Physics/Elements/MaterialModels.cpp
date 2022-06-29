//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/MaterialModels.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

PtrS<MaterialModel_2Din3D_StVK> MaterialModel_2Din3D_StVK::PINSTANCE =
    PtrS<MaterialModel_2Din3D_StVK>(NULL);
PtrS<MaterialModel_2Din3D_CoNH> MaterialModel_2Din3D_CoNH::PINSTANCE =
    PtrS<MaterialModel_2Din3D_CoNH>(NULL);
PtrS<MaterialModel_2Din3D_InNH> MaterialModel_2Din3D_InNH::PINSTANCE =
    PtrS<MaterialModel_2Din3D_InNH>(NULL);
PtrS<MaterialModel_3Din3D_StVK> MaterialModel_3Din3D_StVK::PINSTANCE =
    PtrS<MaterialModel_3Din3D_StVK>(NULL);
PtrS<MaterialModel_3Din3D_CoNH> MaterialModel_3Din3D_CoNH::PINSTANCE =
    PtrS<MaterialModel_3Din3D_CoNH>(NULL);
PtrS<MaterialModel_3Din3D_CoMR> MaterialModel_3Din3D_CoMR::PINSTANCE =
    PtrS<MaterialModel_3Din3D_CoMR>(NULL);
PtrS<MaterialModel_3Din3D_SNH> MaterialModel_3Din3D_SNH::PINSTANCE =
    PtrS<MaterialModel_3Din3D_SNH>(NULL);
vector<string> MaterialModel_3Din3D_SNH::m_vrequired =
    vector<string>({ParameterSet::Param_Lame1, ParameterSet::Param_Lame2,
                    ParameterSet::Param_SNHAlpha});
vector<string> MaterialModel_2Din3D_StVK::m_vrequired =
    vector<string>({ParameterSet::Param_Lame1, ParameterSet::Param_Lame2});
vector<string> MaterialModel_2Din3D_CoNH::m_vrequired =
    vector<string>({ParameterSet::Param_Lame1, ParameterSet::Param_Lame2});
vector<string> MaterialModel_2Din3D_InNH::m_vrequired =
    vector<string>({ParameterSet::Param_Lame1, ParameterSet::Param_Lame2});
vector<string> MaterialModel_3Din3D_StVK::m_vrequired =
    vector<string>({ParameterSet::Param_Lame1, ParameterSet::Param_Lame2});
vector<string> MaterialModel_3Din3D_CoNH::m_vrequired =
    vector<string>({ParameterSet::Param_Lame1, ParameterSet::Param_Lame2});
vector<string> MaterialModel_3Din3D_CoMR::m_vrequired =
    vector<string>({ParameterSet::Param_Mooney01, ParameterSet::Param_Mooney10,
                    ParameterSet::Param_Bulk});
}  // namespace PhySim