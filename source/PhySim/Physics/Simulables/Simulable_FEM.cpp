//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/EnergyElement_FEM.h>
#include <PhySim/Physics/Simulables/Simulable_FEM.h>

#include <PhySim/Utils/IOUtils.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Simulable_FEM::Simulable_FEM() : Simulable_Mesh() {
  // Nothing to do here...
}

Simulable_FEM::~Simulable_FEM() {
  this->FreeInternal();

#ifndef NDEBUG
  IOUtils::logTrace(Verbosity::V1_Default,
                    "\n[DEBUG] Deleting Simulable_ThinShell");
#endif
}

void Simulable_FEM::FreeInternal() {
  Simulable_Mesh::FreeInternal();
  this->m_venergyEle_fem.clear();
}

Simulable_FEM::Options& Simulable_FEM::SetupOptions() {
  if (this->m_pOptions == NULL)  // Create if needed
    this->m_pOptions = new Simulable_FEM::Options();
  return *((Simulable_FEM::Options*)this->m_pOptions);
}

void Simulable_FEM::PreInit() {
  Simulable_Mesh::PreInit();

  // Setup default material parameters

  if (this->SetupOptions().m_pMatParams == NULL) {
    PtrS<ParameterSet> pMatParam(new ParameterSet());
    pMatParam->InitLinearFromYoungPoisson(1e9, 0.3, 1000);
    this->SetupOptions().m_pMatParams.reset(
        new DomainDistribution_Constant<PtrS<ParameterSet>>(pMatParam));
  }

  // Setup default material parameters

  if (this->SetupOptions().m_pMatModels == NULL) {
    PtrS<IMaterialModel> pMatModel;

    switch (this->SetupOptions().m_pMesh->MeshType()) {
      case Discretization::Tri3:
      case Discretization::Tri6:
      case Discretization::Quad4:
      case Discretization::Quad8:
        pMatModel.reset(new MaterialModel_2Din3D_StVK());
        break;
      case Discretization::Tet4:
      case Discretization::Hex8:
      case Discretization::Tet10:
      case Discretization::Hex20:
        pMatModel.reset(new MaterialModel_3Din3D_StVK());
        break;
      default:
        throw PhySim::exception("[ERROR] Invalid discretization value");
    }

    this->SetupOptions().m_pMatModels.reset(
        new DomainDistribution_Constant<PtrS<IMaterialModel>>(pMatModel));
  }

  // Setup default shape functions

  if (this->SetupOptions().m_pShapeFunctions == NULL) {
    PtrS<IShapeFunction> pShapeFunction;

    switch (this->SetupOptions().m_pMesh->MeshType()) {
      case Discretization::Tri3:
        pShapeFunction = ShapeFunction_Tri3::Instance();
        break;
      case Discretization::Tri6:
        pShapeFunction = ShapeFunction_Tri6::Instance();
        break;
      case Discretization::Quad4:
        pShapeFunction = ShapeFunction_Quad4::Instance();
        break;
      case Discretization::Quad8:
        pShapeFunction = ShapeFunction_Quad8::Instance();
        break;
      case Discretization::Tet4:
        pShapeFunction = ShapeFunction_Tet4::Instance();
        break;
      case Discretization::Hex8:
        pShapeFunction = ShapeFunction_Hex8::Instance();
        break;
      case Discretization::Tet10:
        pShapeFunction = ShapeFunction_Tet10::Instance();
        break;
      case Discretization::Hex20:
        pShapeFunction = ShapeFunction_Hex20::Instance();
        break;
      default:
        throw PhySim::exception("[ERROR] Invalid discretization value");
    }

    this->SetupOptions().m_pShapeFunctions.reset(
        new DomainDistribution_Constant<PtrS<IShapeFunction>>(pShapeFunction));
  }

  // Setup default quadrature

  if (this->SetupOptions().m_pQuadratures == NULL) {
    PtrS<IQuadrature> pQuadrature;

    switch (this->SetupOptions().m_pMesh->MeshType()) {
      case Discretization::Tri3:
      case Discretization::Tri6:
        pQuadrature = Quadrature_Tri1::Instance();
        break;
      case Discretization::Tet4:
        pQuadrature = Quadrature_Tet1::Instance();
        break;
      case Discretization::Tet10:
        pQuadrature = Quadrature_Tet8::Instance();
        break;
      case Discretization::Quad4:
      case Discretization::Quad8:
        pQuadrature.reset(new Quadrature_Gauss(2, 2));
        break;
      case Discretization::Hex8:
      case Discretization::Hex20:
        pQuadrature.reset(new Quadrature_Gauss(3, 2));
        break;
      default:
        throw PhySim::exception("[ERROR] Invalid discretization value");
    }

    this->SetupOptions().m_pQuadratures.reset(
        new DomainDistribution_Constant<PtrS<IQuadrature>>(pQuadrature));
  }
}

void Simulable_FEM::CreateEnergyElements(vector<IEnergyElement*>& venerEle) {
  const Options& options = this->SetupOptions();

  Simulable_Mesh::CreateEnergyElements(venerEle);

  size_t numElems = this->m_pMesh->NumElems();
  size_t curElems = this->m_venerEle.size();
  m_venergyEle_fem.resize(numElems);
  venerEle.reserve(numElems);

  for (size_t i = 0; i < numElems; ++i) {
    this->m_venergyEle_fem[i] =
        new EnergyElement_FEM(this, this->m_pMesh->Elems()[i]);

    venerEle.push_back(this->m_venergyEle_fem[i]);
  }
}

void Simulable_FEM::GetRestStrains(vector<MatrixXd>& vE0) const {
  size_t numFem = this->m_venergyEle_fem.size();

  vE0.resize(numFem);

  for (int i = 0; i < numFem; ++i) {
    vE0[i] = this->m_venergyEle_fem[i]->GetRestStrain();
  }
}

void Simulable_FEM::SetRestStrains(const vector<MatrixXd>& vE0) {
  size_t numFem = this->m_venergyEle_fem.size();

  assert(vE0.size() == numFem);

  for (int i = 0; i < numFem; ++i) {
    this->m_venergyEle_fem[i]->SetRestStrain(vE0[i]);
  }

  this->DirtyMechanics();
}

}  // namespace PhySim
