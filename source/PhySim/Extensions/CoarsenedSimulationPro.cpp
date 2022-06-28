////==========================================================
////
////	PhySim library. Generic library for physical simulation.
////
////	Authors:
////			Jesus Perez Rodriguez, URJC Madrid
////
////==========================================================
//
//#include <PhySim/Extensions/CoarsenedSimulationPro.h>
//
//namespace PhySim
//{
//	using namespace std;
//	using namespace Eigen;
//
//	void CoarsenedSimulationPro::Init(const Vector3d& vorigin, const Vector3d &vranges, const Vector3i &vfineDims, const Vector3i &vcoarDims, const vector<Voxel> &voccupancy, const vector<Material> &vmaterials)
//	{
//		// Build coarsened grid mesh ----------------------------------------------------
//		// ------------------------------------------------------------------------------
//		this->QuadraturePerDimension = (vfineDims(0) / vcoarDims[0]);
//		iVector voccupancyIdx((int)voccupancy.size());
//		for (int i = 0; i < (int)voccupancy.size(); ++i)
//			voccupancyIdx[i] = voccupancy[i].VoxIdx;
//
//		vector<Tag> vnTraits(3);
//		vnTraits[0] = Tag_Position_X;
//		vnTraits[1] = Tag_Position_0;
//		vnTraits[2] = Tag_Velocity;
//		this->m_pCoarsenedMesh.reset(new Mesh_GridCoarsen(vorigin, vranges, vfineDims, vcoarDims, voccupancyIdx, Discretization_Hex8, vnTraits));
//
//		VectorXi vcoarsening = this->m_pCoarsenedMesh->Coar();
//
//		// Create material distribution
//
//		m_vpMatCatalogue.resize(vmaterials.size() + 1);
//		m_vpMatCatalogue[0].reset(new ParameterSet());
//		m_vpMatCatalogue[0]->InitLinearFromYoungPoisson(0, 0, 0);
//		for (int i = 0; i < (int)vmaterials.size(); ++i)
//		{
//			m_vpMatCatalogue[i + 1].reset(new ParameterSet());
//			m_vpMatCatalogue[i + 1]->InitLinearFromYoungPoisson(vmaterials[i].Young,
//														  vmaterials[i].Poisson,
//														  vmaterials[i].Density);
//		}
//
//		// Create material mapping
//
//		for (int i = 0; i < (int)voccupancy.size(); ++i)
//		{
//			int fineGlobalActiveIdx = this->m_pCoarsenedMesh->FineMesh()->MapFull2Active_Elems()[voccupancy[i].VoxIdx];
//			assert(i == fineGlobalActiveIdx);
//			this->m_pCoarsenedMesh->FineMesh()->Elems()[i]->Traits().AddTrait<int>(Tag_Mat_0, voccupancy[i].MatIdx + 1);
//		}
//
//		// Build coarsened model --------------------------------------------------------
//		// ------------------------------------------------------------------------------
//
//		this->RecomputeCoarsenedModel();
//
//		// Build fine model -------------------------------------------------------------
//		// ------------------------------------------------------------------------------
//
//		this->RecomputeFullFineModel();
//
//		// Build fine submodels ---------------------------------------------------------
//		// ------------------------------------------------------------------------------
//
//		for (int i = 0; i < this->m_pCoarsenedMesh->NumElems(); ++i)
//			this->RecomputeFineSubModel(i);
//	}
//
//	Mesh_GridCoarsen::TopologyChanges CoarsenedSimulationPro::CutMesh_Disc(Real radius, const Matrix3d &mR, const Vector3d &vt, Tag vs)
//	{
//		Mesh_GridCoarsen::TopologyChanges changes = this->m_pCoarsenedMesh->CutMesh_Disc(radius, mR, vt, vs);
//
//		for (int i = 0; i < (int)changes.vnewElems.size(); ++i)
//			this->RecomputeFineSubModel(changes.vnewElems[i]);
//
//		this->RecomputeFullFineModel();
//		this->RecomputeCoarsenedModel();
//
//		return changes;
//	}
//
//	Mesh_GridCoarsen::TopologyChanges CoarsenedSimulationPro::CutMesh_Rect(Real length, Real height, const Matrix3d &mR, const Vector3d &vt, Tag vs)
//	{
//		Mesh_GridCoarsen::TopologyChanges changes = this->m_pCoarsenedMesh->CutMesh_Rect(length, height, mR, vt, vs);
//
//		for (int i = 0; i < (int)changes.vnewElems.size(); ++i)
//			this->RecomputeFineSubModel(changes.vnewElems[i]);
//
//		this->RecomputeFullFineModel();
//		this->RecomputeCoarsenedModel();
//
//		return changes;
//	}
//
//	void CoarsenedSimulationPro::SetCoarsenedShapeFunction(int idx, const vector<vector<Matrix3d>> &vweights)
//	{
//		auto pSFDist = this->m_pCoarsenedModel->SetupOptions().m_pShapeFunctions;
//		auto pSFVarDist = dynamic_pointer_cast<DomainDistribution_Variable<PtrS<IShapeFunction>>>(pSFDist);
//		pSFVarDist->m_vvalue[idx].reset(new ShapeFunction_Chen(vweights, this->m_pCoarsenedMesh->Coar()));
//
//		this->m_pCoarsenedModel->GetEnergyElements_FEM()[idx]->Init();
//		this->m_pCoarsenedModel->GetEnergyElements_FEM()[idx]->UpdateKinematics();
//		this->m_pCoarsenedModel->GetEnergyElements_FEM()[idx]->UpdateMechanics();
//
//		// Set the default shape function of the mesh for that specific coarse element
//
//		this->m_pCoarsenedMesh->Elems()[idx]->SetShapeFunction(pSFVarDist->m_vvalue[idx]);
//	}
//
//	PtrS<Simulable_FEM_Volume> CoarsenedSimulationPro::ComputeFineFullModel()
//	{
//		PtrS<Simulable_FEM_Volume> pModel(new Simulable_FEM_Volume());
//
//		// Set simulation mesh
//
//		PtrS<Mesh_Hexa> pMesh = this->m_pCoarsenedMesh->FineMesh();
//		pModel->SetupOptions().m_pMesh = dynamic_pointer_cast<Mesh>(pMesh);
//
//		// Set material distribution
//
//		iVector vmeshMats((int)pMesh->NumElems());
//		for (int j = 0; j < (int)vmeshMats.size(); ++j)
//			vmeshMats[j] = pMesh->Elems()[j]->Trait<int>(Tag::Tag_Mat_0);
//
//		pModel->SetupOptions().m_pMatParams.reset(new DomainDistribution_Variable<PtrS<ParameterSet>>(m_vpMatCatalogue, vmeshMats));
//
//		// Setup quadrature type
//
//		pModel->SetupOptions().m_pQuadratures.reset(new DomainDistribution_Constant<PtrS<IQuadrature>>(PtrS<IQuadrature>(new Quadrature_Gauss(3, 2))));
//
//		// Setup material models
//
//		pModel->SetupOptions().m_pMatModels.reset(new DomainDistribution_Constant<PtrS<IMaterialModel>>(MaterialModel_3Din3D_CoNH::Instance()));
//
//		// Set shape distribution
//
//		pModel->SetupOptions().m_pShapeFunctions.reset(new DomainDistribution_Constant<PtrS<IShapeFunction>>(ShapeFunction_Hex8::Instance()));
//
//		// Setup!
//
//		pModel->Setup();
//
//		return pModel;
//	}
//
//	void CoarsenedSimulationPro::RecomputeFullFineModel()
//	{
//		this->m_pFullFineModel = this->ComputeFineFullModel();
//	}
//
//	void CoarsenedSimulationPro::RecomputeCoarsenedModel()
//	{
//		this->m_pCoarsenedModel.reset(new Simulable_FEM_Volume());
//
//		// Set simulation mesh
//
//		this->m_pCoarsenedModel->SetupOptions().m_pMesh = this->m_pCoarsenedMesh;
//
//		// Create material distribution
//
//		int numSubEle =
//			this->m_pCoarsenedMesh->Coar().x() *
//			this->m_pCoarsenedMesh->Coar().y() *
//			this->m_pCoarsenedMesh->Coar().z();
//		MatrixXi mMat(this->m_pCoarsenedMesh->NumElems(), numSubEle);
//		mMat.setZero();
//		for (int i = 0; i < this->m_pCoarsenedMesh->NumElems(); ++i)
//		{
//			Poly *pCoarElem = this->m_pCoarsenedMesh->Elems()[i];
//			auto &coarElemMeta = pCoarElem->Trait<Mesh_GridCoarsen::CoarElemMeta>(Tag::Tag_CoarMeta);
//			for (auto it = coarElemMeta.vfineElem.begin(); it != coarElemMeta.vfineElem.end(); ++it)
//			{
//				Poly *pFineElem = (*it);
//				auto &fineMeta = pFineElem->Trait<Mesh_GridCoarsen::FineElemMeta>(Tag::Tag_FineMeta);
//				mMat(fineMeta.coarElem->ID(), fineMeta.localFullIdx) = pFineElem->Trait<int>(Tag_Mat_0);
//			}
//		}
//
//		// Store the material vector of the coarse element in an elem trait
//
//		for (int i = 0; i < this->m_pCoarsenedMesh->NumElems(); ++i)
//		{
//			this->m_pCoarsenedMesh->Elems()[i]->Traits().AddTrait<VectorXi>(Tag::Tag_Mat_0, mMat.row(i));
//		}
//
//		// Set material parameters
//
//		this->m_pCoarsenedModel->SetupOptions().m_pMatParams.reset(new DomainDistribution_Subelement<PtrS<ParameterSet>>(m_vpMatCatalogue, mMat, m_pCoarsenedMesh->Coar()));
//
//		// Set quadrature type
//
//		//this->m_pCoarsenedModel->SetupOptions().m_pQuadratures.reset(new DomainDistribution_Constant<PtrS<IQuadrature>>(PtrS<IQuadrature>(new Quadrature_FineGauss(QuadraturePerDimension))));
//		this->m_pCoarsenedModel->SetupOptions().m_pQuadratures.reset(new DomainDistribution_Constant<PtrS<IQuadrature>>(PtrS<IQuadrature>(new Quadrature_Gauss(3, QuadraturePerDimension))));
//		//this->m_pCoarsenedModel->SetupOptions().m_pQuadratures.reset(new DomainDistribution_Constant<PtrS<IQuadrature>>(PtrS<IQuadrature>(new Quadrature_Gauss(3,QuadraturePerDimension))));
//
//		// Setup material models
//
//		this->m_pCoarsenedModel->SetupOptions().m_pMatModels.reset(new DomainDistribution_Constant<PtrS<IMaterialModel>>(MaterialModel_3Din3D_CoNH::Instance()));
//
//		// Set shape distribution: initially a variable distribution, i.e.
//		// one different shape function for each of the coarse elements.
//		// By default, such shape function is the trilinar interpolation
//		// Later this will be substituted by the optimized shape functions
//
//		vector<PtrS<IShapeFunction>> vSFvalues(m_pCoarsenedMesh->NumElems());
//		for (int i = 0; i < this->m_pCoarsenedMesh->NumElems(); ++i)
//			vSFvalues[i] = ShapeFunction_Hex8::Instance();
//
//		this->m_pCoarsenedModel->SetupOptions().m_pShapeFunctions.reset(new DomainDistribution_Variable<PtrS<IShapeFunction>>(vSFvalues));
//
//		// Setup the model
//
//		this->m_pCoarsenedModel->Setup();
//	}
//
//	void CoarsenedSimulationPro::RecomputeFineSubModel(int i)
//	{
//		PtrS<Simulable_FEM_Volume> pModel(new Simulable_FEM_Volume());
//
//		// Set simulation mesh
//
//		PtrS<Mesh_Hexa> pSubmesh = this->m_pCoarsenedMesh->GetFineSubmesh(i);
//		pModel->SetupOptions().m_pMesh = dynamic_pointer_cast<Mesh>(pSubmesh);
//
//		// Set material distribution
//
//		auto &coarMeta = this->m_pCoarsenedMesh->Elems()[i]->Trait<Mesh_GridCoarsen::CoarElemMeta>(Tag::Tag_CoarMeta);
//		iVector vsubMeshMats((int)coarMeta.vfineElem.size());
//		for (int j = 0; j < (int)vsubMeshMats.size(); ++j)
//			vsubMeshMats[j] = coarMeta.vfineElem[j]->Trait<int>(Tag::Tag_Mat_0);
//
//		pModel->SetupOptions().m_pMatParams.reset(new DomainDistribution_Variable<PtrS<ParameterSet>>(m_vpMatCatalogue, vsubMeshMats));
//
//		// Setup quadrature type
//
//		pModel->SetupOptions().m_pQuadratures.reset(new DomainDistribution_Constant<PtrS<IQuadrature>>(PtrS<IQuadrature>(new Quadrature_Gauss(3, 2))));
//
//		// Setup material models
//
//		pModel->SetupOptions().m_pMatModels.reset(new DomainDistribution_Constant<PtrS<IMaterialModel>>(MaterialModel_3Din3D_CoNH::Instance()));
//
//		// Set shape distribution
//
//		pModel->SetupOptions().m_pShapeFunctions.reset(new DomainDistribution_Constant<PtrS<IShapeFunction>>(ShapeFunction_Hex8::Instance()));
//
//		// Setup!
//
//		pModel->Setup();
//
//		// Store the mesh in a trait
//
//		this->m_pCoarsenedMesh->Elems()[i]->Traits().AddTrait<PtrS<Simulable_FEM_Volume>>(Tag::Tag_Model_0, pModel);
//	}
//
//	void CoarsenedSimulationPro::ComputeSubmeshHessian(const iVector &vcoarElemIdx,
//													   iVector &vmapSel2Full,
//													   iVector &vmapFull2Sel,
//													   iVector &vselBoundary,
//													   MatrixSd &mH)
//	{
//		// Colect selected nodes and elements
//
//		set<int> sselectedNodes;
//		set<int> sselectedElems;
//		for (int i = 0; i < (int)vcoarElemIdx.size(); ++i)
//		{
//			auto coarElemMeta = this->m_pCoarsenedMesh->Elems()[vcoarElemIdx[i]]->Trait<Mesh_GridCoarsen::CoarElemMeta>(Tag::Tag_CoarMeta);
//			for (int j = 0; j < (int)coarElemMeta.vfineElem.size(); ++j)
//				sselectedElems.insert(coarElemMeta.vfineElem[j]->ID());
//			for (int j = 0; j < (int)coarElemMeta.vfineNode.size(); ++j)
//				sselectedNodes.insert(coarElemMeta.vfineNode[j]->ID());
//		}
//
//		int N = this->m_pCoarsenedMesh->FineMesh()->NumNodes();
//		int M = (int)sselectedNodes.size();
//
//		// Build (two-way) selection maps
//
//		int countSel = 0;
//		vmapSel2Full.reserve(M);
//		vmapFull2Sel.resize(N, -1);
//		for (auto it = sselectedNodes.begin(); it != sselectedNodes.end(); ++it)
//		{
//			vmapSel2Full.push_back(*it);
//			vmapFull2Sel[*it] = countSel++;
//		}
//
//		// Build selection matrix
//
//		MatrixSd mSelected;
//		Utils::buildSelectionMatrix(vmapSel2Full, N, mSelected);
//		Utils::buildBlockExpandedMatrix(mSelected, 3, mSelected);
//
//		// Build selected Hessian
//
//		iVector vselectedElems;
//		vselectedElems.insert(vselectedElems.begin(),
//							  sselectedElems.begin(),
//							  sselectedElems.end());
//		this->m_pFullFineModel->SetSelectedElements(vselectedElems);
//
//		AMatrixSd mFullHessian;
//		this->m_pFullFineModel->GetHessian(mFullHessian);
//		mH = mSelected * mFullHessian * mSelected.transpose();
//
//		this->m_pFullFineModel->SetSelectedElements(iVector());
//
//		// Find submesh boundary
//
//		set<int> sboundaryIndices;
//
//		// Add surface nodes
//
//		for (int i = 0; i < (int)vcoarElemIdx.size(); ++i)
//		{
//			auto coarElemMeta = this->m_pCoarsenedMesh->Elems()[vcoarElemIdx[i]]->Trait<Mesh_GridCoarsen::CoarElemMeta>(Tag::Tag_CoarMeta);
//			for (int j = 0; j < (int)coarElemMeta.vfineNode.size(); ++j)
//			{
//				auto gridNodeMeta = coarElemMeta.vfineNode[j]->Trait<Mesh_GridCoarsen::GridNodeMeta>(Tag::Tag_GridMeta);
//				if (gridNodeMeta.IsSurface())
//					sboundaryIndices.insert(coarElemMeta.vfineNode[j]->ID());
//			}
//		}
//
//		// Add shared nodes
//
//		for (int i = 0; i < vcoarElemIdx.size(); ++i)
//		{
//			for (int j = 0; j < 6; ++j)
//			{
//				vector<Mesh_GridCoarsen::SharedNodes> vshared;
//				this->m_pCoarsenedMesh->GetSharedNodes(vcoarElemIdx[i], j, vshared);
//				for (int k = 0; k < (int)vshared.size(); ++k)
//				{
//					if (find(vcoarElemIdx.begin(), vcoarElemIdx.end(), vshared[k].coarse1) == vcoarElemIdx.end())
//						for (int l = 0; l < (int)vshared[k].vFineNodes.size(); ++l)
//							sboundaryIndices.insert(vshared[k].vFineNodes[l]->ID());
//				}
//			}
//		}
//
//		vselBoundary.reserve(sboundaryIndices.size());
//		for (auto it = sboundaryIndices.begin(); it != sboundaryIndices.end(); ++it)
//			vselBoundary.push_back(vmapFull2Sel[*it]);
//	}
//	inline Vector4i F(int p, Poly *pEle)
//	{
//		MatrixXi m(6, 4);
//		m << 0, 4, 7, 3,
//			1, 2, 6, 5,
//			0, 1, 5, 4,
//			2, 3, 7, 6,
//			0, 3, 2, 1,
//			4, 5, 6, 7;
//		Vector4i v;
//		for (int i = 0; i < 4; i++)
//		{
//			v(i) = pEle->Nodes()[m(p, i)]->ID();
//		}
//		return v;
//	}
//	void CoarsenedSimulationPro::ComputeSubmeshHessian(const iVector &vcoarElemIdx,
//													   iVector &vmapSel2Full,
//													   iVector &vmapFull2Sel,
//													   iVector &vselBoundary,
//													   MatrixXi &Faces,
//													   MatrixXd &Normals,
//													   MatrixSd &mH)
//	{
//		// Colect selected nodes and elements
//
//		set<int> sselectedNodes;
//		set<int> sselectedElems;
//		for (int i = 0; i < (int)vcoarElemIdx.size(); ++i)
//		{
//			auto coarElemMeta = this->m_pCoarsenedMesh->Elems()[vcoarElemIdx[i]]->Trait<Mesh_GridCoarsen::CoarElemMeta>(Tag::Tag_CoarMeta);
//			for (int j = 0; j < (int)coarElemMeta.vfineElem.size(); ++j)
//				sselectedElems.insert(coarElemMeta.vfineElem[j]->ID());
//			for (int j = 0; j < (int)coarElemMeta.vfineNode.size(); ++j)
//				sselectedNodes.insert(coarElemMeta.vfineNode[j]->ID());
//		}
//
//		int N = this->m_pCoarsenedMesh->FineMesh()->NumNodes();
//		int M = (int)sselectedNodes.size();
//
//		// Build (two-way) selection maps
//
//		int countSel = 0;
//		vmapSel2Full.reserve(M);
//		vmapFull2Sel.resize(N, -1);
//		for (auto it = sselectedNodes.begin(); it != sselectedNodes.end(); ++it)
//		{
//			vmapSel2Full.push_back(*it);
//			vmapFull2Sel[*it] = countSel++;
//		}
//
//		// Build selection matrix
//
//		MatrixSd mSelected;
//		Utils::buildSelectionMatrix(vmapSel2Full, N, mSelected);
//		Utils::buildBlockExpandedMatrix(mSelected, 3, mSelected);
//
//		// Build selected Hessian
//
//		iVector vselectedElems;
//		vselectedElems.insert(vselectedElems.begin(),
//							  sselectedElems.begin(),
//							  sselectedElems.end());
//		this->m_pFullFineModel->SetSelectedElements(vselectedElems);
//
//		AMatrixSd mFullHessian;
//		this->m_pFullFineModel->GetHessian(mFullHessian);
//		mH = mSelected * mFullHessian * mSelected.transpose();
//
//		this->m_pFullFineModel->SetSelectedElements(iVector());
//
//		// Find submesh boundary
//
//		set<int> sboundaryIndices;
//
//		// Add surface nodes
//
//		for (int i = 0; i < (int)vcoarElemIdx.size(); ++i)
//		{
//			auto coarElemMeta = this->m_pCoarsenedMesh->Elems()[vcoarElemIdx[i]]->Trait<Mesh_GridCoarsen::CoarElemMeta>(Tag::Tag_CoarMeta);
//			for (int j = 0; j < (int)coarElemMeta.vfineNode.size(); ++j)
//			{
//				auto gridNodeMeta = coarElemMeta.vfineNode[j]->Trait<Mesh_GridCoarsen::GridNodeMeta>(Tag::Tag_GridMeta);
//				if (gridNodeMeta.IsSurface())
//					sboundaryIndices.insert(coarElemMeta.vfineNode[j]->ID());
//			}
//		}
//
//		// Add shared nodes
//
//		for (int i = 0; i < vcoarElemIdx.size(); ++i)
//		{
//			for (int j = 0; j < 6; ++j)
//			{
//				vector<Mesh_GridCoarsen::SharedNodes> vshared;
//				this->m_pCoarsenedMesh->GetSharedNodes(vcoarElemIdx[i], j, vshared);
//				for (int k = 0; k < (int)vshared.size(); ++k)
//				{
//					if (find(vcoarElemIdx.begin(), vcoarElemIdx.end(), vshared[k].coarse1) == vcoarElemIdx.end())
//						for (int l = 0; l < (int)vshared[k].vFineNodes.size(); ++l)
//							sboundaryIndices.insert(vshared[k].vFineNodes[l]->ID());
//				}
//			}
//		}
//
//		vselBoundary.reserve(sboundaryIndices.size());
//		for (auto it = sboundaryIndices.begin(); it != sboundaryIndices.end(); ++it)
//			vselBoundary.push_back(vmapFull2Sel[*it]);
//
//		//set quads and normals
//		vector<Vector4i> vF;
//		vector<Vector3d> vN;
//		Vector3d Nxm(-1, 0, 0), Nxp(1, 0, 0), Nym(0, -1, 0), Nyp(0, 1, 0), Nzm(0, 0, -1), Nzp(0, 0, 1);
//
//		for (int i : sselectedElems)
//		{
//			auto pElem = this->GetFineFullMesh()->Elems()[i];
//			auto gridElemMeta = pElem
//									->Trait<Mesh_GridCoarsen::GridElemMeta>(Tag::Tag_GridMeta);
//			if (gridElemMeta.vDualEdgesXm.size() == 0)
//			{
//				vF.push_back(F(0, pElem));
//				vN.push_back(Nxm);
//			}
//			else
//			{
//				auto elem0 = gridElemMeta.vDualEdgesXm[0]->elem0->ID();
//				auto elem1 = gridElemMeta.vDualEdgesXm[0]->elem1->ID();
//				if ((i == elem0) & (sselectedElems.find(elem1) == sselectedElems.end()))
//				{
//					vF.push_back(F(0, pElem));
//					vN.push_back(Nxm);
//				}
//				else if ((i == elem1) & (sselectedElems.find(elem0) == sselectedElems.end()))
//				{
//					vF.push_back(F(0, pElem));
//					vN.push_back(Nxm);
//				};
//			};
//			if (gridElemMeta.vDualEdgesXp.size() == 0)
//			{
//				vF.push_back(F(1, pElem));
//				vN.push_back(Nxp);
//			}
//			else
//			{
//				auto elem0 = gridElemMeta.vDualEdgesXp[0]->elem0->ID();
//				auto elem1 = gridElemMeta.vDualEdgesXp[0]->elem1->ID();
//				if ((i == elem0) & (sselectedElems.find(elem1) == sselectedElems.end()))
//				{
//					vF.push_back(F(1, pElem));
//					vN.push_back(Nxp);
//				}
//				else if ((i == elem1) & (sselectedElems.find(elem0) == sselectedElems.end()))
//				{
//					vF.push_back(F(1, pElem));
//					vN.push_back(Nxp);
//				};
//			};
//			if (gridElemMeta.vDualEdgesYm.size() == 0)
//			{
//				vF.push_back(F(2, pElem));
//				vN.push_back(Nym);
//			}
//			else
//			{
//				auto elem0 = gridElemMeta.vDualEdgesYm[0]->elem0->ID();
//				auto elem1 = gridElemMeta.vDualEdgesYm[0]->elem1->ID();
//				if ((i == elem0) & (sselectedElems.find(elem1) == sselectedElems.end()))
//				{
//					vF.push_back(F(2, pElem));
//					vN.push_back(Nym);
//				}
//				else if ((i == elem1) & (sselectedElems.find(elem0) == sselectedElems.end()))
//				{
//					vF.push_back(F(2, pElem));
//					vN.push_back(Nym);
//				};
//			};
//			if (gridElemMeta.vDualEdgesYp.size() == 0)
//			{
//				vF.push_back(F(3, pElem));
//				vN.push_back(Nyp);
//			}
//			else
//			{
//				auto elem0 = gridElemMeta.vDualEdgesYp[0]->elem0->ID();
//				auto elem1 = gridElemMeta.vDualEdgesYp[0]->elem1->ID();
//				if ((i == elem0) & (sselectedElems.find(elem1) == sselectedElems.end()))
//				{
//					vF.push_back(F(3, pElem));
//					vN.push_back(Nyp);
//				}
//				else if ((i == elem1) & (sselectedElems.find(elem0) == sselectedElems.end()))
//				{
//					vF.push_back(F(3, pElem));
//					vN.push_back(Nyp);
//				};
//			};
//			if (gridElemMeta.vDualEdgesZm.size() == 0)
//			{
//				vF.push_back(F(4, pElem));
//				vN.push_back(Nzm);
//			}
//			else
//			{
//				auto elem0 = gridElemMeta.vDualEdgesZm[0]->elem0->ID();
//				auto elem1 = gridElemMeta.vDualEdgesZm[0]->elem1->ID();
//				if ((i == elem0) & (sselectedElems.find(elem1) == sselectedElems.end()))
//				{
//					vF.push_back(F(4, pElem));
//					vN.push_back(Nzm);
//				}
//				else if ((i == elem1) & (sselectedElems.find(elem0) == sselectedElems.end()))
//				{
//					vF.push_back(F(4, pElem));
//					vN.push_back(Nzm);
//				};
//			};
//			if (gridElemMeta.vDualEdgesZp.size() == 0)
//			{
//				vF.push_back(F(5, pElem));
//				vN.push_back(Nzp);
//			}
//			else
//			{
//				auto elem0 = gridElemMeta.vDualEdgesZp[0]->elem0->ID();
//				auto elem1 = gridElemMeta.vDualEdgesZp[0]->elem1->ID();
//				if ((i == elem0) & (sselectedElems.find(elem1) == sselectedElems.end()))
//				{
//					vF.push_back(F(5, pElem));
//					vN.push_back(Nzp);
//				}
//				else if ((i == elem1) & (sselectedElems.find(elem0) == sselectedElems.end()))
//				{
//					vF.push_back(F(5, pElem));
//					vN.push_back(Nzp);
//				};
//			};
//		}
//		Faces.resize(vF.size(), 4);
//		Normals.resize(vN.size(), 3);
//		for (Eigen::Index i = 0; i < Faces.rows(); i++)
//		{	Vector4i v;
//			for(int j=0; j<4;j++)
//				v(j)=vmapFull2Sel[vF[i](j)];
//			Faces.row(i) = v;
//			Normals.row(i) = vN[i];
//		}
//	}
//
//	PtrS<CoarsenedSimulationPro> CoarsenedSimulationPro::CreateSubCuboid(const Vector3d &vsizes, Vector2d ranX, Vector2d ranY, Vector2d ranZ, int coarseElements, int coarseningLevel, const Material &mat)
//	{
//		int coarDimX = coarseElements;
//		int coarDimY = coarseElements;
//		int coarDimZ = coarseElements;
//		int fineDimX = coarseElements * coarseningLevel;
//		int fineDimY = coarseElements * coarseningLevel;
//		int fineDimZ = coarseElements * coarseningLevel;
//
//		double fineElemSizeX = vsizes.x() / fineDimX;
//		double fineElemSizeY = vsizes.y() / fineDimY;
//		double fineElemSizeZ = vsizes.z() / fineDimZ;
//
//		// Set occupancy for a sphere of radius
//
//		int countFine = 0;
//		vector<Voxel> vvoxs;
//		for (int i = 0; i < fineDimX; ++i)
//			for (int j = 0; j < fineDimY; ++j)
//				for (int k = 0; k < fineDimZ; ++k)
//				{
//					double centerX = -vsizes.x() / 2 + fineElemSizeX / 2 + i * fineElemSizeX;
//					double centerY = -vsizes.y() / 2 + fineElemSizeY / 2 + j * fineElemSizeY;
//					double centerZ = -vsizes.z() / 2 + fineElemSizeZ / 2 + k * fineElemSizeZ;
//					if (centerX >= ranX.x() && centerX <= ranX.y() &&
//						centerY >= ranY.x() && centerY <= ranY.y() &&
//						centerZ >= ranZ.x() && centerZ <= ranZ.y())
//					{
//						vvoxs.push_back(Voxel());
//						vvoxs.back().MatIdx = 0;
//						vvoxs.back().VoxIdx = countFine;
//					}
//					countFine++;
//				}
//
//		vector<Material> vmats;
//		vmats.push_back(mat);
//		PtrS<CoarsenedSimulationPro> pCoarSim(new CoarsenedSimulationPro());
//		pCoarSim->QuadraturePerDimension = coarseningLevel;
//		pCoarSim->Init(-vsizes*0.5,
//					   vsizes,
//					   Vector3i(fineDimX, fineDimY, fineDimZ),
//					   Vector3i(coarDimX, coarDimY, coarDimZ),
//					   vvoxs, vmats);
//
//		return pCoarSim;
//	}
//
//	PtrS<CoarsenedSimulationPro> CoarsenedSimulationPro::CreateFullCuboid(const Vector3d &vsizes, Vector3i coarseElements, int coarseningLevel, const Material &mat)
//	{
//		int coarDimX = coarseElements(0);
//		int coarDimY = coarseElements(1);
//		int coarDimZ = coarseElements(2);
//		int fineDimX = coarseElements(0) * coarseningLevel;
//		int fineDimY = coarseElements(1) * coarseningLevel;
//		int fineDimZ = coarseElements(2) * coarseningLevel;
//
//		double fineElemSizeX = vsizes.x() / fineDimX;
//		double fineElemSizeY = vsizes.y() / fineDimY;
//		double fineElemSizeZ = vsizes.z() / fineDimZ;
//
//		int countFine = 0;
//		vector<Voxel> vvoxs;
//		for (int i = 0; i < fineDimX; ++i)
//			for (int j = 0; j < fineDimY; ++j)
//				for (int k = 0; k < fineDimZ; ++k)
//				{
//					vvoxs.push_back(Voxel());
//					vvoxs.back().MatIdx = 0;
//					vvoxs.back().VoxIdx = countFine++;
//				}
//
//		vector<Material> vmats;
//		vmats.push_back(mat);
//		PtrS<CoarsenedSimulationPro> pCoarSim(new CoarsenedSimulationPro());
//		pCoarSim->QuadraturePerDimension = coarseningLevel;
//		pCoarSim->Init(-vsizes*0.5,
//					   vsizes,
//					   Vector3i(fineDimX, fineDimY, fineDimZ),
//					   Vector3i(coarDimX, coarDimY, coarDimZ),
//					   vvoxs, vmats);
//
//		return pCoarSim;
//	}
//	PtrS<CoarsenedSimulationPro> CoarsenedSimulationPro::CreateFullCuboid(const Vector3d &vsizes, int coarseElements, int coarseningLevel, const Material &mat)
//	{
//		int coarDimX = coarseElements;
//		int coarDimY = coarseElements;
//		int coarDimZ = coarseElements;
//		int fineDimX = coarseElements * coarseningLevel;
//		int fineDimY = coarseElements * coarseningLevel;
//		int fineDimZ = coarseElements * coarseningLevel;
//
//		double fineElemSizeX = vsizes.x() / fineDimX;
//		double fineElemSizeY = vsizes.y() / fineDimY;
//		double fineElemSizeZ = vsizes.z() / fineDimZ;
//
//		int countFine = 0;
//		vector<Voxel> vvoxs;
//		for (int i = 0; i < fineDimX; ++i)
//			for (int j = 0; j < fineDimY; ++j)
//				for (int k = 0; k < fineDimZ; ++k)
//				{
//					vvoxs.push_back(Voxel());
//					vvoxs.back().MatIdx = 0;
//					vvoxs.back().VoxIdx = countFine++;
//				}
//
//		vector<Material> vmats;
//		vmats.push_back(mat);
//		PtrS<CoarsenedSimulationPro> pCoarSim(new CoarsenedSimulationPro());
//		pCoarSim->QuadraturePerDimension = coarseningLevel;
//		pCoarSim->Init(-vsizes*0.5,
//					   vsizes,
//					   Vector3i(fineDimX, fineDimY, fineDimZ),
//					   Vector3i(coarDimX, coarDimY, coarDimZ),
//					   vvoxs, vmats);
//
//		return pCoarSim;
//	}
//	PtrS<CoarsenedSimulationPro> CoarsenedSimulationPro::CreateSphere(const Vector3d &center, double radius, int coarseElements, int coarseningLevel, const Material &mat)
//	{
//		int coarDimX = coarseElements;
//		int coarDimY = coarseElements;
//		int coarDimZ = coarseElements;
//		int fineDimX = coarseElements * coarseningLevel;
//		int fineDimY = coarseElements * coarseningLevel;
//		int fineDimZ = coarseElements * coarseningLevel;
//
//		double sizeX = 2.001 * radius;
//		double sizeY = 2.001 * radius;
//		double sizeZ = 2.001 * radius;
//		double fineElemSizeX = sizeX / fineDimX;
//		double fineElemSizeY = sizeY / fineDimY;
//		double fineElemSizeZ = sizeZ / fineDimZ;
//
//		int countFine = 0;
//		vector<Voxel> vvoxs;
//		for (int i = 0; i < fineDimX; ++i)
//			for (int j = 0; j < fineDimY; ++j)
//				for (int k = 0; k < fineDimZ; ++k)
//				{
//					double centerX = -sizeX / 2 + fineElemSizeX / 2 + i * fineElemSizeX;
//					double centerY = -sizeY / 2 + fineElemSizeY / 2 + j * fineElemSizeY;
//					double centerZ = -sizeZ / 2 + fineElemSizeZ / 2 + k * fineElemSizeZ;
//					if (centerX * centerX + centerY * centerY + centerZ * centerZ <= radius)
//					{
//						vvoxs.push_back(Voxel());
//						vvoxs.back().MatIdx = 0;
//						vvoxs.back().VoxIdx = countFine;
//					}
//					countFine++;
//				}
//
//		vector<Material> vmats;
//		vmats.push_back(mat);
//		PtrS<CoarsenedSimulationPro> pCoarSim(new CoarsenedSimulationPro());
//		pCoarSim->QuadraturePerDimension = coarseningLevel;
//		pCoarSim->Init(-Vector3d(sizeX, sizeY, sizeZ)*0.5,
//					   Vector3d(sizeX, sizeY, sizeZ),
//					   Vector3i(fineDimX, fineDimY, fineDimZ),
//					   Vector3i(coarDimX, coarDimY, coarDimZ),
//					   vvoxs, vmats);
//
//		return pCoarSim;
//	}
//
//	PtrS<CoarsenedSimulationPro> CoarsenedSimulationPro::CreateVolume(const ScalarField_Grid<int>& volume, const vector<Material>& vmats, int coarseElements, int coarseningLevel, Real scale)
//	{
//		// Compute tentative volume
//
//		int occupiedCount = 0;
//		int ignoredCount = 0;
//		Vector3d vminCor = Vector3d::Ones()*HUGE_VAL;
//		Vector3d vmaxCor = -Vector3d::Ones()*HUGE_VAL;
//		const Vector3i& vshape = volume.Shape();
//		for (int i = 0; i < vshape.x(); ++i)
//		for (int j = 0; j < vshape.y(); ++j)
//		for (int k = 0; k < vshape.z(); ++k)
//		{
//			Vector3i coords = Vector3i(i, j, k);
//			int maskValue = volume.At(coords);
//			if (maskValue != 0)
//			{
//				occupiedCount++;
//				Vector3d center = volume.PointOf(coords);
//				Vector3d vpm = center - (0.5 + 1e-6)*volume.Delta();
//				Vector3d vpp = center + (0.5 + 1e-6)*volume.Delta();
//				if (vpm.x() < vminCor.x()) vminCor.x() = vpm.x();
//				if (vpm.y() < vminCor.y()) vminCor.y() = vpm.y();
//				if (vpm.z() < vminCor.z()) vminCor.z() = vpm.z();
//				if (vpp.x() > vmaxCor.x()) vmaxCor.x() = vpp.x();
//				if (vpp.y() > vmaxCor.y()) vmaxCor.y() = vpp.y();
//				if (vpp.z() > vmaxCor.z()) vmaxCor.z() = vpp.z();
//			}
//			else
//			{
//				ignoredCount++;
//			}
//		}
//
//		Vector3d vsize = vmaxCor - vminCor;
//		Vector3d vcenter = vminCor + vsize*0.5;
//
//		// Compute coarse discretization
//
//		Real minSize = vsize.minCoeff();
//		Real coarDelta = minSize / coarseElements;
//		int dimX = ceil(vsize.x() / (coarDelta + 1e-6));
//		int dimY = ceil(vsize.y() / (coarDelta + 1e-6));
//		int dimZ = ceil(vsize.z() / (coarDelta + 1e-6));
//		Vector3i vcoarDim(dimX, dimY, dimZ);
//
//		// Compute new origin point
//
//		Vector3d vnewSize = vcoarDim.cast<Real>()*coarDelta;
//		Vector3d vnewOrigin = vcenter - vnewSize * 0.5;
//
//
//		// Compute fine dimensions and delta
//
//		Vector3i vfineDim = vcoarDim * coarseningLevel;
//		Real fineDelta = coarDelta / coarseningLevel;
//
//		int countFine = 0;
//		vector<Voxel> vvoxs;
//		for (int i = 0; i < vfineDim.x(); ++i)
//		for (int j = 0; j < vfineDim.y(); ++j)
//		for (int k = 0; k < vfineDim.z(); ++k)
//		{
//			Vector3d vcentroid = vnewOrigin + Vector3d(fineDelta*(i + 0.5),
//													   fineDelta*(j + 0.5),
//													   fineDelta*(k + 0.5));
//
//			int value = volume.Sample(vcentroid);
//			if (value != 0)
//			{
//				vvoxs.push_back(Voxel());
//				vvoxs.back().MatIdx = value - 1;
//				vvoxs.back().VoxIdx = countFine;
//			}
//			countFine++;
//		}
//
//		// Create coarsened simulation instance
//
//		PtrS<CoarsenedSimulationPro> pCoarSim(new CoarsenedSimulationPro());
//
//		pCoarSim->Init(vnewOrigin*scale,
//					   vnewSize*scale,
//					   vfineDim,
//					   vcoarDim,
//					   vvoxs, vmats);
//
//		// Number of quadrature points of the densest dimension
//		pCoarSim->QuadraturePerDimension = coarseningLevel;
//
//		return pCoarSim;
//	}
//
//}; // namespace PhySim
