#include <PhySim/CommonIncludes.h>
#include <PhySim/PhySimInterface.h>
#include <PhySim/CommonIncludes.h>
#include <PhySim/PhySimInterface.h>
#include <PhySim/Physics/DoFSet.h>
#include <PhySim/Physics/Boundary/BC_Gravity.h>
#include <PhySim/Physics/Boundary/BC_FixDoF.h>
#include <PhySim/Physics/Simulables/Simulable_Composite.h>
#include <PhySim/Physics/Simulables/Simulable_FEM_Volume.h>
#include <PhySim/Utils/Serializer_Mesh.h>
#include <PhySim/Geometry/Meshes/Mesh_Tetra.h>
#include <PhySim/Geometry/Meshes/Mesh_Tri.h>
#include <PhySim/Physics/DomainDistribution.h>
#include <PhySim/Geometry/Polytopes/ShapeFunctions.h>
#include <PhySim/Geometry/Polytopes/Quadratures.h>
#include <PhySim/Solvers/OptimProblem_BasicStatic.h>
#include <PhySim/Solvers/OptimSolver_USQP_LS.h>
#include <PhySim/PhySics/SpatialDistribution.h>
#include <PhySim/Utils/Serializer_Mesh.h>
#include <PhySim/Geometry/Samplers/SurfaceSampler_Poisson.h>
#include <PhySim/Physics/Colliders/Collider_SphereCloud.h>
#include <PhySim/Physics/Boundary/BC_Collisions.h>
#include <PhySim/Utils/Serializer_Npy.h>
using namespace PhySim;
using namespace std;

void SaveToFile(PtrS<Collider_SphereCloud> pColliderA, PtrS<Collider_SphereCloud> pColliderB, PtrS<BC_Collisions> pCollisionsBC, const string& prefix, const string& postfix)
{
    size_t NA = pColliderA->Points().size();
    size_t NB = pColliderB->Points().size();
    MatrixXd mPointsA(NA, 4);
    MatrixXd mPointsB(NB, 4);

    for (size_t i = 0; i < NA; ++i)
    {
        Vector3d P = pColliderA->Points()[i].Position->InterpolateValue(Tag_Position_X);
        double R = pColliderA->Points()[i].Radius;

        mPointsA.block(i, 0, 1, 3) = P.transpose();
        mPointsA(i, 3) = R;
    }

    for (size_t i = 0; i < NB; ++i)
    {
        Vector3d P = pColliderB->Points()[i].Position->InterpolateValue(Tag_Position_X);
        double R = pColliderB->Points()[i].Radius;

        mPointsB.block(i, 0, 1, 3) = P.transpose();
        mPointsB(i, 3) = R;
    }

    Serializer_Npy::WriteMatrix(prefix + "PointsA" + postfix + ".npy", mPointsA);
    Serializer_Npy::WriteMatrix(prefix + "PointsB" + postfix + ".npy", mPointsB);
}

int main(int argc, char argv)
{
    //Logger::Instance().verbosity = V4_DeepShit;

    // Load example cube and generate tetrahedralization.
    MatrixXd mV;
    MatrixXi mF;
    Serializer_Mesh::ReadSurface_OBJ("examples/Simulator_CompositeWithCollisions/assets/Object.obj", mV, mF);

    MatrixXd mVtet;
    MatrixXi mFtet, mTtet;
    Utils::tetrahedralize(mV, mF, {}, mVtet, mFtet, mTtet);

    // Allocate vertices for both objects in composite simulable.
    MatrixXd mVtetA = mVtet.rowwise() + RowVector3d(-0.6, 1.1, 0.0);
    MatrixXd mVtetB = mVtet.rowwise() + RowVector3d(0.6, -1.1, 0.0);

    cout << "Simulable A:" << endl;
    cout << "Min BBox = " << mVtetA.colwise().minCoeff() << endl;
    cout << "Max BBox = " << mVtetA.colwise().maxCoeff() << endl;
    cout << endl;
    cout << "Simulable B:" << endl;
    cout << "Min BBox = " << mVtetB.colwise().minCoeff() << endl;
    cout << "Max BBox = " << mVtetB.colwise().maxCoeff() << endl;
    cout << endl;

    // Perform sampling over both surfaces. 
    SurfaceSampler_Poisson samplerA;
    SurfaceSampler_Poisson samplerB;
    vector<SurfaceSample> vSamplesA;
    vector<SurfaceSample> vSamplesB;
    samplerA.Setup(mVtetA, mFtet);
    samplerB.Setup(mVtetB, mFtet);
    samplerA.Sample(size_t(40 * samplerA.TotalArea()), vSamplesA); // 40 points per meter squared
    samplerB.Sample(size_t(40 * samplerB.TotalArea()), vSamplesB); // 40 points per meter squared
    cout << "Generated " << vSamplesA.size() << " samples for object A." << endl;
    cout << "Generated " << vSamplesB.size() << " samples for object B." << endl;

    // Generate meshes.
    PtrS<Mesh_Tetra> pMeshA = NewS<Mesh_Tetra>(mVtetA, mTtet, Discretization::Discretization_Tet4);
    pMeshA->SetNodesTrait(mVtetA, Tag_Position_0);
    pMeshA->SetNodesTrait(mVtetA, Tag_Position_X);
    pMeshA->SetNodesTrait(MatrixXd::Zero(mVtetA.rows(), mVtetA.cols()), Tag_Velocity);
    pMeshA->Traits().AddTrait(Tag_SurfaceMesh_0, mFtet);

    PtrS<Mesh_Tetra> pMeshB = NewS<Mesh_Tetra>(mVtetB, mTtet, Discretization::Discretization_Tet4);
    pMeshB->SetNodesTrait(mVtetB, Tag_Position_0);
    pMeshB->SetNodesTrait(mVtetB, Tag_Position_X);
    pMeshB->SetNodesTrait(MatrixXd::Zero(mVtetB.rows(), mVtetB.cols()), Tag_Velocity);
    pMeshB->Traits().AddTrait(Tag_SurfaceMesh_0, mFtet);

    // Initialize each model.
    // Use linear elements with STVK material model.
    PtrS<Simulable_FEM_Volume> pModelFEMA = NewS<Simulable_FEM_Volume>();
    PtrS<Simulable_FEM_Volume> pModelFEMB = NewS<Simulable_FEM_Volume>();

    PtrS<ParameterSet> pMaterialParams = NewS<ParameterSet>();
    PtrS<IMaterialModel> pMaterialModel = MaterialModel_3Din3D_StVK::Instance();
    PtrS<IShapeFunction> pShapeFunction = ShapeFunction_Tet4::Instance();
    PtrS<IQuadrature> pQuadraturePoints = Quadrature_Tet1::Instance();
    pMaterialParams->InitFromYoungPoisson(1e1, 0.3, 1e1);

    auto& pOptionsA = pModelFEMA->SetupOptions();
    auto& pOptionsB = pModelFEMB->SetupOptions();
    
    pOptionsA.m_pMatParams = NewS<DomainDistribution_Constant<PtrS<ParameterSet>>>(pMaterialParams);
    pOptionsA.m_pMatModels = NewS<DomainDistribution_Constant<PtrS<IMaterialModel>>>(pMaterialModel);
    pOptionsA.m_pShapeFunctions = NewS<DomainDistribution_Constant<PtrS<IShapeFunction>>>(pShapeFunction);
    pOptionsA.m_pQuadratures = NewS<DomainDistribution_Constant<PtrS<IQuadrature>>>(pQuadraturePoints);

    pOptionsB = pOptionsA;
    pOptionsA.m_pMesh = pMeshA;
    pOptionsB.m_pMesh = pMeshB;
    
    // Initialize composite model.
    PtrS<Simulable_Composite> pModel = NewS<Simulable_Composite>();
    pModel->SetupOptions().m_vpSimulables = { pModelFEMA, pModelFEMB };
    pModel->Setup();

    // Initialize colliders.
    PtrS<Collider_SphereCloud> pColliderA = NewS<Collider_SphereCloud>();
    PtrS<Collider_SphereCloud> pColliderB = NewS<Collider_SphereCloud>();
    pColliderA->InitFromSurface(pModelFEMA.get(), vSamplesA);
    pColliderB->InitFromSurface(pModelFEMB.get(), vSamplesB);
    vector<PtrS<Collider>> vpColliders = { pColliderA, pColliderB };
    
    // Initialize collision boundary condition.
    PtrS<BC_Collisions> pCollisionsBC = NewS<BC_Collisions>(dynamic_cast<Simulable*>(pModel.get()), vpColliders);
    pCollisionsBC->DisableCollision(pColliderA, pColliderA);    // Disable self-collisions.
    pCollisionsBC->DisableCollision(pColliderB, pColliderB);
    pCollisionsBC->IsContinuouslyLoaded() = true;
    pModel->AddBoundaryCondition(pCollisionsBC); 
    
    // Initialize fixed boundary conditions.
    GeometryFilter_PlaneDistNode filterA(Tag_Position_X, Vector3d(-1.4, 0, 0.0), Vector3d(-1.0, 0.0, 0.0), Vector2d(0.0, numeric_limits<Real>::infinity()));
    GeometryFilter_PlaneDistNode filterB(Tag_Position_X, Vector3d(1.4, 0, 0.0), Vector3d(1.0, 0.0, 0.0), Vector2d(0.0, numeric_limits<Real>::infinity()));
    vector<IGeometry*> vpFilteredA = filterA.Filter(pModelFEMA->Geometries()[0]);
    vector<IGeometry*> vpFilteredB = filterB.Filter(pModelFEMB->Geometries()[0]);

    vector<IDoFSet*> vFixedDoFsA;
    vector<IDoFSet*> vFixedDoFsB;
    vector<VectorXd> vFixedValueA;
    vector<VectorXd> vFixedValueB;

    vFixedDoFsA.reserve(vpFilteredA.size());
    vFixedDoFsB.reserve(vpFilteredB.size());
    vFixedValueA.reserve(vpFilteredA.size());
    vFixedValueB.reserve(vpFilteredB.size());

    for (auto pGeom : vpFilteredA)
    {
        Node* pNode = dynamic_cast<Node*>(pGeom);
        vFixedDoFsA.push_back(pNode->Traits().DoFSet(Tag_DOF_0));
        vFixedValueA.push_back(pNode->Traits().Vector3d(Tag_Position_X));
    }

    for (auto pGeom : vpFilteredB)
    {
        Node* pNode = dynamic_cast<Node*>(pGeom);
        vFixedDoFsB.push_back(pNode->Traits().DoFSet(Tag_DOF_0));
        vFixedValueB.push_back(pNode->Traits().Vector3d(Tag_Position_X));
    }

    Affine3d aTransformA;
    Affine3d aTransformB;
    aTransformA.setIdentity();
    aTransformB.setIdentity();
    aTransformA.translate(Vector3d(0.0, -0.5, 0.0));
    aTransformB.translate(Vector3d(0.0, 0.5, 0.0));

    vector<Matrix4d> vTransformA = { aTransformA.matrix() };
    vector<Matrix4d> vTransformB = { aTransformB.matrix() };
    vector<int> vTransformStepsA = { 100 };
    vector<int> vTransformStepsB = { 100 };

    PtrS<BC_FixDoF> pFixedBCA = NewS<BC_FixDoF>(pModelFEMA.get(), vFixedDoFsA, vFixedValueA);
    PtrS<BC_FixDoF> pFixedBCB = NewS<BC_FixDoF>(pModelFEMB.get(), vFixedDoFsB, vFixedValueB);
    
    pFixedBCA->NumStage() = vTransformStepsA[0];
    pFixedBCB->NumStage() = vTransformStepsB[0];
    pFixedBCA->m_pAnimator = NewS<VectorAnimator_SequenceT3D>(vTransformA, vTransformStepsA, vFixedValueA);
    pFixedBCB->m_pAnimator = NewS<VectorAnimator_SequenceT3D>(vTransformB, vTransformStepsB, vFixedValueB);
    pModelFEMA->AddBoundaryCondition(pFixedBCA);
    pModelFEMB->AddBoundaryCondition(pFixedBCB);

    // TEST: Export to python file.
    SaveToFile(pColliderA, pColliderB, pCollisionsBC, "D:/Test/", "_0");

    // Numerical problem

    PtrS<OptimProblem> pOptimProblem = NewS<OptimProblem_BasicStatic>(pModel.get());

    // Numerical solver

    OptimSolverOptions options;
    options.maxIters = 100; // Maximum iterations
    options.maxError = 1e-3; // The solver tolerance
    options.lsSolverType = LSSolverType::LS_EigenCG; // CGSolver solver
    options.lsMaxIters = 500;
    options.lsMaxError = 1e-6;
    options.lSearch_type = LSearchType::LSearch_Simple; // Simple line-search

    PtrS<OptimSolver> pOptimSolver = NewS<OptimSolver_USQP_LS>(pOptimProblem.get(), options);

    // Main Loop!
    // Perform simulation. Keep going until the objective function is minimized
    // (or an error is found).

    OSResult res = OR_ONGOING;
    size_t frame = 0;

    while (res != OR_SUCCESS && res != OR_FAILURE && res != OR_NONDESC)
    {
        cout << "Simulating 10 step... ";
        pOptimSolver->Options().maxIters = 10;
        res = pOptimSolver->SolveFull().m_result;
        ++frame;

        // TEST: Export to python file.
        //SaveToFile(pColliderA, pColliderB, pCollisionsBC, "D:/Test/", "_" + to_string(frame));

        switch (res)
        {
        case OR_SUCCESS: cout << "Success!" << endl; break;
        case OR_FAILURE: cout << "Failed!" << endl; break;
        case OR_MAXITER: cout << "Maximum iterations reached!" << endl; break;
        case OR_NONDESC: cout << "Non-descendent step found!" << endl; break;
        default: cout << "Invalid error code obtained!" << endl;
        }
    }
    
    return 0;
}
