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

#include <PhySim/Utils/CustomTimer.h>

#include <PhySim/Utils/ParameterSet.h>

#include <PhySim/Kinematics/KinematicsTree.h>

#include <PhySim/Physics/Boundary/IBCondition.h>
#include <PhySim/Physics/Elements/IConstraintSet.h>
#include <PhySim/Physics/Elements/IEnergyElement.h>
#include <PhySim/Physics/Elements/IMassElement.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class ISimulable {
 public:
  ISimulable() = default;
  virtual ~ISimulable() = default;

  // Initialize

  virtual void Setup() = 0;

  // Description

  virtual int ID() const = 0;
  virtual string GetName() const = 0;

  // Geometry

  virtual vector<PtrS<Geometry>> Geometries() const = 0;

  // Kinematic DoF number

  virtual int GetNumFreeDOF() const = 0;
  virtual int GetNumFixedDOF() const = 0;
  virtual int GetNumFullDOF() const = 0;

  // Kinematic DoF get/set

  virtual void GetDOFVector(VectorXd& vx, Tag s) const = 0;
  virtual void SetDOFVector(const VectorXd& vx, Tag s) = 0;

  // Kinematic State get/set

  virtual void GetState(VectorXd& vs) const = 0;
  virtual void SetState(const VectorXd& vs) = 0;

  // KinematicsEle

  virtual void UpdateKinematics() = 0;

  // Mechanics

  virtual void UpdateMechanics() = 0;

  virtual const VectorXd& GetConstraint() = 0;
  virtual const MatrixSd& GetJacobian() = 0;

  virtual void GetEnergy(Real& energy, bool addBC = true) = 0;
  virtual void AddEnergy(Real& energy, bool addBC = true) = 0;

  virtual void GetGradient(AVectorXd& vgradient, bool addBC = true) = 0;
  virtual void AddGradient(AVectorXd& vgradient, bool addBC = true) = 0;

  virtual void GetHessian(AMatrixSd& mHessian, bool addBC = true) = 0;
  virtual void AddHessian(AMatrixSd& mHessian, bool addBC = true) = 0;

  virtual void GetInertia(AMatrixSd& mInertia) = 0;
  virtual void AddInertia(AMatrixSd& mInertia) = 0;

  virtual void GetDMvDt(AVectorXd& mDMvDt) = 0;
  virtual void AddDMvDt(AVectorXd& mDMvDt) = 0;

  // DoF free/fixed

  virtual const MatrixSd& GetFreeSelection() = 0;
  virtual const MatrixSd& GetFreePermutation() = 0;
  virtual void FixVectorDoFs(VectorXd& vv) = 0;
  virtual void FixMatrixDoFs(MatrixSd& mM) = 0;

  // Boundary conditions

  virtual bool BoundaryConditionsLoaded() = 0;
  virtual void ResetBoundaryConditions() = 0;
  virtual void StepBoundaryConditions() = 0;
  virtual void FullBoundaryConditions() = 0;
};

class KinematicsEle;
class KinematicsMap;
class KinematicsTree;
class IConstraintSet;
class IEnergyElement;
class IMassElement;
class IBCondition;

/**
 * Simulable (Abstract).
 *
 * Base abstract class that can be inherited from for model implementation.
 * It provides the basic functionality regarding energy, gradient and Hessian
 * management, DoF get/set, state serialization, global derivative testing and
 * profiling.
 *
 * Any class deriving from Simulable must implement the method InitInternal().
 */
class Simulable : public ISimulable {
 private:
  static int IDCount;

 protected:
  int m_ID;

  PtrS<KinematicsTree> m_pTree;

  int m_numFreeDoF;
  int m_numFixedDoF;
  MatrixSd m_mFreeSele;
  MatrixSd m_mFreePerm;
  iVector m_vFixIndices;
  bVector m_vFixStencil;

  DirtyFlags m_dirtyFlags;

  bool m_isProfiling;
  vector<CustomTimer> m_vpTimers;
  CustomTimer m_timerComputeKine;
  CustomTimer m_timerComputeMech;
  CustomTimer m_timerComputeEner;
  CustomTimer m_timerComputeGrad;
  CustomTimer m_timerComputeHess;
  CustomTimer m_timerComputeCons;
  CustomTimer m_timerComputeJaco;
  CustomTimer m_timerAssembleEner;
  CustomTimer m_timerAssembleGrad;
  CustomTimer m_timerAssembleHess;
  CustomTimer m_timerAssembleCons;
  CustomTimer m_timerAssembleJaco;
  CustomTimer m_timerPropagaGrad;
  CustomTimer m_timerPropagaHess;

  vector<PtrS<IMassElement>> m_vmassEle;
  vector<PtrS<IEnergyElement>> m_venerEle;
  vector<PtrS<IConstraintSet>> m_vconsSet;

  vector<PtrS<IBCondition>> m_vBC;

  // vector<PtrS<IModelParameter>> m_vpModelParam
  // vector<PtrS<ParameterEnergy>> m_vpParamElements;
  // vector<PtrS<Parameter>> m_vpParamDoFSet;

  iVector m_vselEles;

 public:
  /**
   * Constructor.
   */
  Simulable();

  /**
   * Destructor.
   */
  virtual ~Simulable();

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // Metadata
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  /**
   * Returns the name of the simulation model for logging.
   */
  virtual string GetName() const override { return "Base Simulable"; }

  virtual int ID() const override { return this->m_ID; }

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // Initialization
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  /**
   * This method frees the current allocated memory and then
   * calls InitInternal. Any class deriving from Simulable
   * should implement that method.
   */
  virtual void Setup();

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // Profiling
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  /**
   * Get if performance profiling is active.
   */
  virtual bool GetIsProfiling() const { return this->m_isProfiling; }

  /**
   * Set if performance profiling is active.
   */
  virtual void SetIsProfiling(bool ip) { this->m_isProfiling = ip; }

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // Geometry
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  // virtual Geometry* Geometry() const override { return NULL; }

  virtual vector<PtrS<Geometry>> Geometries() const override {
    return vector<PtrS<Geometry>>();
  }

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // State Save/Load
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  virtual void GetState(VectorXd& vs) const override;
  virtual void SetState(const VectorXd& vs) override;

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // Kinematic DOF
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  /**
   * Returns the tree defining the kinematics of the object.
   */
  inline virtual const PtrS<KinematicsTree>& GetKinematicsTree() const;

  /**
   * Returns a const reference to the current DoF set vector.
   */
  inline virtual const vector<PtrS<KinematicsEle>>& GetKinematics() const;

  /**
   * Returns the size of the free DoF variables vector.
   */
  inline virtual int GetNumFreeDOF() const override;

  /**
   * Returns the size of the free DoF variables vector.
   */
  inline virtual int GetNumFixedDOF() const override;

  /**
   * Returns the size of the full DoF variables vector.
   */
  inline virtual int GetNumFullDOF() const override;

  virtual void GetDOFVector(VectorXd& vx, Tag s) const override;
  virtual void SetDOFVector(const VectorXd& vx, Tag s) override;

  virtual void UpdateMechanics() override;
  virtual void UpdateKinematics() override;

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // Mechanical state
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  virtual void SetUseFDGradientAtElements(bool u);
  virtual void SetUseFDHessianAtElements(bool u);

  virtual const iVector& GetSelectedElements() const {
    return this->m_vselEles;
  }
  virtual void SetSelectedElements(const iVector& vsel) {
    this->m_vselEles = vsel;
  }

  inline virtual const VectorXd& GetConstraint() override;
  inline virtual const MatrixSd& GetJacobian() override;

  virtual void GetEnergy(Real& energy, bool addBC = true);
  virtual void AddEnergy(Real& energy, bool addBC = true);

  virtual void GetGradient(AVectorXd& vgradient, bool addBC = true);
  virtual void AddGradient(AVectorXd& vgradient, bool addBC = true);

  virtual void GetHessian(AMatrixSd& mHessian, bool addBC = true);
  virtual void AddHessian(AMatrixSd& mHessian, bool addBC = true);

  virtual void GetInertia(AMatrixSd& mInertia);
  virtual void AddInertia(AMatrixSd& mInertia);

  virtual void GetDMvDt(AVectorXd& vDMvDt);
  virtual void AddDMvDt(AVectorXd& vDMvDt);

  virtual void GetDEDp(AVectorXd& vDEDp, bool addBC = true) {}
  virtual void GetDE2Exp(AMatrixSd& mDE2Exp, bool addBC = true) {}
  virtual void GetDE2Epp(AMatrixSd& mDE2Epp, bool addBC = true) {}

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // DoF free/fixed
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  virtual void ComputeAndStore_Fixed();
  virtual const MatrixSd& GetFreeSelection() override;
  virtual const MatrixSd& GetFreePermutation() override;
  virtual void FixVectorDoFs(VectorXd& vv) override;
  virtual void FixMatrixDoFs(MatrixSd& mM) override;

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // Mechanical computations
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  virtual void ComputeAndStore_Mass();
  virtual void ComputeAndStore_Rest();
  virtual void ComputeAndStore_Energy(bool addBC = true);
  virtual void ComputeAndStore_Gradient(bool addBC = true);
  virtual void ComputeAndStore_Hessian(bool addBC = true);
  virtual void ComputeAndStore_Constraint(bool addBC = true);
  virtual void ComputeAndStore_Jacobian(bool addBC = true);

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // Mechanical computations
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  virtual void ComputeAndStore_DEDp(bool addBC = true) {}
  virtual void ComputeAndStore_D2EDxp(bool addBC = true) {}
  virtual void ComputeAndStore_D2EDpp(bool addBC = true) {}

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // Dirty flags
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  virtual void CleanFlags_Fixed();

  virtual void DirtyMass();
  virtual void DirtyRest();
  virtual void DirtyKinematics();
  virtual void DirtyMechanics();
  virtual void DirtyFixed();

  virtual bool IsDirty_Kinematics() const;
  virtual bool IsDirty_Mechanics() const;
               
  virtual bool IsDirty_Energy() const;
  virtual bool IsDirty_Gradient() const;
  virtual bool IsDirty_Hessian() const;
  virtual bool IsDirty_Rest() const;
  virtual bool IsDirty_Mass() const;
  virtual bool IsDirty_Constraint() const;
  virtual bool IsDirty_Jacobian() const;
  virtual bool IsDirty_Fixed() const;

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // Finite difference testing
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  virtual void TestLocalGradients();
  virtual void TestGlobalGradient();
  virtual void TestLocalHessians();
  virtual void TestGlobalHessian();
  virtual void TestGlobalJacobian();

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // Object, mass and energy elements
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  virtual const vector<PtrS<IConstraintSet>>& GetConstraintSets() const {
    return this->m_vconsSet;
  }
  virtual const vector<PtrS<IEnergyElement>>& GetEnergyElements() const {
    return this->m_venerEle;
  }
  virtual const vector<PtrS<IMassElement>>& GetMassElements() const {
    return this->m_vmassEle;
  }

  virtual void AddEnergyElement(PtrS<IEnergyElement>& pEle);
  virtual void AddConstraintSet(PtrS<IConstraintSet>& pCon);
  virtual void AddMassElement(PtrS<IMassElement>& pMass);

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // Boundary conditions management
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  virtual bool BoundaryConditionsLoaded() override;
  virtual void ResetBoundaryConditions() override;
  virtual void StepBoundaryConditions() override;
  virtual void FullBoundaryConditions() override;

  virtual void AddBoundaryCondition(PtrS<IBCondition> pBC);
  virtual void RemoveBoundaryCondition(PtrS<IBCondition> pBC);
  virtual void ClearBoundaryConditions();
  virtual const vector<PtrS<IBCondition>>& GetBoundaryConditions() {
    return this->m_vBC;
  }

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // Timers
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  virtual const CustomTimer& GetTimer_ComputeEnergy() const {
    return this->m_timerComputeEner;
  }
  virtual const CustomTimer& GetTimer_ComputeGradient() const {
    return this->m_timerComputeGrad;
  }
  virtual const CustomTimer& GetTimer_ComputeHessian() const {
    return this->m_timerComputeHess;
  }
  virtual const CustomTimer& GetTimer_AssembleEnergy() const {
    return this->m_timerAssembleEner;
  }
  virtual const CustomTimer& GetTimer_AssembleGradient() const {
    return this->m_timerAssembleGrad;
  }
  virtual const CustomTimer& GetTimer_AssembleHessian() const {
    return this->m_timerAssembleHess;
  }

  //////////////////////////////////////////////
  //////////////////////////////////////////////
  // DoF selection
  //////////////////////////////////////////////
  //////////////////////////////////////////////

  virtual vector<IDoFSet*> SelectDoF(const Vector3d& vboxMin,
                                     const Vector3d& vboxMax,
                                     Tag s) const {
    return vector<IDoFSet*>();
  }

 protected:
  /**
   * Frees the model data. This method frees the memory allocated to store
   * the DoF, energy elements and mass elements. Any class deriving from
   * Simulable that allocates additional memory must override this method as it
   * is called by Simulable destructor. Make sure the memory allocated by Module
   * is also freed by always calling parent FreeInternal() method.
   */
  virtual void FreeInternal();

  /**
   * Initialize all the necessary components of the Simulable.
   */
  virtual void InitInternal();
};
}  // namespace PhySim
