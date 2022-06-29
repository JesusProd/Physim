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

#include <PhySim/Physics/Simulables/Simulable_Mesh.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class IEnergyElement;
class EnergyElement_SpringLinear;
class EnergyElement_SpringHinge;
class EnergyElement_SpringCross;

/**
 * Simulable_MassSpring.
 *
 * TODO.
 */
class Simulable_MassSpring : public Simulable_Mesh {
 public:
  /**
   * TODO.
   */
  struct Springs {
    VectorXi m_vlinear;
    MatrixXi m_mhinges;
    MatrixXi m_mcrosses;

    Springs() {
      m_vlinear.setZero();
      m_mhinges.setZero();
      m_mcrosses.setZero();
    }
  };

  /**
   * TODO
   */
  struct Options : public Simulable_Mesh::Options {
    /**
     * Structure of springs where vlinear, mhinges and mcrosses contain indices
     * refering to mElems rows.
     */
    Springs m_springs;

    ParameterSet m_material;

    Options() {
      m_material.AddParameter(ParameterSet::Param_Density, 1);
      m_material.AddParameter(ParameterSet::Param_StretchK, 1);
      m_material.AddParameter(ParameterSet::Param_BendingK, 0);
      m_material.AddParameter(ParameterSet::Param_ShearK, 0);
    }
  };

 protected:
  vector<EnergyElement_SpringLinear*> m_venergyEle_linear;
  vector<EnergyElement_SpringHinge*> m_venergyEle_hinge;
  vector<EnergyElement_SpringCross*> m_venergyEle_cross;
  vector<ParameterSet> m_vmat_linear;
  vector<ParameterSet> m_vmat_hinge;
  vector<ParameterSet> m_vmat_cross;

 public:
  virtual string GetName() const override { return "[Mass-Spring System]"; }

  virtual Options& SetupOptions() override;

  /**
   * Constructor.
   */
  Simulable_MassSpring();

  /**
   * Destructor.
   */
  virtual ~Simulable_MassSpring();

  // Initialization
  virtual void FreeInternal();

  // Parameter: rest length
  virtual VectorXd GetRestLinearLengths() const;
  virtual void SetRestLinearLengths(const VectorXd&);

  // Parameter: rest hinge angles
  virtual VectorXd GetRestHingeAngles() const;
  virtual void SetRestHingeAngles(const VectorXd&);

  // Parameter: rest cross ratios
  virtual VectorXd GetRestCrossStrain() const;
  virtual void SetRestCrossStrain(const VectorXd&);

  // Parameter: stretch K
  virtual VectorXd GetStretchK() const;
  virtual void SetStretchK(const VectorXd&);

  // Parameter: bending K
  virtual VectorXd GetBendingK() const;
  virtual void SetBendingK(const VectorXd&);

  // Parameter: shear K
  virtual VectorXd GetShearK() const;
  virtual void SetShearK(const VectorXd&);

  virtual const vector<EnergyElement_SpringLinear*>&
  GetEnergyElements_SpringsLinear() const {
    return this->m_venergyEle_linear;
  }
  virtual const vector<EnergyElement_SpringHinge*>&
  GetEnergyElements_SpringsHinge() const {
    return this->m_venergyEle_hinge;
  }
  virtual const vector<EnergyElement_SpringCross*>&
  GetEnergyElements_SpringsCross() const {
    return this->m_venergyEle_cross;
  }

 protected:
  virtual void CreateEnergyElements(
      vector<IEnergyElement*>& vEnergies) override;
};
}  // namespace PhySim
