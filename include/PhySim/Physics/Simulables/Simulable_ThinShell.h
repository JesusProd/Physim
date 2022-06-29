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

#include <PhySim/Geometry/Meshes/Mesh_Face.h>

#include <PhySim/Physics/Simulables/Simulable_FEM_Surface.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

class EnergyElement_DiscreteShells;

/**
 * Simulable_ThinShell.
 *
 * TODO.
 */
class Simulable_ThinShell : public Simulable_FEM_Surface {
 public:
  /**
   * TODO.
   */
  struct Options : public Simulable_FEM_Surface::Options {
    ParameterSet m_material;

    Options() {
      // Nothing to do here...
    }
  };

 protected:
  vector<EnergyElement_DiscreteShells*> m_venergyEle_bending;

 public:
  virtual string GetName() const override { return "[Thin-Shells]"; }

  virtual Options& SetupOptions() override;

  /**
   * Constructor.
   */
  Simulable_ThinShell();

  /**
   * Destructor.
   */
  virtual ~Simulable_ThinShell();

  /**
   * FreeInternal extra allocated memory.
   */
  virtual void FreeInternal() override;

  virtual void ComputePlanarStrain(vector<vector<Matrix2d>>& vmEp);
  virtual void ComputeBendingStrain(vector<vector<Matrix2d>>& vmEb);
  virtual void ComputeTotalStrain(vector<vector<Matrix2d>>& vmEt);
  virtual void ComputeStressIntegral(const vector<vector<Matrix2d>>& vmE,
                                     vector<Matrix2d>& vmS);
  virtual void ComputeStressDensity(const vector<vector<Matrix2d>>& vmE,
                                    vector<Matrix2d>& vmS);
  virtual Matrix2d ComputeStressForE(const Matrix2d& mE);

  virtual void ComputeNodalBendingStrain(vector<Matrix2d>& vmEb);
  virtual void ComputeNodalBendingStress(vector<Matrix2d>& vmSb);

  virtual const vector<EnergyElement_DiscreteShells*>&
  GetEnergyElements_ShellHinge() const {
    return this->m_venergyEle_bending;
  }

 protected:
  virtual void CreateEnergyElements(
      vector<IEnergyElement*>& vEnergies) override;
};
}  // namespace PhySim
