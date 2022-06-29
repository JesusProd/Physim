#include <PhySim/Utils/ParameterSet.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

const string ParameterSet::Param_CollT = "COLLT";
const string ParameterSet::Param_CollK = "COLLK";
const string ParameterSet::Param_Pressure = "PRESSURE";
const string ParameterSet::Param_Density = "DENSITY";
const string ParameterSet::Param_Poisson = "POISSON";
const string ParameterSet::Param_StretchK = "STRETCH";
const string ParameterSet::Param_BendingK = "BENDING";
const string ParameterSet::Param_ShearK = "SHEAR";
const string ParameterSet::Param_TwistK = "TWIST";
const string ParameterSet::Param_Young = "YOUNG";
const string ParameterSet::Param_Bulk = "BULK";
const string ParameterSet::Param_Lame1 = "LAME1";
const string ParameterSet::Param_Lame2 = "LAME2";
const string ParameterSet::Param_SNHAlpha = "ALPHA";
const string ParameterSet::Param_ShearMod = "LAME2";
const string ParameterSet::Param_Mooney01 = "MR01";
const string ParameterSet::Param_Mooney10 = "MR10";
const string ParameterSet::Param_Thickness = "THICK";
const string ParameterSet::Param_Radius0 = "RADIUS0";
const string ParameterSet::Param_Radius1 = "RADIUS1";

void ParameterSet::InitFromDensity(Real density) {
  this->AddParameter(Param_Density, density);
}

void ParameterSet::InitSNHFromYoungPoisson(Real young,
                                           Real poisson,
                                           Real density) {
  this->AddParameter(Param_Young, young);
  this->AddParameter(Param_Poisson, poisson);
  this->AddParameter(Param_Density, density);

  // Parameterization of Lame parameters for linear materials
  Real lame1 = (young * poisson / ((1 + poisson) * (1 - 2 * poisson)));
  Real lame2 = (young / (2 * (1 + poisson)));

  // Reparameterization to be consistent with linear elasticity. This
  // reparameterization is mentioned in Stable Neo-Hookean Flesh Simulation
  // [Smith et al. 2018, Sec. 3.4]
  lame1 += lame2 * (5.0 / 6.0);
  lame2 = lame2 * (4.0 / 3.0);
  Real alpha = (1 + lame2 / lame1 - lame2 / (lame1 * 4));
  this->AddParameter(Param_Lame1, lame1);
  this->AddParameter(Param_Lame2, lame2);
  this->AddParameter(Param_SNHAlpha, alpha);
}

void ParameterSet::InitLinearFromYoungPoisson(Real young,
                                              Real poisson,
                                              Real density) {
  this->AddParameter(Param_Young, young);
  this->AddParameter(Param_Poisson, poisson);
  this->AddParameter(Param_Density, density);

  Real t0 = 1 + poisson;
  Real t1 = 1 - 2 * poisson;
  Real lame1 = (young * poisson) / (t0 * t1);
  Real lame2 = young / (2 * t0);
  Real bulk = young / (3 * t1);

  this->AddParameter(Param_Lame1, lame1);
  this->AddParameter(Param_Lame2, lame2);
  this->AddParameter(Param_Bulk, bulk);
}

void ParameterSet::InitLinearFromLameParameters(Real lame1,
                                                Real lame2,
                                                Real density) {
  this->AddParameter(Param_Lame1, lame1);
  this->AddParameter(Param_Lame2, lame2);
  this->AddParameter(Param_Density, density);

  Real l = lame1;
  Real G = lame2;
  Real bulk = l + (2 / 3) * G;
  Real poisson = l / (2 * (l + G));
  Real young = G * (3 * l + 2 * G) / (l + G);

  this->AddParameter(Param_Young, young);
  this->AddParameter(Param_Poisson, poisson);
  this->AddParameter(Param_Bulk, bulk);
}
}  // namespace PhySim