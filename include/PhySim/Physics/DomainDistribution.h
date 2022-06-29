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

#include <PhySim/Utils/SpatialDistribution.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

template <class T>
class IDomainDistribution {
 public:
  IDomainDistribution() {}
  virtual ~IDomainDistribution() {}

  virtual T& GetValueAtDomainPoint(
      int eleIdx,
      const VectorXd& vpoint = VectorXd::Zero(3)) = 0;
};

template <class T>
class DomainDistribution_Constant : public IDomainDistribution<T> {
 protected:
  T m_value;

 public:
  T& Value() { return this->m_value; }

  DomainDistribution_Constant(const T& value) { this->m_value = value; }
  virtual ~DomainDistribution_Constant() {}

  virtual T& GetValueAtDomainPoint(int eleIdx,
                                   const VectorXd& vpoint = VectorXd::Zero(3)) {
    return this->m_value;
  };
};

template <class T>
class DomainDistribution_Variable : public IDomainDistribution<T> {
 public:
  vector<T> m_vvalue;
  iVector m_vmapEle;

 public:
  DomainDistribution_Variable(const vector<T>& vvalue, const iVector& vmapEle) {
    this->m_vvalue = vvalue;
    this->m_vmapEle = vmapEle;
  }

  DomainDistribution_Variable(const vector<T>& vvalue) {
    this->m_vvalue = vvalue;
    this->m_vmapEle.resize(vvalue.size());
    for (int i = 0; i < (int)vvalue.size(); ++i)
      this->m_vmapEle[i] = i;
  }

  virtual ~DomainDistribution_Variable() {}

  virtual T& GetValueAtDomainPoint(int eleIdx,
                                   const VectorXd& vpoint = VectorXd::Zero(3)) {
    return this->m_vvalue[m_vmapEle[eleIdx]];
  };
};

template <class T>
class DomainDistribution_Subelement : public IDomainDistribution<T> {
 protected:
  vector<T> m_vvalue;
  MatrixXi m_mmapEle;
  VectorXi m_vCoarsening;

 public:
  DomainDistribution_Subelement(const vector<T>& vvalue,
                                const MatrixXi& mmapEle,
                                const VectorXi& vCoarsening) {
    this->m_vvalue = vvalue;
    this->m_mmapEle = mmapEle;
    this->m_vCoarsening = vCoarsening;
  }

  virtual ~DomainDistribution_Subelement() {}

  virtual T& GetValueAtDomainPoint(int eleIdx,
                                   const VectorXd& vpoint = VectorXd::Zero(3)) {
    return this->m_vvalue[m_mmapEle(eleIdx, ChooseSubelement(vpoint))];
  };

 private:
  int ChooseSubelement(const VectorXd& vp) const {
    int x = (int)(std::floor((vp(0) + 1) * this->m_vCoarsening.x() / 2));
    int y = (int)(std::floor((vp(1) + 1) * this->m_vCoarsening.y() / 2));
    int z = (int)(std::floor((vp(2) + 1) * this->m_vCoarsening.z() / 2));
    return x * this->m_vCoarsening.y() * this->m_vCoarsening.z() +
           y * this->m_vCoarsening.z() + z;
  };
};

template <class T>
class DomainDistribution_Procedural : public IDomainDistribution<T> {
 protected:
  PtrS<ISpatialDistribution<T>> m_pFunction;
  vector<Poly*> m_vPolys;
  Tag m_layer;

 public:
  DomainDistribution_Procedural(PtrS<ISpatialDistribution<T>> pFunction,
                                const vector<Poly*>& vPolys,
                                Tag layer) {
    this->m_vPolys = vPolys;
    this->m_pFunction = pFunction;
    this->m_layer = layer;
  }

  virtual ~DomainDistribution_Procedural() {}

  virtual T& GetValueAtDomainPoint(int eleIdx,
                                   const VectorXd& vpoint = VectorXd()) {
    // Interpolate the positions of the geometric element
    // at the specified parametric point. If the geometric
    // element is a node, this should return the value of
    // the node at the specified layer.

    VectorXd vx = m_vPolys[eleIdx]->InterpolateValue(vpoint, m_layer);

    return this->m_pFunction->GetValueAtSpatialPoint(vx);
  };
};
}  // namespace PhySim