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

#include <PhySim/Physics/Boundary/BCondition.h>

#include <PhySim/Utils/DomainDistribution.h>

#include <PhySim/Solvers/OptimProblem.h>

#include <PhySim/Utils/Third/pugixml.hpp>

namespace PhySim {
using namespace std;
using namespace Eigen;

using xml_node = pugi::xml_node;
using xml_docu = pugi::xml_document;
using xml_attr = pugi::xml_attribute;

class IShapeFunction;
class IQuadrature;
class IMaterialModel;
class ParameterSet;
class OptimSolver;
class OptimProblem;
class IBCondition;

class Serializer_XML {
 protected:
  string m_filePath;
  string m_fileDire;

  xml_docu m_document;

 public:
  Serializer_XML() {
    // Nothing to do here...
  }

  virtual ~Serializer_XML() {
    // Nothing to do here...
  }

  bool LoadFile(const string& filePath);

  const string& FilePath() const { return this->m_filePath; }
  const string& FileDire() const { return this->m_fileDire; }
  xml_docu& Document() { return this->m_document; }

  bool LoadNode(const xml_node& eleIn, xml_node& eleOut);

  // template <typename T> bool LoadMatrix(const xml_node& ele, Matrix<T, -1,
  // -1>& mat); template <typename T> bool LoadVector(const xml_node& ele,
  // Matrix<T, -1, 1>& mat);
  bool LoadVectorXi(const xml_node& ele, VectorXi& vec);
  bool LoadMatrixXi(const xml_node& ele, MatrixXi& mat);
  bool LoadVectorXd(const xml_node& ele, VectorXd& vec);
  bool LoadMatrixXd(const xml_node& ele, MatrixXd& mat);
  bool LoadVector2D(const xml_node& ele, Vector2d& vec);
  bool LoadMatrix2D(const xml_node& ele, Matrix2d& mat);
  bool LoadVector3D(const xml_node& ele, Vector3d& vec);
  bool LoadMatrix3D(const xml_node& ele, Matrix3d& mat);
  bool LoadRotation2DEuler(const xml_node& ele, Matrix2d& mR);
  bool LoadRotation3DEuler(const xml_node& ele, Matrix3d& mR);
  bool LoadScale2DVector(const xml_node& ele, Matrix2d& mS);
  bool LoadScale3DVector(const xml_node& ele, Matrix3d& mS);
  bool LoadSemantics(const xml_attr& att, Tag& sema);

  bool LoadSimulableVector(const xml_attr& att,
                           Simulable* pModel,
                           VectorXd& vx);

  bool LoadDiscretization(const xml_node& ele, Discretization& dis);

  template <typename T>
  bool LoadDistributed(const xml_node& ele, PtrS<T>& pEle);
  bool LoadDistributed(const xml_node& ele, PtrS<IQuadrature>& pQuad);
  bool LoadDistributed(const xml_node& ele, PtrS<IShapeFunction>& pShape);
  bool LoadDistributed(const xml_node& ele, PtrS<IMaterialModel>& pModel);
  bool LoadDistributed(const xml_node& ele, PtrS<ParameterSet>& pParam);
  template <typename T>
  bool LoadDistribution(const xml_node& ele,
                        PtrS<IDomainDistribution<PtrS<T>>>& pDist);

  // bool LoadMaterialSampler(const xml_node& ele, int dimSpace, int dimBasis,
  // vector<PtrS<IMaterialModel>>& vpModel, vector<ParameterSet>& vParam);

  bool LoadMesh_Surface(const xml_node& ele,
                        MatrixXd& mV,
                        MatrixXi& mF,
                        MatrixXd& mN);

  bool LoadMesh_HexVolume(const xml_node& ele,
                          MatrixXd& mV,
                          MatrixXi& mF,
                          MatrixXi& mT);
  bool LoadMesh_TetVolume(const xml_node& ele,
                          MatrixXd& mV,
                          MatrixXi& mF,
                          MatrixXi& mT);

  bool LoadSimulable_ThinShell(const xml_node& ele, PtrS<Simulable>& pModel);
  bool LoadSimulable_VolumeFEM(const xml_node& ele, PtrS<Simulable>& pModel);

  bool LoadOptimProblem(const xml_node& ele,
                        ISimulable* pModel,
                        PtrS<OptimProblem>& pProblem);

  bool LoadOptimSolver(const xml_node& ele,
                       IOptimProblem* pProblem,
                       PtrS<OptimSolver>& pSolver);

  bool LoadBoundary(const xml_node& ele,
                    Simulable* pModel,
                    vector<PtrS<IBCondition>>& vpBC);

  bool LoadBoundary(const xml_node& ele,
                    Simulable* pModel,
                    PtrS<IBCondition>& pBC);

  bool LoadGeometryFilter(const xml_node& ele, PtrS<GeometryFilter>& pGF);
  bool LoadVectorAnimator(const xml_node& ele, PtrS<IVectorAnimator>& pVA);

  bool CheckNodeChild(const xml_node& ele, const string& name, bool warning);
  bool CheckNodeAttrr(const xml_node& ele, const string& name, bool warning);
  void InvalidNodeValue(const xml_node& ele,
                        const string& expected,
                        bool warning);
  void InvalidAttriValue(const xml_attr& att,
                         const string& expected,
                         bool warning);
};
}  // namespace PhySim