//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Utils/DomainDistribution.h>
#include <PhySim/Utils/GeometryUtils.h>
#include <PhySim/Utils/IOUtils.h>
#include <PhySim/Utils/MathUtils.h>
#include <PhySim/Utils/MeshUtils.h>
#include <PhySim/Utils/Serializer_Mesh.h>
#include <PhySim/Utils/Serializer_XML.h>

#include <PhySim/Geometry/Meshes/Mesh_Hexa.h>
#include <PhySim/Geometry/Meshes/Mesh_Quad.h>
#include <PhySim/Geometry/Meshes/Mesh_Tetra.h>
#include <PhySim/Geometry/Meshes/Mesh_Tri.h>
#include <PhySim/Physics/DoFSet.h>

#include <PhySim/Physics/Boundary/BC_AlignFace.h>
#include <PhySim/Physics/Boundary/BC_FixDoF.h>
#include <PhySim/Physics/Boundary/BC_Force.h>
#include <PhySim/Physics/Boundary/BC_Gravity.h>
#include <PhySim/Physics/Boundary/BC_PlaneColl.h>
#include <PhySim/Physics/Boundary/BCondition.h>
#include <PhySim/Physics/Simulables/Simulable_FEM_Volume.h>
#include <PhySim/Physics/Simulables/Simulable_ThinShell.h>

#include <PhySim/Solvers/OptimProblem_BasicDynamic.h>
#include <PhySim/Solvers/OptimProblem_BasicStatic.h>
#include <PhySim/Solvers/OptimSolver_USQP_LS.h>

#include <PhySim/Geometry/Polytopes/Quadratures.h>
#include <PhySim/Physics/Elements/MaterialModels.h>

#include <fstream>
#include <streambuf>

namespace PhySim {
using namespace std;
using namespace Eigen;

bool Serializer_XML::LoadFile(const string& filePath) {
  // Read setup XML

  string fileString;

  if (!IOUtils::readFileString(filePath, fileString)) {
    IOUtils::logTrace(Verbosity::V1_Default, "\n[ERROR] Invalid file path: %s",
                      filePath);
    return false;
  }

  pugi::xml_parse_result result =
      this->m_document.load_string(fileString.c_str());

  if (!result) {
    IOUtils::logTrace(Verbosity::V1_Default,
                      "\n[ERROR] Invalid XML file %s: %s", filePath.c_str(),
                      result.description());
    return false;
  }

  m_fileDire = IOUtils::getFileDirectory(filePath);
  m_filePath = filePath;

  return true;
}

bool Serializer_XML::LoadNode(const xml_node& eleIn, xml_node& eleOut) {
  // If there is not "outsource" attribute then
  // the node is defined in the same document

  if (eleIn.attribute("outsource").empty()) {
    eleOut = eleIn;
  } else {
    // The node is defined externally in the specified document path

    const string& nodePath = eleIn.attribute("outsource").as_string();

    string nodeString;

    if (!IOUtils::readFileString(nodePath, nodeString)) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Invalid file path: %s", nodePath);
      return false;
    }

    xml_docu docu;

    pugi::xml_parse_result result = docu.load_string(nodeString.c_str());

    if (!result) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Invalid XML file %s: %s", nodePath.c_str(),
                        result.description());
      return false;
    }

    eleOut = docu.root();
  }

  return true;
}

bool Serializer_XML::LoadVectorXi(const xml_node& ele, VectorXi& vec) {
  VectorXd vecTemp = IOUtils::stringToVector_CSV(ele.text().as_string());

  vec.resize(vecTemp.size());
  for (int i = 0; i < vecTemp.size(); ++i)
    vec(i) = (int)vecTemp(i);

  if (vec.size() == 0) {
    IOUtils::logTrace(
        Verbosity::V1_Default,
        "\n[ERROR] Expected node [%s] text: CSV vector. Found: %s", ele.name(),
        ele.text());
    return false;
  }

  return true;
}

bool Serializer_XML::LoadMatrixXi(const xml_node& ele, MatrixXi& mat) {
  MatrixXd matTemp = IOUtils::stringToMatrix_CSV(ele.text().as_string());

  mat.resize(matTemp.rows(), matTemp.cols());
  for (int i = 0; i < matTemp.rows(); ++i)
    for (int j = 0; j < matTemp.cols(); ++j)
      mat(i, j) = (int)matTemp(i, j);

  if (mat.size() == 0) {
    IOUtils::logTrace(
        Verbosity::V1_Default,
        "\n[ERROR] Expected node [%s] text: CSV matrix. Found: %s", ele.name(),
        ele.text());
    return false;
  }

  return true;
}

bool Serializer_XML::LoadVectorXd(const xml_node& ele, VectorXd& vec) {
  vec = IOUtils::stringToVector_CSV(ele.text().as_string());

  if (vec.size() == 0) {
    IOUtils::logTrace(
        Verbosity::V1_Default,
        "\n[ERROR] Expected node [%s] text: CSV vector. Found: %s", ele.name(),
        ele.text());
    return false;
  }

  return true;
}

bool Serializer_XML::LoadMatrixXd(const xml_node& ele, MatrixXd& mat) {
  mat = IOUtils::stringToMatrix_CSV(ele.text().as_string());

  if (mat.size() == 0) {
    IOUtils::logTrace(
        Verbosity::V1_Default,
        "\n[ERROR] Expected node [%s] text: CSV matrix. Found: %s", ele.name(),
        ele.text());
    return false;
  }

  return true;
}

bool Serializer_XML::LoadVector2D(const xml_node& ele, Vector2d& vec) {
  vec = IOUtils::stringToVector_CSV(ele.text().as_string());

  if (vec.size() != 2) {
    IOUtils::logTrace(
        Verbosity::V1_Default,
        "\n[ERROR] Expected node [%s] text: CSV 2D vector. Found: %s",
        ele.name(), ele.text());
    return false;
  }

  return true;
}

bool Serializer_XML::LoadMatrix2D(const xml_node& ele, Matrix2d& mat) {
  mat = IOUtils::stringToMatrix_CSV(ele.text().as_string());

  if (mat.rows() != 2 || mat.cols() != 2) {
    IOUtils::logTrace(
        Verbosity::V1_Default,
        "\n[ERROR] Expected node [%s] text: CSV 2D matrix. Found: %s",
        ele.name(), ele.text());
    return false;
  }

  return true;
}

bool Serializer_XML::LoadVector3D(const xml_node& ele, Vector3d& vec) {
  vec = IOUtils::stringToVector_CSV(ele.text().as_string());

  if (vec.size() != 3) {
    IOUtils::logTrace(
        Verbosity::V1_Default,
        "\n[ERROR] Expected node [%s] text: CSV 3D vector. Found: %s",
        ele.name(), ele.text());
    return false;
  }

  return true;
}

bool Serializer_XML::LoadMatrix3D(const xml_node& ele, Matrix3d& mat) {
  mat = IOUtils::stringToMatrix_CSV(ele.text().as_string());

  if (mat.rows() != 3 || mat.cols() != 3) {
    IOUtils::logTrace(
        Verbosity::V1_Default,
        "\n[ERROR] Expected node [%s] text: CSV 3D matrix. Found: %s",
        ele.name(), ele.text());
    return false;
  }

  return true;
}

bool Serializer_XML::LoadRotation3DEuler(const xml_node& ele, Matrix3d& mR) {
  Vector3d vdegrees;

  if (!this->LoadVector3D(ele, vdegrees))
    return false;

  Vector3d vradians(MathUtils::degreesToRadians(vdegrees.x()),
                    MathUtils::degreesToRadians(vdegrees.y()),
                    MathUtils::degreesToRadians(vdegrees.z()));
  mR = GeometryUtils::rotationEulerToMatrix(vradians);

  return true;
}

bool Serializer_XML::LoadRotation2DEuler(const xml_node& ele, Matrix2d& mR) {
  double degrees = ele.text().as_double();
  double radians = MathUtils::degreesToRadians(degrees);
  double cosr = cos(radians);
  double sinr = sin(radians);
  mR(0, 0) = cosr;
  mR(0, 1) = -sinr;
  mR(1, 0) = sinr;
  mR(1, 1) = cosr;

  return true;
}

bool Serializer_XML::LoadScale3DVector(const xml_node& ele, Matrix3d& mS) {
  Vector3d vscale;

  if (!this->LoadVector3D(ele, vscale))
    return false;

  mS.setIdentity();
  mS(0, 0) = vscale.x();
  mS(1, 1) = vscale.y();
  mS(2, 2) = vscale.z();

  return true;
}

bool Serializer_XML::LoadScale2DVector(const xml_node& ele, Matrix2d& mS) {
  Vector2d vscale;

  if (!this->LoadVector2D(ele, vscale))
    return false;

  mS.setIdentity();
  mS(0, 0) = vscale.x();
  mS(1, 1) = vscale.y();

  return true;
}

bool Serializer_XML::LoadSemantics(const xml_attr& att, Tag& sema) {
  string value = att.as_string();

  if (value.compare("PosX") == 0) {
    sema = Tag::Position_X;
    return true;
  }
  if (value.compare("Pos0") == 0) {
    sema = Tag::Position_0;
    return true;
  }
  if (value.compare("VelX") == 0) {
    sema = Tag::Velocity;
    return true;
  }

  this->InvalidAttriValue(att, "Semantics (PosX, Pos0, VelX...)", false);

  return false;
}

bool Serializer_XML::LoadDiscretization(const xml_node& ele,
                                        Discretization& dis) {
  if (!this->CheckNodeAttrr(ele, "type", false))
    return false;

  string typeValue = ele.attribute("type").as_string();

  if (typeValue == "NODE") {
    dis = Discretization::Nodes;
    return true;
  }

  if (typeValue == "EDGE2") {
    dis = Discretization::Edge2;
    return true;
  }
  if (typeValue == "EDGE3") {
    dis = Discretization::Edge3;
    return true;
  }

  if (typeValue == "TRI3") {
    dis = Discretization::Tri3;
    return true;
  }
  if (typeValue == "TRI6") {
    dis = Discretization::Tri6;
    return true;
  }

  if (typeValue == "QUAD4") {
    dis = Discretization::Quad4;
    return true;
  }
  if (typeValue == "QUAD8") {
    dis = Discretization::Quad8;
    return true;
  }

  if (typeValue == "TET4") {
    dis = Discretization::Tet4;
    return true;
  }
  if (typeValue == "TET10") {
    dis = Discretization::Tet10;
    return true;
  }

  if (typeValue == "HEX8") {
    dis = Discretization::Hex8;
    return true;
  }
  if (typeValue == "HEX20") {
    dis = Discretization::Hex20;
    return true;
  }

  this->InvalidAttriValue(ele.attribute("type"),
                          "Discretization type (TRI3, TET4, HEX8, ...)", false);

  return false;
}

template <typename T>
bool Serializer_XML::LoadDistributed(const xml_node& ele, PtrS<T>& pEle) {
  // Not implemented...

  return false;
}

bool Serializer_XML::LoadDistributed(const xml_node& ele,
                                     PtrS<IQuadrature>& pQuad) {
  if (!this->CheckNodeAttrr(ele, "type", false))
    return false;

  string typeValue = ele.attribute("type").as_string();

  if (typeValue == "TRI_1") {
    pQuad.reset(new Quadrature_Tri1());
    return true;
  }

  if (typeValue == "TET_1") {
    pQuad.reset(new Quadrature_Tet1());
    return true;
  }
  if (typeValue == "TET_4") {
    pQuad.reset(new Quadrature_Tet4());
    return true;
  }
  if (typeValue == "TET_5") {
    pQuad.reset(new Quadrature_Tet5());
    return true;
  }
  if (typeValue == "TET_8") {
    pQuad.reset(new Quadrature_Tet8());
    return true;
  }
  if (typeValue == "TET_11") {
    pQuad.reset(new Quadrature_Tet11());
    return true;
  }

  if (typeValue == "QUAD_CENTER") {
    pQuad.reset(new Quadrature_Quad1());
    return true;
  }
  if (typeValue == "HEX_CENTER") {
    pQuad.reset(new Quadrature_Hex1());
    return true;
  }

  if (typeValue == "QUAD_GAUSS") {
    pQuad.reset(new Quadrature_Gauss(2, ele.attribute("num").as_int()));
    return true;
  }
  if (typeValue == "HEX_GAUSS") {
    pQuad.reset(new Quadrature_Gauss(3, ele.attribute("num").as_int()));
    return true;
  }

  this->InvalidAttriValue(ele.attribute("type"),
                          "Quadrature type (TRI_1, TET_1, HEX_GAUSS, ...)",
                          false);

  return false;
}

bool Serializer_XML::LoadDistributed(const xml_node& ele,
                                     PtrS<IShapeFunction>& pShape) {
  if (!this->CheckNodeAttrr(ele, "type", false))
    return false;

  string typeValue = ele.attribute("type").as_string();

  if (typeValue == "TRI_LIN") {
    pShape.reset(new ShapeFunction_Tri3());
    return true;
  }
  if (typeValue == "TRI_QUAD") {
    pShape.reset(new ShapeFunction_Tri6());
    return true;
  }

  if (typeValue == "QUAD_BILIN") {
    pShape.reset(new ShapeFunction_Quad4());
    return true;
  }
  if (typeValue == "QUAD_BIQUAD") {
    pShape.reset(new ShapeFunction_Quad8());
    return true;
  }

  if (typeValue == "TET_LIN") {
    pShape.reset(new ShapeFunction_Tet4());
    return true;
  }
  if (typeValue == "TET_QUAD") {
    pShape.reset(new ShapeFunction_Tet10());
    return true;
  }

  if (typeValue == "HEX_TRILIN") {
    pShape.reset(new ShapeFunction_Hex8());
    return true;
  }
  if (typeValue == "HEX_TRIQUAD") {
    pShape.reset(new ShapeFunction_Hex20());
    return true;
  }

  // Numerically coarsened SF following the implementation by Chen et
  // al. 2018: Numerical Coarsening using Discontinuous Shape Functions

  if (typeValue == "HEX_COARSEN") {
    if (!this->CheckNodeChild(ele, "subelements", false))
      return false;
    if (!this->CheckNodeChild(ele, "coefficients", false))
      return false;

    // Load coarsening

    VectorXi vCoars;

    if (!this->LoadVectorXi(ele.child("subelements"), vCoars))
      return false;

    // Load coefficients

    VectorXd vCoeff;

    if (!this->LoadVectorXd(ele.child("coefficients"), vCoeff))
      return false;

    // Convert to format accepted by the shape function

    int numFineNodes = (vCoars.x() + 1) * (vCoars.y() + 1) * (vCoars.z() + 1);

    vector<vector<Matrix3d>> vmnij;
    vmnij.resize(8);
    for (int j = 0; j < 8; ++j) {
      vmnij[j].resize(numFineNodes);
      for (int k = 0; k < numFineNodes; ++k) {
        for (int ii = 0; ii < 3; ii++)
          for (int jj = 0; jj < 3; jj++)
            vmnij[j][k](ii, jj) =
                vCoeff(9 * numFineNodes * j + 9 * k + 3 * jj + ii);
      }
    }

    pShape.reset(new ShapeFunction_Chen(vmnij, vCoars));

    return true;
  }

  this->InvalidAttriValue(
      ele.attribute("type"),
      "Discretization type (TRI_LIN, TET_LIN, HEX_TRILIN, ...)", false);

  return false;
}

bool Serializer_XML::LoadDistributed(const xml_node& ele,
                                     PtrS<IMaterialModel>& pModel) {
  if (!this->CheckNodeAttrr(ele, "type", false))
    return false;
  if (!this->CheckNodeAttrr(ele, "dimSpace", false))
    return false;
  if (!this->CheckNodeAttrr(ele, "dimBasis", false))
    return false;

  string typeValue = ele.attribute("type").as_string();
  int dimSpace = ele.attribute("dimSpace").as_int();
  int dimBasis = ele.attribute("dimBasis").as_int();

  if (dimSpace == 3 && dimBasis == 3) {
    if (typeValue == "STVK") {
      pModel.reset(new MaterialModel_3Din3D_StVK());
      return true;
    }
    if (typeValue == "CONH") {
      pModel.reset(new MaterialModel_3Din3D_CoNH());
      return true;
    }
    if (typeValue == "COMR") {
      pModel.reset(new MaterialModel_3Din3D_CoMR());
      return true;
    }
  }

  if (dimSpace == 3 && dimBasis == 2) {
    if (typeValue == "STVK") {
      pModel.reset(new MaterialModel_2Din3D_StVK());
      return true;
    }
    if (typeValue == "CONH") {
      pModel.reset(new MaterialModel_2Din3D_CoNH());
      return true;
    }
    if (typeValue == "InNH") {
      pModel.reset(new MaterialModel_2Din3D_InNH());
      return true;
    }
  }

  this->InvalidAttriValue(ele.attribute("type"),
                          "Material model (STVK, CONH, InNH, ...)", false);

  return false;
}

bool Serializer_XML::LoadDistributed(const xml_node& ele,
                                     PtrS<ParameterSet>& pParam) {
  pParam.reset(new ParameterSet());

  pugi::xml_named_node_iterator itCur = ele.children("parameter").begin();
  pugi::xml_named_node_iterator itEnd = ele.children("parameter").end();
  for (; itCur != itEnd; itCur++) {
    bool found = false;

    if (!this->CheckNodeAttrr(*itCur, "name", false))
      return false;
    if (!this->CheckNodeAttrr(*itCur, "value", false))
      return false;

    string name = itCur->attribute("name").as_string();
    double value = itCur->attribute("value").as_double();
    pParam->AddParameter(name, value);
  }

  if (pParam->HasParameter(ParameterSet::Param_Young) &&
      pParam->HasParameter(ParameterSet::Param_Poisson) &&
      pParam->HasParameter(ParameterSet::Param_Density)) {
    pParam->InitLinearFromYoungPoisson((*pParam)[ParameterSet::Param_Young],
                                       (*pParam)[ParameterSet::Param_Poisson],
                                       (*pParam)[ParameterSet::Param_Density]);
    return true;
  }

  if (pParam->HasParameter(ParameterSet::Param_Lame1) &&
      pParam->HasParameter(ParameterSet::Param_Lame2) &&
      pParam->HasParameter(ParameterSet::Param_Density)) {
    pParam->InitLinearFromLameParameters(
        (*pParam)[ParameterSet::Param_Lame1],
        (*pParam)[ParameterSet::Param_Lame2],
        (*pParam)[ParameterSet::Param_Density]);
    return true;
  }

  return true;
}

template <typename T>
bool Serializer_XML::LoadDistribution(
    const xml_node& ele,
    PtrS<IDomainDistribution<PtrS<T>>>& pDist) {
  if (!this->CheckNodeAttrr(ele, "type", false))
    return false;
  if (!this->CheckNodeChild(ele, ele.name(), false))
    return false;

  string typeValue = ele.attribute("type").as_string();

  // Constant distribution

  if (typeValue == "CONSTANT") {
    if (!this->CheckNodeChild(ele, ele.name(), false))
      return false;

    PtrS<T> pModel;

    if (!this->LoadDistributed(ele.child(ele.name()), pModel))
      return false;
    pDist.reset(new DomainDistribution_Constant<PtrS<T>>(pModel));

    return true;
  }

  // Variable distribution

  if (typeValue == "VARIABLE") {
    // Load object catalogue

    vector<PtrS<T>> vpModel;

    pugi::xml_named_node_iterator itCur = ele.children(ele.name()).begin();
    pugi::xml_named_node_iterator itEnd = ele.children(ele.name()).end();
    for (; itCur != itEnd; itCur++) {
      PtrS<T> pModel;

      if (!this->LoadDistributed(*itCur, pModel))
        return false;

      vpModel.push_back(pModel);
    }

    int numE = (int)vpModel.size();

    // Load/create mapping

    VectorXi vdist;

    if (ele.child("distribution").empty()) {
      vdist.resize(vpModel.size());
      for (int i = 0; i < numE; ++i)
        vdist(i) = i;
    } else {
      this->LoadVectorXi(ele.child("distribution"), vdist);
    }

    // Create distribution

    pDist.reset(
        new DomainDistribution_Variable<PtrS<T>>(vpModel, Utils::toSTL(vdist)));

    return true;
  }

  // Variable distribution

  if (typeValue == "SUBELEMENT") {
    if (!this->CheckNodeChild(ele, "distribution", false))
      return false;
    if (!this->CheckNodeChild(ele, "subelements", false))
      return false;

    // Load coarsening

    VectorXi vCoars;

    if (!this->LoadVectorXi(ele.child("subelements"), vCoars))
      return false;

    // Load material models

    vector<PtrS<T>> vpModel;

    pugi::xml_named_node_iterator itCur = ele.children(ele.name()).begin();
    pugi::xml_named_node_iterator itEnd = ele.children(ele.name()).end();
    for (; itCur != itEnd; itCur++) {
      PtrS<T> pModel;

      if (!this->LoadDistributed(*itCur, pModel))
        return false;

      vpModel.push_back(pModel);
    }

    // Load distribution

    MatrixXi mdist;
    this->LoadMatrixXi(ele.child("distribution"), mdist);
    pDist.reset(
        new DomainDistribution_Subelement<PtrS<T>>(vpModel, mdist, vCoars));

    return true;
  }

  // Assume constant distribution

  PtrS<T> pModel;

  if (!this->LoadDistributed(ele.child(ele.name()), pModel))
    return false;
  pDist.reset(new DomainDistribution_Constant<PtrS<T>>(pModel));

  return false;
}

bool Serializer_XML::LoadSimulableVector(const xml_attr& att,
                                         Simulable* pModel,
                                         VectorXd& vx) {
  string value = att.as_string();

  if (value.compare("PosX") == 0) {
    pModel->GetDOFVector(vx, Tag::Position_X);
    return true;
  }
  if (value.compare("VelX") == 0) {
    pModel->GetDOFVector(vx, Tag::Velocity);
    return true;
  }
  if (value.compare("Pos0") == 0) {
    pModel->GetDOFVector(vx, Tag::Position_0);
    return true;
  }
  // if (value.compare("Mass") == 0) { vx = pModel->GetMass().diagonal(); return
  // true; } if (value.compare("Grad") == 0) { vx = pModel->GetGradient();
  // return true; }

  this->InvalidAttriValue(att, "Model magnitude (PosX, VelX, Mass)", false);

  return false;
}

bool Serializer_XML::LoadMesh_Surface(const xml_node& ele,
                                      MatrixXd& mV,
                                      MatrixXi& mF,
                                      MatrixXd& mN) {
  if (!this->CheckNodeAttrr(ele, "type", false))
    return false;

  string value = ele.attribute("type").as_string();

  if (value.compare("File") == 0) {
    if (!this->CheckNodeAttrr(ele, "path", false))
      return false;

    string meshPath = IOUtils::getAbsolutePath(
        ele.attribute("path").as_string(), this->m_fileDire);

    if (!Serializer_Mesh::ReadSurface_Any(meshPath, mV, mF, mN)) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Invalid mesh file: %s", meshPath);
      return false;
    }

    return true;
  }

  // Create 2D rectangle (Voronoi)

  if (string("rectangle") == value) {
    if (!this->CheckNodeAttrr(ele, "sizeX", false))
      return false;
    if (!this->CheckNodeAttrr(ele, "sizeY", false))
      return false;
    if (!this->CheckNodeAttrr(ele, "numeX", false))
      return false;
    if (!this->CheckNodeAttrr(ele, "numeY", false))
      return false;

    double sizeX = ele.attribute("sizeX").as_double();
    double sizeY = ele.attribute("sizeY").as_double();
    int numeX = ele.attribute("numeX").as_int();
    int numeY = ele.attribute("numeY").as_int();

    if (!ele.attribute("meshing").empty()) {
      string meshingstr = ele.attribute("meshing").as_string();

      if (!MeshUtils::create2DRectangle(sizeX, sizeY, numeX, numeY, mV, mF,
                                        meshingstr)) {
        IOUtils::logTrace(Verbosity::V1_Default,
                          "\n[ERROR] Impossible to triangulate");
        return false;
      }
    } else {
      if (!MeshUtils::create2DRectangle(sizeX, sizeY, numeX, numeY, mV, mF)) {
        IOUtils::logTrace(Verbosity::V1_Default,
                          "\n[ERROR] Impossible to triangulate");
        return false;
      }
    }

    return true;
  }

  // Create 2D triangle (Voronoi)

  if (string("triangle") == value) {
    // TODO
  }

  // Create 2D circle (Voronoi)

  if (string("circle") == value) {
    // TODO
  }

  // Create 3D sphere

  if (string("cone") == value) {
    // TODO
  }

  // Create 3D cone

  if (string("cone") == value) {
    // TODO
  }

  // Create 3D prism

  if (string("prism") == value) {
    // TODO
  }

  // Read position, rotation and scale

  Vector3d vT(0, 0, 0);
  Matrix3d mR = Matrix3d::Identity();
  Matrix3d mS = Matrix3d::Identity();

  if (!ele.child("position").empty())
    if (!this->LoadVector3D(ele.child("position"), vT))
      return false;

  if (!ele.child("scale").empty())
    if (!this->LoadScale3DVector(ele.child("scale"), mS))
      return false;

  if (!ele.child("rotation").empty())
    if (!this->LoadRotation3DEuler(ele.child("rotation"), mR))
      return false;

  // Transform mesh

  mV = ((mR * mS * mV.transpose()).colwise() + vT).transpose();

  return true;
}

bool Serializer_XML::LoadMesh_TetVolume(const xml_node& ele,
                                        MatrixXd& mV,
                                        MatrixXi& mF,
                                        MatrixXi& mE) {
  if (!this->CheckNodeAttrr(ele, "type", false))
    return false;

  string typeValue = ele.attribute("type").as_string();

  // Defined as quadrangular surface + tetrahedralization

  if (typeValue == "FILESURFACE") {
    if (!this->CheckNodeChild(ele, "path", false))
      return false;

    string meshPath = IOUtils::getAbsolutePath(
        ele.attribute("path").as_string(), this->m_fileDire);

    MatrixXd mVs;
    MatrixXi mFs;
    MatrixXd mNs;
    MatrixXd mHs;

    if (!Serializer_Mesh::ReadSurface_Any(meshPath, mVs, mFs, mNs)) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Invalid mesh file: %s", meshPath);
      return false;
    }

    if (ele.attribute("meshing").empty()) {
      if (!MeshUtils::tetrahedralize(mVs, mFs, mHs, mV, mF, mE)) {
        IOUtils::logTrace(Verbosity::V1_Default,
                          "\n[ERROR] Impossible to tetrahedralize");
        return false;
      }
    } else {
      string meshingstr = ele.attribute("meshing").as_string();

      if (!MeshUtils::tetrahedralize(mVs, mFs, mHs, mV, mF, mE, meshingstr)) {
        IOUtils::logTrace(Verbosity::V1_Default,
                          "\n[ERROR] Impossible to tetrahedralize");
        return false;
      }
    }

    return true;
  }

  // Defined as a tetrahedral volume in a given format

  if (typeValue == "FILEVOLUME") {
    if (!this->CheckNodeAttrr(ele, "path", false))
      return false;

    string meshPath = IOUtils::getAbsolutePath(
        ele.attribute("mesh").as_string(), this->m_fileDire);

    if (!Serializer_Mesh::ReadVolume_TET(meshPath, mV, mE, mF)) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Invalid mesh file: %s", meshPath);
      return false;
    }
  }

  // Create 3D sphere

  if (typeValue == "SPHERE") {
    // TODO
  }

  // Create 3D cone

  if (typeValue == "CONE") {
    // TODO
  }

  // Create 3D prism

  if (typeValue == "Grid") {
    // TODO
  }

  // Read position, rotation and scale

  Vector3d vT(0, 0, 0);
  Matrix3d mR = Matrix3d::Identity();
  Matrix3d mS = Matrix3d::Identity();

  if (!ele.child("position").empty())
    if (!this->LoadVector3D(ele.child("position"), vT))
      return false;

  if (!ele.child("scale").empty())
    if (!this->LoadScale3DVector(ele.child("scale"), mS))
      return false;

  if (!ele.child("rotation").empty())
    if (!this->LoadRotation3DEuler(ele.child("rotation"), mR))
      return false;

  // Transform mesh

  mV = ((mR * mS * mV.transpose()).colwise() + vT).transpose();

  return true;
}

bool Serializer_XML::LoadMesh_HexVolume(const xml_node& ele,
                                        MatrixXd& mV,
                                        MatrixXi& mF,
                                        MatrixXi& mE) {
  if (!this->CheckNodeAttrr(ele, "type", false))
    return false;

  string typeValue = ele.attribute("type").as_string();

  // Defined as quadrangular surface + hexahedralization

  if (typeValue == "FILESURFACE") {
    if (!this->CheckNodeChild(ele, "path", false))
      return false;

    // TODO
  }

  // Defined as an hexahedral volume in a given format

  if (typeValue == "FILEVOLUME") {
    if (!this->CheckNodeChild(ele, "path", false))
      return false;

    string meshPath = IOUtils::getAbsolutePath(
        ele.attribute("mesh").as_string(), this->m_fileDire);

    MatrixXi mFdummy;
    MatrixXd mNdummy;
    VectorXi vFdummy;

    if (!Serializer_Mesh::ReadVolume_HEX(meshPath, mV, mE, vFdummy, mFdummy,
                                         mNdummy)) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Invalid mesh file: %s", meshPath);
      return false;
    }
  }

  // Create a volumetric regular prism

  if (typeValue == "GRID") {
    if (!this->CheckNodeChild(ele, "dims", false))
      return false;
    if (!this->CheckNodeChild(ele, "size", false))
      return false;

    Vector3d vdims;
    Vector3d vsize;
    if (!this->LoadVector3D(ele.child("dims"), vdims))
      return false;
    if (!this->LoadVector3D(ele.child("size"), vsize))
      return false;

    MeshUtils::create3DGridNodes((int)vdims[0], (int)vdims[1], (int)vdims[2],
                                 vsize, mV);
    MeshUtils::create3DGridIndex_Hex8(vdims[0], vdims[1], vdims[2], mE);
    MeshUtils::create3DGridIndex_Tri3(vdims[0], vdims[1], vdims[2], mF);

    return true;
  }

  // Create a volumetric sphere

  if (typeValue == "SPHERE") {
    // TODO
  }

  // Create a volumetric cone

  if (typeValue == "CONE") {
    // TODO
  }

  // Read position, rotation and scale

  Vector3d vT(0, 0, 0);
  Matrix3d mR = Matrix3d::Identity();
  Matrix3d mS = Matrix3d::Identity();

  if (!ele.child("position").empty())
    if (!this->LoadVector3D(ele.child("position"), vT))
      return false;

  if (!ele.child("scale").empty())
    if (!this->LoadScale3DVector(ele.child("scale"), mS))
      return false;

  if (!ele.child("rotation").empty())
    if (!this->LoadRotation3DEuler(ele.child("rotation"), mR))
      return false;

  // Transform mesh

  mV = ((mR * mS * mV.transpose()).colwise() + vT).transpose();

  return true;
}

bool Serializer_XML::LoadSimulable_ThinShell(const xml_node& ele,
                                             PtrS<Simulable>& pModel) {
  // if (this->CheckNodeChild(ele, "meshInit", false)) return false;
  // if (this->CheckNodeChild(ele, "material", false)) return false;
  // if (!this->CheckNodeAttrr(ele, "discretization", false)) return false;
  // if (!this->CheckNodeAttrr(ele, "quadrature", false)) return false;
  // if (!this->CheckNodeAttrr(ele, "model", false)) return false;

  // MatrixXd mV_X, mV_0;
  // MatrixXi mF_X, mF_0;
  // MatrixXd mN_X, mN_0;

  // if (!this->LoadMesh_Surface(ele.child("meshInit"), mV_X, mF_X, mN_X))
  //{
  //	IOUtils::logTrace(Verbosity::V1_Default, "\n[ERROR] Invalid input
  // triangle mesh (world)"); 	return false;
  //}

  // if (ele.child("meshRest").empty())
  //{
  //	mV_0 = mV_X;
  //	mF_0 = mF_X;
  //	mN_0 = mN_X;
  //}
  // else
  //{
  //	if (!this->LoadMesh_Surface(ele.child("meshRest"), mV_0, mF_0, mN_0))
  //	{
  //		IOUtils::logTrace(Verbosity::V1_Default, "\n[ERROR] Invalid
  // input triangle mesh (rest)"); 		return false;
  //	}

  //	// Assert compatibility

  //	assert(mF_X == mF_0);
  //}

  //// Create Thin Shell model

  // Simulable_ThinShell* pModelThinShell = new Simulable_ThinShell();

  //// Set input mesh

  // Discretization dis;
  // bool success = this->LoadDiscretization(ele.attribute("discretization"),
  // dis); if (!success || (dis != Discretization::Tri3 && dis !=
  // Discretization::Tri6))
  //{
  //	IOUtils::logTrace(Verbosity::V1_Default, "\n[ERROR] Invalid simulation
  // mesh
  // discretization. Expected Tri3 or Tri6"); 	return false;
  //}

  // vector<Tag> vnTraits(3);
  // vnTraits[0] = Tag::Position_X; // World configuration
  // vnTraits[1] = Tag::Position_0; // Rest configuration
  // vnTraits[2] = Tag::Velocity; // Rest configuration
  // Mesh_Tri* pInputMesh = new Mesh_Tri(mV_X, mF_X, dis, vnTraits);

  //// Set rest and vel configuration

  // pInputMesh->SetNodesTrait(mV_0, Tag::Position_0);
  // pInputMesh->SetNodesTrait(0 * mV_0, Tag::Velocity);

  // pModelThinShell->SetupOptions().m_pMesh.reset(pInputMesh);

  //// Set material parameters

  // ParameterSet material;
  // if (!this->LoadMaterialParameters(ele.child("material"), material))
  //{
  //	IOUtils::logTrace(Verbosity::V1_Default, "\n[ERROR] Invalid material
  // definition"); 	return false;
  //}

  // pModelThinShell->SetupOptions().m_material = material;

  //// Set FEM material model

  // PtrS<IMaterialModel> pFEMModel;
  // if (!this->LoadMaterialSimulable_FEM2Din3D(ele.attribute("model"),
  // pFEMModel))
  //{
  //	IOUtils::logTrace(Verbosity::V1_Default, "\n[ERROR] Invalid FEM model
  // definition"); 	return false;
  //}

  // pModelThinShell->SetupOptions().m_vpMatModels.push_back(pFEMModel);

  //// Set FEM quadrature

  // switch (dis)
  //{
  // case Discretization::Tri3:
  // case Discretization::Tri6:
  //	pModelThinShell->SetupOptions().m_vpQuadratures.push_back(Quadrature_Tri1::Instance());
  // break; 	break;

  // case Discretization::Quad4:
  // case Discretization::Quad8:
  //	pModelThinShell->SetupOptions().m_vpQuadratures.push_back(PtrS<IQuadrature>(new
  // Quadrature_Gauss(2, ele.attribute("quadrature").as_int()))); 	break;
  //}

  //// Initialize model

  // pModelThinShell->Setup();

  //// Reset output

  // pModel.reset(pModelThinShell);

  return true;
}

bool Serializer_XML::LoadSimulable_VolumeFEM(const xml_node& ele,
                                             PtrS<Simulable>& pModel) {
  if (!this->CheckNodeChild(ele, "meshInit", false))
    return false;
  if (!this->CheckNodeChild(ele, "discretization", false))
    return false;
  if (!this->CheckNodeChild(ele, "shapeFunction", false))
    return false;
  if (!this->CheckNodeChild(ele, "materialParam", false))
    return false;
  if (!this->CheckNodeChild(ele, "materialModel", false))
    return false;
  if (!this->CheckNodeChild(ele, "quadrature", false))
    return false;

  // Create Volume FEM model

  PtrS<Simulable_FEM_Volume> pModelVolumeFEM(new Simulable_FEM_Volume());

  //////////////////////////////////////////// LOAD DISCRETIZATION
  ///////////////////////////////////////////////

  Discretization dis;
  bool success = this->LoadDiscretization(ele.child("discretization"), dis);
  if (!success ||
      (dis != Discretization::Tet4 && dis != Discretization::Tet10 &&
       dis != Discretization::Hex8 && dis != Discretization::Hex20)) {
    IOUtils::logTrace(
        Verbosity::V1_Default,
        "\n[ERROR] Invalid simulation mesh discretization. Expected "
        "[Tet4|Tet10|Hex8|Hex20]");
    return false;
  }

  this->LoadDistribution<ParameterSet>(
      ele.child("materialParam"), pModelVolumeFEM->SetupOptions().m_pMatParams);
  this->LoadDistribution<IMaterialModel>(
      ele.child("materialModel"), pModelVolumeFEM->SetupOptions().m_pMatModels);
  this->LoadDistribution<IShapeFunction>(
      ele.child("shapeFunction"),
      pModelVolumeFEM->SetupOptions().m_pShapeFunctions);
  this->LoadDistribution<IQuadrature>(
      ele.child("quadrature"), pModelVolumeFEM->SetupOptions().m_pQuadratures);

  //////////////////////////////////////////// LOAD INITIAL MESH
  ///////////////////////////////////////////////

  vector<Tag> vnTraits(3);
  vnTraits[0] = Tag::Position_0;  // Rest positions
  vnTraits[1] = Tag::Position_X;  // World positions
  vnTraits[2] = Tag::Velocity;    // World Velocity
  Mesh_Cell* pInputMesh = NULL;

  MatrixXd mV_X, mV_0;
  MatrixXi mE_X, mE_0;
  MatrixXi mF_X, mF_0;

  if (dis == Discretization::Tet4 || dis == Discretization::Tet10) {
    if (!this->LoadMesh_TetVolume(ele.child("meshInit"), mV_X, mF_X, mE_X)) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Invalid input volume mesh (world)");
      return false;
    }

    if (ele.child("meshRest").empty()) {
      mV_0 = mV_X;
      mF_0 = mF_X;
      mE_0 = mE_X;
    } else {
      if (!this->LoadMesh_TetVolume(ele.child("meshRest"), mV_0, mF_0, mE_0)) {
        IOUtils::logTrace(Verbosity::V1_Default,
                          "\n[ERROR] Invalid input volume mesh (rest)");
        return false;
      }

      // Assert compatibility

      assert(mF_X == mF_0);
      assert(mE_X == mE_0);
    }

    pInputMesh = new Mesh_Tetra(mV_X, mE_X, dis, vnTraits);
  }

  if (dis == Discretization::Hex8 || dis == Discretization::Hex20) {
    if (!this->LoadMesh_HexVolume(ele.child("meshInit"), mV_X, mF_X, mE_X)) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Invalid input volume mesh (world)");
      return false;
    }

    if (ele.child("meshRest").empty()) {
      mV_0 = mV_X;
      mF_0 = mF_X;
      mE_0 = mE_X;
    } else {
      if (!this->LoadMesh_HexVolume(ele.child("meshRest"), mV_0, mF_0, mE_0)) {
        IOUtils::logTrace(Verbosity::V1_Default,
                          "\n[ERROR] Invalid input volume mesh (rest)");
        return false;
      }

      // Assert compatibility

      assert(mF_X == mF_0);
      assert(mE_X == mE_0);
    }

    pInputMesh = new Mesh_Hexa(mV_X, mE_X, dis, vnTraits);
  }

  pInputMesh->SetNodesTrait(mV_0, Tag::Position_0);
  pInputMesh->Scale(Vector3d::Zero(), Tag::Velocity);
  pInputMesh->Traits().AddTrait(Tag::SurfaceMesh_0, mF_X);

  pModelVolumeFEM->SetupOptions().m_pMesh.reset(pInputMesh);

  // Initialize model

  pModelVolumeFEM->Setup();

  // Reset output

  // pModel.reset(pModelVolumeFEM);

  return true;
}

bool Serializer_XML::LoadOptimProblem(const xml_node& ele,
                                      ISimulable* pModel,
                                      PtrS<OptimProblem>& pProblem) {
  if (!this->CheckNodeAttrr(ele, "type", false))
    return false;

  string typevalue = ele.attribute("type").as_string();

  if (typevalue.compare("Static") == 0) {
    pProblem = PtrS<OptimProblem>(new OptimProblem_BasicStatic(pModel));
    return true;
  }

  if (typevalue.compare("Dynamic") == 0) {
    if (!CheckNodeAttrr(ele, "step", false))
      return false;
    double timeStep = ele.attribute("step").as_double();

    pProblem =
        PtrS<OptimProblem>(new OptimProblem_BasicDynamic(pModel, timeStep));
    return true;
  }

  this->InvalidAttriValue(ele.attribute("type"),
                          "Problen type (Static, Dynamic...)", false);

  return false;
}

bool Serializer_XML::LoadOptimSolver(const xml_node& ele,
                                     IOptimProblem* pProblem,
                                     PtrS<OptimSolver>& pSolver) {
  OptimSolverOptions options;

  if (!ele.attribute("maxIters").empty())
    options.numMaxIters = ele.attribute("maxIters").as_int();

  if (!ele.attribute("maxError").empty())
    options.tolMaxError = ele.attribute("maxError").as_double();

  if (!ele.attribute("qpStep").empty()) {
    bool found = false;
    string value = ele.attribute("qpStep").as_string();
    if (!found && value.compare("Steepest") == 0) {
      found = true;
      options.qpSolverType = QPSolverType::Steepest;
    }
    if (!found && value.compare("Newton") == 0) {
      found = true;
      options.qpSolverType = QPSolverType::Newton;
    }
    if (!found && value.compare("BFGS_D") == 0) {
      found = true;
      options.qpSolverType = QPSolverType::BFGS_D;
    }
    if (!found && value.compare("BFGS_I") == 0) {
      found = true;
      options.qpSolverType = QPSolverType::BFGS_I;
    }
    if (!found && value.compare("LBFGS") == 0) {
      found = true;
      options.qpSolverType = QPSolverType::LBFGS;
    }
    if (!found && value.compare("Gauss") == 0) {
      found = true;
      options.qpSolverType = QPSolverType::Gauss;
    }

    if (!found) {
      this->InvalidAttriValue(ele.attribute("qpStep"),
                              "QP Step (Newton, LBFGS..)", false);
      return false;
    }
  }

  if (!ele.child("linear").empty()) {
    xml_node eleLinear = ele.child("linear");

    if (!eleLinear.attribute("type").empty()) {
      bool found = false;

      string value = eleLinear.attribute("type").as_string();
      if (value.compare("CUDASC") == 0) {
        options.lsSolverType = LSSolverType::CUDASC;
        found = true;
      }
      if (value.compare("EigenCG") == 0) {
        options.lsSolverType = LSSolverType::EigenCG;
        found = true;
      }
      if (value.compare("EigenLU") == 0) {
        options.lsSolverType = LSSolverType::EigenLU;
        found = true;
      }
      if (value.compare("EigenLDLT") == 0) {
        options.lsSolverType = LSSolverType::EigenLDLT;
        found = true;
      }
      if (value.compare("SSparseSPQR") == 0) {
        options.lsSolverType = LSSolverType::SSparseSPQR;
        found = true;
      }
      if (value.compare("CholmodLDLT") == 0) {
        options.lsSolverType = LSSolverType::CholmodLDLT;
        found = true;
      }
      if (value.compare("BiCGSTAB") == 0) {
        options.lsSolverType = LSSolverType::BiCGSTAB;
        found = true;
      }

      if (!found) {
        this->InvalidAttriValue(
            eleLinear.attribute("type"),
            "Linear solver (LSSolverType::EigenCG, LSSolverType::EigenLDLT...)",
            false);
        return false;
      }
    }

    if (!eleLinear.attribute("maxIters").empty())
      options.lsNumMaxIters = eleLinear.attribute("maxIters").as_int();

    if (!eleLinear.attribute("maxError").empty())
      options.lsTolMaxError = eleLinear.attribute("maxError").as_double();
  }

  if (!ele.child("lsearch").empty()) {
    xml_node eleLsearch = ele.child("lsearch");

    if (!eleLsearch.attribute("type").empty()) {
      bool found = false;

      string value = eleLsearch.attribute("type").as_string();
      if (value.compare("None") == 0) {
        options.lSearchType = LSearchType::None;
        found = true;
      }
      if (value.compare("Simple") == 0) {
        options.lSearchType = LSearchType::Simple;
        found = true;
      }
      if (value.compare("Armijo") == 0) {
        options.lSearchType = LSearchType::Armijo;
        found = true;
      }
      if (value.compare("WolfeWeak") == 0) {
        options.lSearchType = LSearchType::WolfeWeak;
        found = true;
      }
      if (value.compare("WolfeStrong") == 0) {
        options.lSearchType = LSearchType::WolfeStrong;
        found = true;
      }

      if (!found) {
        this->InvalidAttriValue(eleLsearch.attribute("type"),
                                "Line search (Simple, Armijo...)", false);
        return false;
      }
    }

    if (!eleLsearch.attribute("iters").empty())
      options.lsearch_numTry = eleLsearch.attribute("iters").as_int();

    if (!eleLsearch.attribute("alpha").empty())
      options.lSearch_alpha = eleLsearch.attribute("alpha").as_double();

    if (!eleLsearch.attribute("wolfe1").empty())
      options.lSearch_wolfe1 = eleLsearch.attribute("wolfe1").as_double();

    if (!eleLsearch.attribute("wolfe2").empty())
      options.lSearch_wolfe2 = eleLsearch.attribute("wolfe2").as_double();
  }

  pSolver = PtrS<OptimSolver>(new OptimSolver_USQP_LS(pProblem, options));

  return true;
}

bool Serializer_XML::LoadBoundary(const xml_node& ele,
                                  Simulable* pModel,
                                  vector<PtrS<IBCondition>>& vpBC) {
  pugi::xml_named_node_iterator itCur = ele.children("bc").begin();
  pugi::xml_named_node_iterator itEnd = ele.children("bc").end();
  for (; itCur != itEnd; ++itCur) {
    PtrS<IBCondition> pBC = NULL;
    if (!this->LoadBoundary(*itCur, pModel, pBC)) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Invalid boundary condition");
      return false;
    }
    vpBC.push_back(pBC);
  }

  return true;
}

bool Serializer_XML::LoadBoundary(const xml_node& ele,
                                  Simulable* pModel,
                                  PtrS<IBCondition>& pBC) {
  if (!this->CheckNodeAttrr(ele, "type", false))
    return false;

  PtrS<BCondition> pBCObject;

  string value = ele.attribute("type").as_string();

  bool found = false;

  if (value.compare("Gravity") == 0) {
    Vector3d vg(0.0, -9.8, 0.0);

    if (!ele.child("gravity").empty()) {
      if (!this->LoadVector3D(ele.child("gravity"), vg))
        IOUtils::logTrace(Verbosity::V1_Default,
                          "[WARNING] Invalid acceleration vector");
    }

    pBCObject.reset(new BC_Gravity(pModel, vg));

    found = true;
  }

  if (value.compare("FixPosition") == 0) {
    if (!this->CheckNodeAttrr(ele, "soft", false))
      return false;
    if (!this->CheckNodeChild(ele, "selection", false))
      return false;

    // Selection

    PtrS<GeometryFilter> pGF;

    if (!this->LoadGeometryFilter(ele.child("selection"), pGF))
      return false;

    vector<Geometry*> vgeom = pGF->Filter(pModel->Geometries());
    vector<IDoFSet*> vDoF((int)vgeom.size());
    vector<VectorXd> vval((int)vgeom.size());
    for (int i = 0; i < (int)vgeom.size(); ++i) {
      vDoF[i] = vgeom[i]->Traits().Kinematics(Tag::DOF_0);
      vval[i] = vDoF[i]->GetValue();
    }

    pBCObject.reset(
        new BC_FixDoF(pModel, vDoF, vval, ele.attribute("soft").as_bool()));

    if (this->CheckNodeAttrr(ele, "numStep", false)) {
      pBCObject->m_numStage = ele.attribute("numStep").as_int();
    }

    if (!ele.child("animation").empty()) {
      PtrS<IVectorAnimator> pVA;

      if (!this->LoadVectorAnimator(ele.child("animation"), pVA))
        return false;

      pVA->SetInitial(vval);

      pBCObject->m_pAnimator = pVA;
    }

    found = true;
  }

  if (value.compare("AlignFace") == 0) {
    found = true;
  }

  if (value.compare("CollPlane") == 0) {
    if (!this->CheckNodeAttrr(ele, "soft", false))
      return false;
    if (!this->CheckNodeChild(ele, "point", false))
      return false;
    if (!this->CheckNodeChild(ele, "normal", false))
      return false;

    Vector3d vp;
    Vector3d vn;
    if (!this->LoadVector3D(ele.child("point"), vp)) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Invalid collision plane point");
      return false;
    }
    if (!this->LoadVector3D(ele.child("normal"), vn)) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Invalid collision plane point");
      return false;
    }

    double tol = 0;
    if (!ele.attribute("tolerance").empty())
      tol = ele.attribute("tolerance").as_double();

    bool soft = true;
    if (!ele.attribute("soft").empty())
      soft = ele.attribute("soft").as_bool();

    pBCObject.reset(new BC_PlaneColl(pModel, vp, vn, tol, soft));

    found = true;
  }

  if (value.compare("Force") == 0) {
    found = true;
  }

  if (!found) {
    this->InvalidAttriValue(ele.attribute("type"),
                            "BC type (Gravity, FixPosition...)", false);
    return false;
  }

  if (!ele.attribute("numStep").empty())
    pBCObject->NumStage() = ele.attribute("numStep").as_int();

  pBC = pBCObject;

  return true;
}

bool Serializer_XML::LoadVectorAnimator(const xml_node& ele,
                                        PtrS<IVectorAnimator>& pVA) {
  if (!this->CheckNodeAttrr(ele, "type", false))
    return false;

  string value = ele.attribute("type").as_string();

  if (value.compare("Transform3D") == 0) {
    Vector3d vT;
    Vector3d vR;
    Vector3d vS;
    if (this->CheckNodeChild(ele, "position", false))
      this->LoadVector3D(ele.child("position"), vT);
    else
      vT = Vector3d(0.0, 0.0, 0.0);

    if (this->CheckNodeChild(ele, "rotation", false))
      this->LoadVector3D(ele.child("rotation"), vR);
    else
      vR = Vector3d(0.0, 0.0, 0.0);

    if (this->CheckNodeChild(ele, "scale", false))
      this->LoadVector3D(ele.child("scale"), vS);
    else
      vS = Vector3d(1.0, 1.0, 1.0);

    pVA.reset(new VectorAnimator_LerpT3D(vT, vR, vS));

    return true;
  }

  // string& seltypestr = ele["type"].get<string>();

  //// Constant heterogeneous typeValue

  // if (seltypestr.compare("CONSTVAL") == 0)
  //{
  //	VectorXd vval;
  //	LoadVectorXd(ele["typeValue"], vval);
  //	bcSetup.m_pAnimator.reset(new VectorAnimator_Constant(1, vval));

  //	return true;
  //}

  // if (seltypestr.compare("CONSTVEC") == 0)
  //{
  //	vector<VectorXd> vvals;
  //	LoadVectorValues(ele["field"], pModel, bcSetup.m_vGeometry, vvals);
  //	bcSetup.m_pAnimator.reset(new VectorAnimator_Constant(vvals));

  //	return true;
  //}

  //// Linear interpolation of values

  // if (seltypestr.compare("LERP_VAL") == 0)
  //{
  //	vector<VectorXd> vvalsIni;
  //	vector<VectorXd> vvalsEnd;
  //	LoadVectorValues(ele["fieldIni"], pModel, bcSetup.m_vGeometry,
  // vvalsIni); 	LoadVectorValues(ele["fieldEnd"], pModel,
  // bcSetup.m_vGeometry, vvalsEnd); 	bcSetup.m_pAnimator.reset(new
  // VectorAnimator_LerpValues(vvalsIni, vvalsEnd));

  //	return true;
  //}

  //// Linear interpolation of 2D transform

  // if (seltypestr.compare("LERP_T2D") == 0)
  //{
  //	vector<VectorXd> vvalsIni;
  //	Vector2d vT;
  //	Vector2d vR;
  //	Vector2d vS;
  //	this->LoadVector2D(ele["position"], vT);
  //	this->LoadVector2D(ele["rotation"], vR);
  //	this->LoadVector2D(ele["scale"], vS);
  //	LoadVectorValues(ele["fieldIni"], pModel, bcSetup.m_vGeometry,
  // vvalsIni); 	bcSetup.m_pAnimator.reset(new
  // VectorAnimator_LerpT2D(vvalsIni, vT, vR, vS));

  //	return true;
  //}

  //// Linear interpolation of 3D transform

  // if (seltypestr.compare("LERP_T3D") == 0)
  //{
  //	vector<VectorXd> vvalsIni;
  //	Vector3d vT;
  //	Vector3d vR;
  //	Vector3d vS;
  //	this->LoadVector3D(ele["position"], vT);
  //	this->LoadVector3D(ele["rotation"], vR);
  //	this->LoadVector3D(ele["scale"], vS);
  //	LoadVectorValues(ele["fieldIni"], pModel, bcSetup.m_vGeometry,
  // vvalsIni); 	bcSetup.m_pAnimator.reset(new
  // VectorAnimator_LerpT3D(vvalsIni, vT, vR, vS));

  //	return true;
  //}

  // IOUtils::logTrace(Verbosity::V1_Default, "\n[ERROR] Invalid animation
  // options");

  return false;
}

bool Serializer_XML::LoadGeometryFilter(const xml_node& ele,
                                        PtrS<GeometryFilter>& pGF) {
  if (!this->CheckNodeAttrr(ele, "type", false))
    return false;

  string value = ele.attribute("type").as_string();

  if (value.compare("AllNodes") == 0) {
    pGF.reset(new GeometryFilter_AllNode());
    return true;
  }
  if (value.compare("AllElems") == 0) {
    pGF.reset(new GeometryFilter_AllElem());
    return true;
  }
  if (value.compare("BoundaryNodes") == 0) {
    pGF.reset(new GeometryFilter_BoundaryNode());
    return true;
  }
  if (value.compare("BoundaryEdges") == 0) {
    pGF.reset(new GeometryFilter_BoundaryEdge());
    return true;
  }
  if (value.compare("BoundaryFaces") == 0) {
    if (!this->CheckNodeAttrr(ele, "useNodes", true))
      pGF.reset(new GeometryFilter_BoundaryFace(true));
    else {
      bool useNodes = ele.attribute("useNodes").as_bool();
      pGF.reset(new GeometryFilter_BoundaryFace(useNodes));
    }

    return true;
  }

  if (value.compare("IndexNodes") == 0 || value.compare("IndexElems") == 0) {
    if (!this->CheckNodeChild(ele, "indices", false))
      return false;

    VectorXd vidxD;
    if (!LoadVectorXd(ele.child("indices"), vidxD))
      return false;

    iVector vidxI(vidxD.size());
    for (int i = 0; i < vidxD.size(); ++i)
      vidxI[i] = (int)vidxD(i);  // Cast

    if (value.compare("IndexNodes") == 0)
      pGF.reset(new GeometryFilter_IndexNode(vidxI));
    else
      pGF.reset(new GeometryFilter_IndexElem(vidxI));

    return true;
  }

  if (value.compare("BoxNodes") == 0 || value.compare("BoxElems") == 0) {
    if (!this->CheckNodeChild(ele, "minCorner", false))
      return false;
    if (!this->CheckNodeChild(ele, "maxCorner", false))
      return false;
    if (!this->CheckNodeAttrr(ele, "field", false))
      return false;

    Tag sema;
    VectorXd vminCorner;
    VectorXd vmaxCorner;
    if (!LoadVectorXd(ele.child("minCorner"), vminCorner))
      return false;
    if (!LoadVectorXd(ele.child("maxCorner"), vmaxCorner))
      return false;
    if (!LoadSemantics(ele.attribute("field"), sema))
      return false;

    if (value.compare("BoxNodes") == 0)
      pGF.reset(new GeometryFilter_BoxNodes(vminCorner, vmaxCorner, sema));
    else
      pGF.reset(new GeometryFilter_BoxElems(vminCorner, vmaxCorner, sema));

    return true;
  }

  if (value.compare("MeshNodes") == 0 || value.compare("MeshElems") == 0) {
    if (!this->CheckNodeChild(ele, "mesh", false))
      return false;
    if (!this->CheckNodeAttrr(ele, "field", false))
      return false;

    MatrixXd mV;
    MatrixXi mF;
    MatrixXd mN;
    if (!this->LoadMesh_Surface(ele.child("mesh"), mV, mF, mN)) {
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Invalid selection triangle mesh");
      return false;
    }

    Tag sema;
    VectorXd vminCorner = mV.colwise().minCoeff();
    VectorXd vmaxCorner = mV.colwise().maxCoeff();
    if (!LoadSemantics(ele.attribute("field"), sema))
      return false;

    if (value.compare("MeshNodes") == 0)
      pGF.reset(new GeometryFilter_BoxNodes(vminCorner, vmaxCorner, sema));
    else
      pGF.reset(new GeometryFilter_BoxElems(vminCorner, vmaxCorner, sema));

    return true;
  }

  return false;
}

bool Serializer_XML::CheckNodeAttrr(const xml_node& ele,
                                    const string& name,
                                    bool warning) {
  if (ele.attribute(name.c_str()).empty()) {
    if (warning)
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[WARNING] Expected node [%s] attribute: %s",
                        ele.name(), name);
    else
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Expected node [%s] attribute: %s",
                        ele.name(), name);
    return false;
  }

  return true;
}

bool Serializer_XML::CheckNodeChild(const xml_node& ele,
                                    const string& name,
                                    bool warning) {
  if (ele.child(name.c_str()).empty()) {
    if (warning)
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[WARNING] Expected node [%s] child: %s", ele.name(),
                        name);
    else
      IOUtils::logTrace(Verbosity::V1_Default,
                        "\n[ERROR] Expected node [%s] child: %s", ele.name(),
                        name);
    return false;
  }

  return true;
}

void Serializer_XML::InvalidNodeValue(const xml_node& ele,
                                      const string& expected,
                                      bool warning) {
  if (warning)
    IOUtils::logTrace(Verbosity::V1_Default,
                      "\n[WARNING] Invalid node [%s] value. Expected: %s",
                      ele.value(), expected);
  else
    IOUtils::logTrace(Verbosity::V1_Default,
                      "\n[ERROR] Invalid node [%s] attribute. Expected: %s",
                      ele.value(), expected);
}

void Serializer_XML::InvalidAttriValue(const xml_attr& att,
                                       const string& expected,
                                       bool warning) {
  if (warning)
    IOUtils::logTrace(Verbosity::V1_Default,
                      "\n[WARNING] Invalid node [%s] value. Expected: %s",
                      att.as_string(), expected);
  else
    IOUtils::logTrace(Verbosity::V1_Default,
                      "\n[ERROR] Invalid node [%s] attribute. Expected: %s",
                      att.as_string(), expected);
}

//	bool Serializer_XML::LoadBC_GeometryFilter(xml_node& ele, Simulable*
// pModel, BCSetup& bcSetup)
//	{
//		//string& seltypestr = ele["type"].get<string>();
//
//		//// Selection of all DoF within a box
//
//		//if (seltypestr.compare("BOXFILE") == 0 ||
// seltypestr.compare("BOXRANGE") == 0)
//		//{
//		//	string spacestr = ele["space"].get<string>();
//
//		//	Semantics space;
//		//	if (spacestr.compare("REST") == 0) space = Tag::Rest_X;
//		//	if (spacestr.compare("WORLD") == 0) space =
// Tag::World_X;
//
//		//	Vector3d vmin;
//		//	Vector3d vmax;
//
//		//	// Box obtained provinding a mesh file
//
//		//	if (seltypestr.compare("BOXFILE") == 0)
//		//	{
//		//		string filePath = ele["path"];
//
//		//		MatrixXd mV;
//		//		MatrixXi mF;
//		//		if (Serializer_Mesh::ReadSurface_OBJ(filePath,
// mV, mF))
//		//		{
//		//			Vector3d vmin;
//		//			Vector3d vmax;
//		//			computeBoundingBox(mV, vmin, vmax);
//		//		}
//		//	}
//
//		//	// Box obtained provinding the range
//
//		//	if (seltypestr.compare("BOXRANGE") == 0)
//		//	{
//		//		VectorXd vminIn;
//		//		VectorXd vmaxIn;
//		//		this->LoadVectorXd(ele["boxMin"], vminIn);
//		//		this->LoadVectorXd(ele["boxMax"], vmaxIn);
//		//		vmin = vminIn;
//		//		vmax = vmaxIn;
//		//	}
//
//		//	bcSetup.m_pSelector.reset(new
// GeometryFilter_BoxDoF(space, vmin, vmax));
//
//		//	return true;
//		//}
//
//		//// Selection of specific indices
//
//		//if (seltypestr.compare("INDICES") == 0)
//		//{
//		//	VectorXd vindicesIn;
//		//	iVector vindicesSTL;
//		//	LoadVectorXd(ele["indices"], vindicesIn);
//		//	int numIdx = (int)vindicesIn.size();
//
//		//	vindicesSTL.resize(numIdx);
//		//	for (int i = 0; i < numIdx; ++i)
//		//		vindicesSTL[i] = (int)vindicesIn[i];
//
//		//	bcSetup.m_pSelector.reset(new
// GeometryFilter_IndexNode(vindicesSTL));
//
//		//	return true;
//		//}
//
//		//// Selection of all DoF
//
//		//if (seltypestr.compare("ALL") == 0)
//		//{
//		//	bcSetup.m_pSelector.reset(new GeometryFilter_AllNode());
//
//		//	return true;
//		//}
//
//		//IOUtils::logTrace(Verbosity::V1_Default, "\n[ERROR] Invalid
// selection
// options");
//
//		return false;
//	}
//
//	bool Serializer_XML::LoadBC_VectorAnimator(xml_node& ele, Simulable*
// pModel, BCSetup& bcSetup)
//	{
//		//string& seltypestr = ele["type"].get<string>();
//
//		//// Constant heterogeneous typeValue
//
//		//if (seltypestr.compare("CONSTVAL") == 0)
//		//{
//		//	VectorXd vval;
//		//	LoadVectorXd(ele["typeValue"], vval);
//		//	bcSetup.m_pAnimator.reset(new VectorAnimator_Constant(1,
// vval));
//
//		//	return true;
//		//}
//
//		//if (seltypestr.compare("CONSTVEC") == 0)
//		//{
//		//	vector<VectorXd> vvals;
//		//	LoadVectorValues(ele["field"], pModel,
// bcSetup.m_vGeometry, vvals);
//		//	bcSetup.m_pAnimator.reset(new
// VectorAnimator_Constant(vvals));
//
//		//	return true;
//		//}
//
//		//// Linear interpolation of values
//
//		//if (seltypestr.compare("LERP_VAL") == 0)
//		//{
//		//	vector<VectorXd> vvalsIni;
//		//	vector<VectorXd> vvalsEnd;
//		//	LoadVectorValues(ele["fieldIni"], pModel,
// bcSetup.m_vGeometry, vvalsIni);
//		//	LoadVectorValues(ele["fieldEnd"], pModel,
// bcSetup.m_vGeometry, vvalsEnd);
//		//	bcSetup.m_pAnimator.reset(new
// VectorAnimator_LerpValues(vvalsIni, vvalsEnd));
//
//		//	return true;
//		//}
//
//		//// Linear interpolation of 2D transform
//
//		//if (seltypestr.compare("LERP_T2D") == 0)
//		//{
//		//	vector<VectorXd> vvalsIni;
//		//	Vector2d vT;
//		//	Vector2d vR;
//		//	Vector2d vS;
//		//	this->LoadVector2D(ele["position"], vT);
//		//	this->LoadVector2D(ele["rotation"], vR);
//		//	this->LoadVector2D(ele["scale"], vS);
//		//	LoadVectorValues(ele["fieldIni"], pModel,
// bcSetup.m_vGeometry, vvalsIni);
//		//	bcSetup.m_pAnimator.reset(new
// VectorAnimator_LerpT2D(vvalsIni, vT, vR, vS));
//
//		//	return true;
//		//}
//
//		//// Linear interpolation of 3D transform
//
//		//if (seltypestr.compare("LERP_T3D") == 0)
//		//{
//		//	vector<VectorXd> vvalsIni;
//		//	Vector3d vT;
//		//	Vector3d vR;
//		//	Vector3d vS;
//		//	this->LoadVector3D(ele["position"], vT);
//		//	this->LoadVector3D(ele["rotation"], vR);
//		//	this->LoadVector3D(ele["scale"], vS);
//		//	LoadVectorValues(ele["fieldIni"], pModel,
// bcSetup.m_vGeometry, vvalsIni);
//		//	bcSetup.m_pAnimator.reset(new
// VectorAnimator_LerpT3D(vvalsIni, vT, vR, vS));
//
//		//	return true;
//		//}
//
//		//IOUtils::logTrace(Verbosity::V1_Default, "\n[ERROR] Invalid
// animation
// options");
//
//		return false;
//	}
//
//	//bool Serializer_XML::LoadSimulableVector(xml_node& ele, Simulable*
// pModel, const vector<Geometry*>& vGeom, vector<VectorXd>& vval)
//	//{
//	//	//string fieldstr = ele.get<string>();
//
//	//	//int numVal = (int)vGeom.size();
//	//	//vval.resize(numVal);
//
//	//	//if (fieldstr.compare("WORLDPOS") == 0)
//	//	//{
//	//	//	for (int i = 0; i < numVal; ++i)
//	//	//		vval[i] =
// vGeom[i]->Traits().Kinematics(Semantics::Tag::DOF_0).GetPosition(Tag::Position_X);
//
//	//	//	return true;
//	//	//}
//
//	//	//if (fieldstr.compare("WORLDVEL") == 0)
//	//	//{
//	//	//	for (int i = 0; i < numVal; ++i)
//	//	//		vval[i] =
// vGeom[i]->Traits().Kinematics(Semantics::Tag::DOF_0).GetPosition(Tag::Velocity);
//
//	//	//	return true;
//	//	//}
//
//	//	//if (fieldstr.compare("RESTPOS") == 0)
//	//	//{
//	//	//	for (int i = 0; i < numVal; ++i)
//	//	//		vval[i] =
// vGeom[i]->Traits().Kinematics(Semantics::Tag::DOF_0).GetPosition(Tag::Position_0);
//
//	//	//	return true;
//	//	//}
//
//	//	//if (fieldstr.compare("RESTVEL") == 0)
//	//	//{
//	//	//	for (int i = 0; i < numVal; ++i)
//	//	//		vval[i] =
// vGeom[i]->Traits().Kinematics(Semantics::Tag::DOF_0).GetPosition(Tag::Velocity_0);
//
//	//	//	return true;
//	//	//}
//
//	//	//if (fieldstr.compare("GRADIENT") == 0)
//	//	//{
//	//	//	const VectorXd& vg = pModel->GetGradient();
//	//	//	for (int i = 0; i < numVal; ++i)
//	//	//	{
//	//	//		const DoFSet& dof =
// vGeom[i]->Traits().Kinematics(Semantics::Tag::DOF_0);
//	//	//		vval[i] = vg.block(dof.Offset(), 0,
// dof.NumDim(), 1);
//	//	//	}
//
//	//	//	return true;
//	//	//}
//
//	//	//if (fieldstr.compare("MASS") == 0)
//	//	//{
//	//	//	const VectorXd& vm = pModel->GetMass().diagonal();
//	//	//	for (int i = 0; i < numVal; ++i)
//	//	//	{
//	//	//		const DoFSet& dof =
// vGeom[i]->Traits().Kinematics(Semantics::Tag::DOF_0);
//	//	//		vval[i] = vm.block(dof.Offset(), 0,
// dof.NumDim(), 1);
//	//	//	}
//
//	//	//	return true;
//	//	//}
//
//	//	//if (fieldstr.compare("CSVFILE") == 0)
//	//	//{
//	//	//	// TODO
//
//	//	//	return true;
//	//	//}
//
//	//	//if (fieldstr.compare("MESHFILE") == 0)
//	//	//{
//	//	//	string meshPath = ele["path"].get<string>();
//	//	//	MatrixXd mV;
//	//	//	MatrixXi mF;
//	//	//	MatrixXd mN;
//	//	//	if (!Serializer_Mesh::ReadSurface_Any(meshPath, mV, mF,
// mN))
//	//	//	{
//	//	//		IOUtils::logTrace(Verbosity::V1_Default,
//"\n[ERROR] Invalid mesh file: %s", meshPath);
//	//	//		return false;
//	//	//	}
//
//	//	//	vval.resize(mV.rows());
//	//	//	for (int i = 0; i < mV.rows(); ++i)
//	//	//		vval[i] = mV.row(i);
//
//	//	//	return true;
//	//	//}
//
//	//	//IOUtils::logTrace(Verbosity::V1_Default, "\n[ERROR] Invalid
// vector
// values");
//
//	//	return false;
//	//}

}  // namespace PhySim

// bool Serializer_XML::LoadMaterialSimulable_FEM2Din3D(const xml_attr& att,
// PtrS<IMaterialModel>& pMat)
//{
//	bool found = false;
//	string value = att.as_string();
//	if (!found && value.compare("StVK") == 0) { pMat =
// MaterialModel_2Din3D_StVK::Instance();  found = true; } 	if (!found &&
// value.compare("CoNH") == 0) { pMat = MaterialModel_2Din3D_CoNH::Instance();
// found = true; } 	if (!found && value.compare("InNH") == 0) { pMat =
// MaterialModel_2Din3D_InNH::Instance(); found = true; }
//	//if (!found && typeValue.compare("CoMR") == 0) { pMat =
// MaterialModel_2Din3D_StVK::Instance(); found = true; }
//	//if (!found && typeValue.compare("InMR") == 0) { pMat =
// MaterialModel_2Din3D_StVK::Instance(); found = true; }
//	//if (!found && typeValue.compare("Ogden") == 0) { pMat =
// MaterialModel_2Din3D_StVK::Instance(); found = true; }
//	//if (!found && typeValue.compare("CoRot") == 0) { pMat =
// MaterialModel_2Din3D_StVK::Instance(); found = true; }
//
//	if (!found)
//	{
//		this->InvalidAttriValue(att, "Material model (StVK, CoNH...)",
// true);
//	}
//
//	return true;
//}
//
// bool Serializer_XML::LoadMaterialSimulable_FEM3Din3D(const xml_attr& att,
// PtrS<IMaterialModel>& pMat)
//{
//	bool found = false;
//	string value = att.as_string();
//	if (!found && value.compare("StVK") == 0) { pMat =
// MaterialModel_3Din3D_StVK::Instance();  found = true; } 	if (!found &&
// value.compare("CoNH") == 0) { pMat = MaterialModel_3Din3D_CoNH::Instance();
// found = true; } 	if (!found && value.compare("CoMR") == 0) { pMat =
// MaterialModel_3Din3D_CoMR::Instance(); found = true; }
//	//if (!found && typeValue.compare("CoMR") == 0) { pMat =
// MaterialModel_2Din3D_StVK::Instance(); found = true; }
//	//if (!found && typeValue.compare("InMR") == 0) { pMat =
// MaterialModel_2Din3D_StVK::Instance(); found = true; }
//	//if (!found && typeValue.compare("Ogden") == 0) { pMat =
// MaterialModel_2Din3D_StVK::Instance(); found = true; }
//	//if (!found && typeValue.compare("CoRot") == 0) { pMat =
// MaterialModel_2Din3D_StVK::Instance(); found = true; }
//
//	if (!found)
//	{
//		this->InvalidAttriValue(att, "Material model (StVK, CoNH...)",
// true);
//	}
//
//	return true;
//}
//
// bool Serializer_XML::LoadMaterialParameters(const xml_node& ele,
// ParameterSet& mat)
//{
//	pugi::xml_named_node_iterator itCur = ele.children("parameter").begin();
//	pugi::xml_named_node_iterator itEnd = ele.children("parameter").end();
//	for (; itCur != itEnd; itCur++)
//	{
//		bool found = false;
//
//		if (!this->CheckNodeAttrr(*itCur, "name", false)) return false;
//		if (!this->CheckNodeAttrr(*itCur, "value", false)) return false;
//
//		string name = itCur->attribute("name").as_string();
//		double value = itCur->attribute("value").as_double();
//		mat.AddParameter(name, value);
//	}
//
//	if (mat.HasParameter(ParameterSet::Param_Young) &&
//		mat.HasParameter(ParameterSet::Param_Poisson) &&
//		mat.HasParameter(ParameterSet::Param_Density))
//	{
//		mat.InitLinearFromYoungPoisson(
//			mat[ParameterSet::Param_Young],
//			mat[ParameterSet::Param_Poisson],
//			mat[ParameterSet::Param_Density]);
//		return true;
//	}
//
//	if (mat.HasParameter(ParameterSet::Param_Lame1) &&
//		mat.HasParameter(ParameterSet::Param_Lame2) &&
//		mat.HasParameter(ParameterSet::Param_Density))
//	{
//		mat.InitLinearFromLameParameters(
//			mat[ParameterSet::Param_Lame1],
//			mat[ParameterSet::Param_Lame2],
//			mat[ParameterSet::Param_Density]);
//		return true;
//	}
//
//	return true;
//}
//
// bool Serializer_XML::LoadShapeFunction(const xml_node& ele,
// vector<PtrS<IShapeFunction>>& vpShape)
//{
//	if (!this->CheckNodeAttrr(ele, "type", false)) return false;
//
//	string typeValue = ele.attribute("type").as_string();
//
//	// Heterogeneous distribution
//
//	if (typeValue == "VARIABLE")
//	{
//		// TODO
//
//		return true;
//	}
//
//	// Constant shape functions
//
//	vpShape.resize(1);
//
//
//
//	return false;
//}

// bool Serializer_XML::LoadMaterialSampler(const xml_node& ele, int dimSpace,
// int dimBasis, vector<PtrS<IMaterialModel>>& vpModel, vector<ParameterSet>&
// vParam)
//{
//	if (!this->CheckNodeAttrr(ele, "type", false)) return false;
//
//	string typeValue = ele.attribute("type").as_string();
//
//	// Heterogeneous distribution
//
//	if (typeValue == "VARIABLE")
//	{
//
//		return true;
//	}
//
//	if (typeValue == "CONSTANT")
//	{
//
//	}
//
//	// Constant shape functions
//
//	vpModel.resize(1);
//	vParam.resize(1);
//
//	if (dimSpace == 3 && dimBasis == 3)
//	{
//		if (typeValue == "STVK") { vpModel.back().reset(new
// MaterialModel_3Din3D_StVK()); return true; } 		if (typeValue ==
// "CONH") { vpModel.back().reset(new MaterialModel_3Din3D_CoNH()); return true;
// } 		if (typeValue == "COMR") { vpModel.back().reset(new
// MaterialModel_3Din3D_CoMR()); return true; }
//	}
//
//	if (dimSpace == 3 && dimBasis == 2)
//	{
//		if (typeValue == "STVK") { vpModel.back().reset(new
// MaterialModel_2Din3D_StVK()); return true; } 		if (typeValue ==
// "CONH") { vpModel.back().reset(new MaterialModel_2Din3D_CoNH()); return true;
// } 		if (typeValue == "InNH") { vpModel.back().reset(new
// MaterialModel_2Din3D_InNH()); return true; }
//	}
//
//	this->LoadMaterialParameters(ele, vParam.back());
//
//	// Numerically coarsened SF following the implementation by Chen et
//	// al. 2018: Numerical Coarsening using Discontinuous Shape Functions
//
//	if (typeValue == "HEX_COARSEN")
//	{
//		if (!this->CheckNodeAttrr(ele, "subdivisions", false)) return
// false; 		if (!this->CheckNodeChild(ele, "distribution", false))
// return false;
//
//		vector<PtrS<IMaterialModel>> vAllFEMMats;
//		vector<ParameterSet> vAllParams;
//
//		pugi::xml_named_node_iterator itCur =
// ele.children("material").begin(); 		pugi::xml_named_node_iterator
// itEnd = ele.children("material").end(); 		for (; itCur != itEnd;
// itCur++)
//		{
//			vector<PtrS<IMaterialModel>> vFEMMat(1);
//			vector<ParameterSet> vParam(1);
//			if (!this->LoadMaterialSampler(*itCur, dimSpace,
// dimBasis, vFEMMat, vParam)) return false;
//
//			vAllFEMMats.push_back(vFEMMat.back());
//			vAllParams.push_back(vParam.back());
//		}
//
//		xml_node& eleDist = ele.child("distribution");
//
//		MatrixXi mDist;
//
//		if (eleDist.attribute("path").empty())
//		{
//			//this->LoadMatrix<int,-1,-1>(eleDist, mDist);
//		}
//		else
//		{
//			string matrixPath =
// Utils::getAbsolutePath(eleDist.attribute("path").as_string(),
// this->m_fileDire);
//
//			//Utils::readMatrix_CSV<int,-1,-1>(matrixPath, mDist);
//		}
//
//		// Convert to format accepted by the FEM material
//
//		int numCoarElems = mDist.rows();
//		vpModel.resize(numCoarElems);
//
//		int numDiv = ele.attribute("subdivisions").as_int();
//		int numFineElems = pow(pow(2, numDiv), 3);
//
//		// For each coarse element, coarse and fine nodes
//
//		for (int i = 0; i < numCoarElems; ++i)
//		{
//			//vector<ParameterSet> vmats;
//			//vmnij.resize(numCoarElems);
//
//			//vmnij.resize(8);
//			//for (int j = 0; j < 8; ++j)
//			//{
//			//	vmnij[j].resize(numFineNodes);
//			//	for (int k = 0; k < numFineNodes; ++k)
//			//	{
//			//		for (int ii = 0; ii < 3; ii++)
//			//			for (int jj = 0; jj < 3; jj++)
//			//				vmnij[j][k](ii, jj) =
// mCoeff.row(i)(9 * numFineNodes * j + 9 * k + 3 * ii + jj);
//			//	}
//			//}
//
//			//vpModel[i].reset(new ShapeFunction_Chen(vmnij));
//		}
//	}
//
//	this->InvalidAttriValue(ele.attribute("type"), "Discretization type
//(TRI_LIN, TET_LIN, HEX_TRILIN, ...)", false);
//
//	return false;
//
//}

// bool Serializer_XML::LoadQuadrature(const xml_node& ele,
// vector<PtrS<IQuadrature>>& vpQuad)
//{
//	if (!this->CheckNodeAttrr(ele, "type", false)) return false;

//	string typeValue = ele.attribute("type").as_string();

//	// Heterogeneous distribution

//	if (typeValue == "VARIABLE")
//	{
//		// TODO

//		return true;
//	}

//	// Constant shape functions

//	vpQuad.resize(1);

//	if (typeValue == "TRI_1") { vpQuad.back().reset(new Quadrature_Tri1());
// return true; }

//	if (typeValue == "TET_1") { vpQuad.back().reset(new Quadrature_Tet1());
// return true; } 	if (typeValue == "TET_4") { vpQuad.back().reset(new
// Quadrature_Tet4()); return true; } 	if (typeValue == "TET_5") {
// vpQuad.back().reset(new Quadrature_Tet5()); return true; } 	if (typeValue ==
//"TET_8") { vpQuad.back().reset(new Quadrature_Tet8()); return true; }
// if (typeValue == "TET_11") { vpQuad.back().reset(new Quadrature_Tet11());
// return true; }

//	if (typeValue == "QUAD_CENTER") { vpQuad.back().reset(new
// Quadrature_Quad1()); return true; } 	if (typeValue == "HEX_CENTER") {
// vpQuad.back().reset(new Quadrature_Hex1()); return true; }

//	if (typeValue == "QUAD_GAUSS") { vpQuad.back().reset(new
// Quadrature_Gauss(2, ele.attribute("num").as_int())); return true; } 	if
//(typeValue == "HEX_GAUSS") { vpQuad.back().reset(new Quadrature_Gauss(3,
// ele.attribute("num").as_int())); return true; }

//	this->InvalidAttriValue(ele.attribute("type"), "Quadrature type (TRI_1,
// TET_1, HEX_GAUSS, ...)", false);

//	return false;
//}
