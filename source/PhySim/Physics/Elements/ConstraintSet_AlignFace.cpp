//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Elements/ConstraintSet_AlignFace.h>

#include <PhySim/Geometry/Polytopes/Face.h>
#include <PhySim/Geometry/Polytopes/Node.h>

#include <PhySim/Kinematics/KinematicsEle.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

ConstraintSet_AlignFace::ConstraintSet_AlignFace(Simulable* pModel,
                                                 Face* pFace,
                                                 const Vector3d& vn,
                                                 bool isSoft)
    : ConstraintSet(pModel, isSoft) {
  this->m_vDoFs.resize(3);
  this->m_vDoFs[0] = pFace->Nodes()[0]->Traits().Kinematics(Tag_DOF_0);
  this->m_vDoFs[1] = pFace->Nodes()[1]->Traits().Kinematics(Tag_DOF_0);
  this->m_vDoFs[2] = pFace->Nodes()[2]->Traits().Kinematics(Tag_DOF_0);

  this->m_vvalues.resize(3);
  this->m_mJacobian.resize(3, 3);
  this->m_vHessian.resize(0);

  this->m_type = Type::EQ;

  this->m_vn = vn;
}

ConstraintSet_AlignFace::~ConstraintSet_AlignFace(void) {
  // Nothing to do here...
}

void ConstraintSet_AlignFace::Init() {
  throw exception("Not implemented");
}

void ConstraintSet_AlignFace::ProjectConstraint() {
  throw exception("Not implemented");
}

void ConstraintSet_AlignFace::ComputeAndStore_Values() {
  throw exception("Not implemented");
}

void ConstraintSet_AlignFace::ComputeAndStore_Jacobian() {
  throw exception("Not implemented");
}
}  // namespace PhySim