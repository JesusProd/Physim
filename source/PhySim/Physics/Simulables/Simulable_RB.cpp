//==========================================================
//
//	PhySim library. Generic library for physical simulation.
//
//	Authors:
//			Jesus Perez Rodriguez, jesusprod @ GitHub
//
//==========================================================

#include <PhySim/Physics/Simulables/Simulable_RB.h>

#include <PhySim/Utils/IOUtils.h>
#include <PhySim/Utils/Serializer_Mesh.h>
#include <PhySim/Utils/Utils.h>

#include <PhySim/Kinematics/KEleParticle3D.h>
#include <PhySim/Kinematics/KEleRigidBody3D.h>
#include <PhySim/Kinematics/KMapRB2Particle3D.h>

#include <PhySim/Physics/Elements/EnergyElement_SixDJoint.h>
#include <PhySim/Physics/Elements/EnergyElement_SpringBetwNodes.h>
#include <PhySim/Physics/Elements/MassElement_Lumped.h>
#include <PhySim/Physics/Elements/MassElement_RigidBody.h>

namespace PhySim {
using namespace std;
using namespace Eigen;

Simulable_RB::Simulable_RB() : Simulable() {
  this->m_pOptions = NULL;
}

Simulable_RB::~Simulable_RB() {
  this->FreeInternal();

  if (this->m_pOptions != NULL)
    delete this->m_pOptions;
  this->m_pOptions = NULL;

#ifndef NDEBUG
  IOUtils::logTrace(Verbosity::V1_Default, "\n[DEBUG] Deleting Simulable_RB");
#endif
}

Simulable_RB::Options& Simulable_RB::SetupOptions() {
  if (this->m_pOptions == NULL)  // Create if needed
    this->m_pOptions = new Simulable_RB::Options();
  return *((Simulable_RB::Options*)this->m_pOptions);
}

void Simulable_RB::FreeInternal() {
  Simulable::FreeInternal();
}

void Simulable_RB::InitInternal() {
  int numRB = (int)this->m_pOptions->m_vpGeometries.size();

  // Create RB kinematics

  vector<PtrS<KinematicsEle>> vpEle(numRB);

  for (int i = 0; i < numRB; ++i) {
    PtrS<Geometry> pGeom = this->m_pOptions->m_vpGeometries[i];

    PtrS<KEleRigidBody3D> pRB(new KEleRigidBody3D(this, pGeom.get()));
    pRB->Initialize();
    pGeom->Traits().AddTrait<IDoFSet*>(Tag::DOF_0, pRB.get());
    vpEle[i] = dynamic_pointer_cast<KinematicsEle>(pRB);

    // Add particle kinematics

    vector<PtrS<KinematicsEle>> vpPar(pGeom->NumNodes());

    int gg = pGeom->NumNodes();

    for (int j = 0; j < pGeom->NumNodes(); ++j) {
      if (pGeom->DimSpace() == 3) {
        vpPar[j].reset(new KEleParticle3D(this, pGeom->Nodes()[j]));
        vpPar[j]->Initialize();
      }

      pGeom->Nodes()[j]->Traits().AddTrait<IDoFSet*>(Tag::DOF_0,
                                                     vpPar[j].get());
    }
    KMapRB2Particle3D* pMapRB = new KMapRB2Particle3D();
    // PtrS<KMapRB2Particle3D> pMapRB = make_shared<KMapRB2Particle3D>();

    pMapRB->Init(this, pRB, vpPar);
  }

  this->m_pTree.reset(new KinematicsTree(this, vpEle));

  // Create energy elements
  // int numEneElem = this->m_pOptions->m_couplings.rows();

  // int numEneElem = (int)this->m_pOptions->m_springs.size();
  /// Here put a spring between specific nodes
  // this->m_venerEle.resize(numEneElem);
  // for (int i = 0; i < numEneElem; ++i)
  //{
  //	PtrS<Geometry> pGeom0 =
  // this->m_pOptions->m_vpGeometries[m_pOptions->m_couplings(i,0)];
  //	PtrS<Geometry> pGeom1 =
  // this->m_pOptions->m_vpGeometries[m_pOptions->m_couplings(i, 2)];

  //	PtrS<ParameterSet> pMat = make_shared<ParameterSet>();
  //	pMat->AddParameter(ParameterSet::Param_StretchK, 1e6);
  //
  //	Node* node0 = pGeom0->Nodes()[m_pOptions->m_couplings(i, 1)];
  //	Node* node1 = pGeom1->Nodes()[m_pOptions->m_couplings(i, 3)];
  //	Vector3d pos0 = node0->Traits().Vector3d(Tag::Position_X);
  //	Vector3d pos1 = node1->Traits().Vector3d(Tag::Position_X);

  //	this->m_venerEle[i] = PtrS<IEnergyElement>(new
  // EnergyElement_SpringBetwNodes(this, node0, node1, pMat));
  //}

  // Here make a sixDjoint between the two rigids
  // Create energy elements
  int numEneElem = this->m_pOptions->m_couplings.rows();

  // read some files from now, so no need to read them in every iteration. Also
  // make the matrices with the joint positions for the vert-rib case, and the
  // matrices with the index of the sternum, in whose position we are going to
  // place the joint for the rib-sternum case.
  // MatrixXd mV0;
  // MatrixXi mT0;
  // Serializer_Mesh::ReadSurface_OBJ(Utils::getAbsolutePath("C:/workspace/rainbow/christos-examples/Non-Working/inProgress/SI16/OrientedBones/Sternum.obj",
  // Utils::getCurrentDirectory()), mV0, mT0);
  //
  //
  // const std::string fileRibVertCouplingsLeft =
  // "C:/workspace/rainbow/christos-examples/Non-Working/inProgress/numpyResults/PointsJointsToPhySim/ribVertCouplingsLeft.csv";
  // MatrixXd jointPosRibVertCouplingsLeft(12, 3);
  // bool readFileRibVertCouplingsLeft =
  // Utils::readMatrix_CSV(fileRibVertCouplingsLeft,
  // jointPosRibVertCouplingsLeft);

  // const std::string fileRibVertCouplingsRight =
  // "C:/workspace/rainbow/christos-examples/Non-Working/inProgress/numpyResults/PointsJointsToPhySim/ribVertCouplingsRight.csv";
  // MatrixXd jointPosRibVertCouplingsRight(12, 3);
  // bool readRibVertCouplingsRight =
  // Utils::readMatrix_CSV(fileRibVertCouplingsRight,
  // jointPosRibVertCouplingsRight);

  // const std::string fileRibSternCouplingsLeft =
  // "C:/workspace/rainbow/christos-examples/Non-Working/inProgress/numpyResults/PointsJointsToPhySim/ribSternCouplingsLeft.csv";
  // MatrixXd IndexOfSternumRibSternCouplingsLeft(10, 1);
  // bool readRibSternCouplingsLeft =
  // Utils::readMatrix_CSV(fileRibSternCouplingsLeft,
  // IndexOfSternumRibSternCouplingsLeft);

  // const std::string fileRibSternCouplingsRight =
  // "C:/workspace/rainbow/christos-examples/Non-Working/inProgress/numpyResults/PointsJointsToPhySim/ribSternCouplingsRight.csv";
  // MatrixXd IndexOfSternumRibSternCouplingsRight(10, 1);
  // bool readRibSternCouplingsRight =
  // Utils::readMatrix_CSV(fileRibSternCouplingsRight,
  // IndexOfSternumRibSternCouplingsRight);

  /////////////the above code is used for the old patient with the anattemplate.
  /// The code below is to be used for hameds template with/out shoulder,
  /// comment appropriately

  const std::string fileRibVertCouplingsLeft =
      "C:/workspace/homework/simvizPhySim/assets/morphedPatients/" +
      this->m_pOptions->m_patientNumber + "/leftVertRibJointPos.csv";
  MatrixXd jointPosRibVertCouplingsLeft(12, 3);
  bool readFileRibVertCouplingsLeft = IOUtils::readMatrix_CSV(
      fileRibVertCouplingsLeft, jointPosRibVertCouplingsLeft);

  const std::string fileRibVertCouplingsRight =
      "C:/workspace/homework/simvizPhySim/assets/morphedPatients/" +
      this->m_pOptions->m_patientNumber + "/rightVertRibJointPos.csv";
  MatrixXd jointPosRibVertCouplingsRight(12, 3);
  bool readRibVertCouplingsRight = IOUtils::readMatrix_CSV(
      fileRibVertCouplingsRight, jointPosRibVertCouplingsRight);

  const std::string fileRibSternCouplingsLeft =
      "C:/workspace/homework/simvizPhySim/assets/morphedPatients/" +
      this->m_pOptions->m_patientNumber + "/leftRibSternJointPos.csv";
  MatrixXd jointPosRibSternCouplingsLeft(10, 3);
  bool readRibSternCouplingsLeft = IOUtils::readMatrix_CSV(
      fileRibSternCouplingsLeft, jointPosRibSternCouplingsLeft);

  const std::string fileRibSternCouplingsRight =
      "C:/workspace/homework/simvizPhySim/assets/morphedPatients/" +
      this->m_pOptions->m_patientNumber + "/rightRibSternJointPos.csv";
  MatrixXd jointPosRibSternCouplingsRight(10, 3);
  bool readRibSternCouplingsRight = IOUtils::readMatrix_CSV(
      fileRibSternCouplingsRight, jointPosRibSternCouplingsRight);

  ///////  to add the shoulder joints Pos

  const std::string fileShoulderCouplingsLeft =
      "C:/workspace/homework/simvizPhySim/assets/morphedPatients/" +
      this->m_pOptions->m_patientNumber + "/leftShoulderJointPosToExport.csv";
  MatrixXd jointPosShoulderLeft(3, 3);
  bool readjointPosShoulderLeft =
      IOUtils::readMatrix_CSV(fileShoulderCouplingsLeft, jointPosShoulderLeft);

  const std::string fileShoulderCouplingsRight =
      "C:/workspace/homework/simvizPhySim/assets/morphedPatients/" +
      this->m_pOptions->m_patientNumber + "/RightShoulderJointPosToExport.csv";
  MatrixXd jointPosShoulderRight(3, 3);
  bool readjointPosShoulderRight = IOUtils::readMatrix_CSV(
      fileShoulderCouplingsRight, jointPosShoulderRight);

  int nbVertJoints = 18 - 1;
  this->m_venerEle.resize(numEneElem);
  for (int i = 0; i < numEneElem; ++i) {
    Matrix3d R0;
    R0.setIdentity();
    MatrixXd K(6, 6);
    // K.setIdentity();
    // K = K * 10000;
    // K(3, 3) = 1000;
    // K(4, 4) = 2000;
    // K(5, 5) = 4000;
    K = m_pOptions->m_params[i];
    if (i < nbVertJoints) {
      this->m_venerEle[i] = PtrS<IEnergyElement>(
          new EnergyElement_SixDJoint(this,
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 1)]),
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 0)]),
                                      K));
    } else if (i < nbVertJoints + 12) {
      Vector3d jointPos;
      jointPos[0] = jointPosRibVertCouplingsLeft(i - nbVertJoints, 0);
      jointPos[1] = jointPosRibVertCouplingsLeft(i - nbVertJoints, 1);
      jointPos[2] = jointPosRibVertCouplingsLeft(i - nbVertJoints, 2);
      /*std::cout << jointPos << std::endl;*/

      this->m_venerEle[i] = PtrS<IEnergyElement>(
          new EnergyElement_SixDJoint(this,
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 0)]),
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 1)]),
                                      K, R0, jointPos));
    } else if (i < nbVertJoints + 12 + 12) {
      Vector3d jointPos;
      jointPos[0] = jointPosRibVertCouplingsRight(i - nbVertJoints - 12, 0);
      jointPos[1] = jointPosRibVertCouplingsRight(i - nbVertJoints - 12, 1);
      jointPos[2] = jointPosRibVertCouplingsRight(i - nbVertJoints - 12, 2);
      // std::cout << jointPos << std::endl;
      this->m_venerEle[i] = PtrS<IEnergyElement>(
          new EnergyElement_SixDJoint(this,
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 0)]),
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 1)]),
                                      K, R0, jointPos));
    }
    //// box of code to be commented if necessary starts here
    /*else if (i < nbVertJoints + 12 + 12 + 10) {
                    int index = (int)IndexOfSternumRibSternCouplingsLeft(i -
    nbVertJoints - 12 - 12, 0); Vector3d jointPos; jointPos[0] = mV0(index, 0);
                    jointPos[1] = mV0(index, 1);
                    jointPos[2] = mV0(index, 2);
                    std::cout << "SternumRibSternCouplingsLeft"<<jointPos <<
    std::endl; this->m_venerEle[i] = PtrS<IEnergyElement>(new
    EnergyElement_SixDJoint(this,
    dynamic_pointer_cast<KEleRigidBody3D>(vpEle[m_pOptions->m_couplings(i, 0)]),
                            dynamic_pointer_cast<KEleRigidBody3D>(vpEle[m_pOptions->m_couplings(i,
    1)]), K, R0, jointPos));
    }
    else {
                    int index = (int)IndexOfSternumRibSternCouplingsRight(i -
    nbVertJoints - 12 - 12 - 10, 0); Vector3d jointPos; jointPos[0] = mV0(index,
    0); jointPos[1] = mV0(index, 1); jointPos[2] = mV0(index, 2); std::cout <<
    "SternumRibSternCouplingsRight" << jointPos << std::endl;
                    this->m_venerEle[i] = PtrS<IEnergyElement>(new
    EnergyElement_SixDJoint(this,
    dynamic_pointer_cast<KEleRigidBody3D>(vpEle[m_pOptions->m_couplings(i, 0)]),
                            dynamic_pointer_cast<KEleRigidBody3D>(vpEle[m_pOptions->m_couplings(i,
    1)]), K, R0, jointPos));
    }		*/
    /////////// comment below code for case of patient with ant template.
    /// Comment code above in the box for case with hameds template and shoulder
    else if (i < nbVertJoints + 12 + 12 + 10) {
      Vector3d jointPos;
      jointPos[0] =
          jointPosRibSternCouplingsLeft(i - nbVertJoints - 12 - 12, 0);
      jointPos[1] =
          jointPosRibSternCouplingsLeft(i - nbVertJoints - 12 - 12, 1);
      jointPos[2] =
          jointPosRibSternCouplingsLeft(i - nbVertJoints - 12 - 12, 2);
      std::cout << "SternumRibSternCouplingsLeft" << jointPos << std::endl;
      this->m_venerEle[i] = PtrS<IEnergyElement>(
          new EnergyElement_SixDJoint(this,
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 0)]),
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 1)]),
                                      K, R0, jointPos));
    } else if (i < nbVertJoints + 12 + 12 + 10 + 10) {
      Vector3d jointPos;
      jointPos[0] =
          jointPosRibSternCouplingsRight(i - nbVertJoints - 12 - 12 - 10, 0);
      jointPos[1] =
          jointPosRibSternCouplingsRight(i - nbVertJoints - 12 - 12 - 10, 1);
      jointPos[2] =
          jointPosRibSternCouplingsRight(i - nbVertJoints - 12 - 12 - 10, 2);
      std::cout << "SternumRibSternCouplingsRight" << jointPos << std::endl;
      this->m_venerEle[i] = PtrS<IEnergyElement>(
          new EnergyElement_SixDJoint(this,
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 0)]),
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 1)]),
                                      K, R0, jointPos));
    } else if (i < nbVertJoints + 12 + 12 + 10 + 10 + 3) {
      Vector3d jointPos;
      jointPos[0] =
          jointPosShoulderLeft(i - nbVertJoints - 12 - 12 - 10 - 10, 0);
      jointPos[1] =
          jointPosShoulderLeft(i - nbVertJoints - 12 - 12 - 10 - 10, 1);
      jointPos[2] =
          jointPosShoulderLeft(i - nbVertJoints - 12 - 12 - 10 - 10, 2);
      this->m_venerEle[i] = PtrS<IEnergyElement>(
          new EnergyElement_SixDJoint(this,
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 0)]),
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 1)]),
                                      K, R0, jointPos));
    } else if (i < nbVertJoints + 12 + 12 + 10 + 10 + 3 + 3) {
      Vector3d jointPos;
      jointPos[0] =
          jointPosShoulderRight(i - nbVertJoints - 12 - 12 - 10 - 10 - 3, 0);
      jointPos[1] =
          jointPosShoulderRight(i - nbVertJoints - 12 - 12 - 10 - 10 - 3, 1);
      jointPos[2] =
          jointPosShoulderRight(i - nbVertJoints - 12 - 12 - 10 - 10 - 3, 2);
      this->m_venerEle[i] = PtrS<IEnergyElement>(
          new EnergyElement_SixDJoint(this,
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 0)]),
                                      dynamic_pointer_cast<KEleRigidBody3D>(
                                          vpEle[m_pOptions->m_couplings(i, 1)]),
                                      K, R0, jointPos));
    }
  }

  // this->m_venerEle.clear();

  // Create constraint sets

  this->m_vconsSet.clear();

  // Create mass elements

  this->m_vmassEle.resize(numRB);
  for (int i = 0; i < numRB; ++i) {
    PtrS<ParameterSet> pPar = this->m_pOptions->m_vpMaterials[i];
    Geometry* pGeom = this->m_pOptions->m_vpGeometries[i].get();
    this->m_vmassEle[i] =
        PtrS<IMassElement>(new MassElement_RigidBody(this, pGeom, pPar));
  }
}

void Simulable_RB::SetState(const VectorXd& vs) {
  Simulable::SetState(vs);

  // Rigid body mass depends on
  // the deformad configuration

  this->DirtyMass();
}

void Simulable_RB::SetDOFVector(const VectorXd& vx, Tag s) {
  Simulable::SetDOFVector(vx, s);

  // Rigid body mass depends on
  // the deformad configuration

  this->DirtyMass();
}

}  // namespace PhySim