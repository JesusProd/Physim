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


#include <PhySim/Geometry/Meshes/Mesh_Frame.h>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class Rod;

	class Mesh_DER : public Mesh_Frame
	{
	public:
		Mesh_DER();
		Mesh_DER(const Mesh_DER& toCopy);
		Mesh_DER(const vector<PtrS<Rod>>& vrods, Tag trait, Real tol);
		void Init(const vector<PtrS<Rod>>& vrods, Tag trait, Real tol);
		Mesh_DER(const MatrixXd& mV, const MatrixXi& mE, const vector<Frame3d>& vF = vector<Frame3d>(), const vector<Vector2d>& vr = vector<Vector2d>());
		void Init(const MatrixXd& mV, const MatrixXi& mE, const vector<Frame3d>& vF = vector<Frame3d>(), const vector<Vector2d>& vr = vector<Vector2d>());
		virtual ~Mesh_DER(void);

		virtual Mesh* Clone() const override { return new Mesh_DER(*this); }
		
		virtual Frame3d ComputeMatFrame(int i, Tag s);
		virtual void ComputeMatFrameVector(vector<Frame3d>&, Tag s);
		virtual void ComputeMatFrameArrows(MatrixXd& m0, MatrixXd& mN, MatrixXd& mB, Tag s);

		virtual void GetRefFrameVector(vector<Frame3d>& vF, Tag s) const;
		virtual void SetRefFrameVector(const vector<Frame3d>& vF, Tag s);

		virtual void GetMatAngleVector(vector<Real>& vF, Tag s) const;
		virtual void SetMatAngleVector(const vector<Real>& vF, Tag s);

		virtual void GetRefTwistVector(vector<Real>& vF) const;
		virtual void SetRefTwistVector(const vector<Real>& vF);

		virtual void GetRotEulerVector(vector<Vector3d>& vF) const;
		virtual void SetRotEulerVector(const vector<Vector3d>& vF);

		virtual void GetRotMatrixVector(vector<Matrix3d>& vF) const;
		virtual void SetRotMatrixVector(const vector<Matrix3d>& vF);

		virtual Frame3d& RefFrame(int i, Tag s);
		virtual Real& MatAngle(int i, Tag s);
		virtual Real& RefTwist(int i);
		virtual Vector3d& RotEuler(int i);
		virtual Matrix3d& RotMatrix(int i);

		virtual void AdaptReferenceFrame(int i, Tag s);
		virtual void ComputeReferenceTwist(int i);
		virtual void UpdateConnectionRotation(int i);

		virtual void AdaptReferenceFrames(Tag s);
		virtual void ComputeReferenceTwists();
		virtual void UpdateConnectionRotations();

	private:
		void UpdateMetadata();
		void FreeMetadata();

	};

}
