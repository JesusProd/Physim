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


#include <PhySim/Geometry/Geometry.h>

#include <numeric>
#include <algorithm>

namespace PhySim
{
	using namespace std;
	using namespace Eigen;

	class IInterpolation
	{
	public:
		IInterpolation() { };
		virtual ~IInterpolation() { };

		/**
		* Returns the dimensionality of the manifold basis, e.g.,
		* 2D for a triangle in R^3 and 3D for a hexahedron in R^3
		*/
		virtual int DimBasis() const = 0;

		/**
		* Returns the number of control points for the interpolation.
		*/
		virtual int NumPoints() const = 0;

		virtual bool IsValidParametric(const VectorXd& vp) const = 0;

		virtual void Compute_Weights(const VectorXd& vp, VectorXd& vW) const = 0;
		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const = 0;

		virtual void Interpolate(const MatrixXd& mV, const MatrixXd& mP, MatrixXd& mI) const = 0;

	};

	class Interpolation : public IInterpolation
	{
	public:

		Interpolation() { };
		virtual ~Interpolation() { };

		virtual int DimBasis() const override = 0;
		virtual int NumPoints() const override = 0;

		virtual bool IsValidParametric(const VectorXd& vp) const override = 0;

		virtual void Compute_Weights(const VectorXd& vp, VectorXd& vW) const override = 0;
		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override = 0;

		virtual void Interpolate(const MatrixXd& mV, const MatrixXd& mP, MatrixXd& mI) const override;

	};

	class Interpolation_Identity : public Interpolation
	{
	public:
		static PtrS<Interpolation_Identity> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Interpolation_Identity());
			return PINSTANCE;
		}

	private:
		static PtrS<Interpolation_Identity>	PINSTANCE;

	public:
		virtual int DimBasis() const override { return 0; };

		virtual int NumPoints() const override { return 1; };

		virtual bool IsValidParametric(const VectorXd& vp) const override
		{
			return true;
		}

		virtual void Compute_Weights(const VectorXd& vp, VectorXd& vW) const override
		{
			vW.resize(1);
			vW(1) = 1.0;
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			mB = MatrixXd();
		}

	};

	class Interpolation_Linear3 : public Interpolation
	{
	public:
		static PtrS<Interpolation_Linear3> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Interpolation_Linear3());
			return PINSTANCE;
		}

	private:
		static PtrS<Interpolation_Linear3>	PINSTANCE;

	public:
		virtual int DimBasis() const override { return 2; };

		virtual int NumPoints() const override { return 3; };

		virtual bool IsValidParametric(const VectorXd& vp) const override
		{
			if (vp.size() != 2)
				return false;

			if (vp(0) < 0 || vp(0) > 1 ||
				vp(1) < 0 || vp(1) > 1 ||
				vp(0) + vp(1) > 1)
				return false;

			return true;
		}

		virtual void Compute_Weights(const VectorXd& vp, VectorXd& vW) const override
		{
			assert(vp.size() == this->DimBasis());

			vW.resize(3);
			vW(1) = vp(0);
			vW(2) = vp(1);
			vW(0) = 1.0f - vp(0) - vp(1);
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			mB.resize(3, 2);
			mB.row(0) = Vector2d(-1, -1);
			mB.row(1) = Vector2d(1, 0);
			mB.row(2) = Vector2d(0, 1);
		}

	};

	class Interpolation_Quadratic3 : public Interpolation_Linear3
	{
	public:
		static PtrS<Interpolation_Quadratic3> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Interpolation_Quadratic3());
			return PINSTANCE;
		}

	private:
		static PtrS<Interpolation_Quadratic3>	PINSTANCE;

	public:
		virtual int DimBasis() const override { return 2; };

		virtual int NumPoints() const override { return 6; };

		virtual void Compute_Weights(const VectorXd& vp, VectorXd& vW) const override
		{
			throw PhySim::exception("Not implemented");
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			throw PhySim::exception("Not implemented");
		}

	};

	class Interpolation_Bilinear : public Interpolation
	{
	public:
		static PtrS<Interpolation_Bilinear> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Interpolation_Bilinear());
			return PINSTANCE;
		}

	private:
		static PtrS<Interpolation_Bilinear> PINSTANCE;

	public:
		virtual int DimBasis() const override { return 2; };

		virtual int NumPoints() const override { return 4; };

		virtual bool IsValidParametric(const VectorXd& vp) const override
		{
			if (vp.size() != 2)
				return false;

			if (vp(0) < -1 || vp(0) > 1 ||
				vp(1) < -1 || vp(1) > 1)
				return false;

			return true;
		}

		virtual void Compute_Weights(const VectorXd& vp, VectorXd& vW) const override
		{
			vW.resize(4);
			Real factor = 1.0 / 4.0;
			vW(0) = (1 - vp[0])*(1 - vp[1]);
			vW(1) = (1 + vp[0])*(1 - vp[1]);
			vW(2) = (1 + vp[0])*(1 + vp[1]);
			vW(3) = (1 - vp[0])*(1 + vp[1]);
			vW = factor * vW;
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			mB.resize(4, 2);
			Real factor = 1.0 / 4.0;
			// DNDp1
			mB(0, 0) = -(1 - vp[1]);
			mB(1, 0) = +(1 - vp[1]);
			mB(2, 0) = +(1 + vp[1]);
			mB(3, 0) = -(1 + vp[1]);
			// DNDp1
			mB(0, 1) = -(1 - vp[0]);
			mB(1, 1) = -(1 + vp[0]);
			mB(2, 1) = +(1 + vp[0]);
			mB(3, 1) = +(1 - vp[0]);
			mB = factor * mB;
		}

	};

	class Interpolation_Biquadratic : public Interpolation_Bilinear
	{
	public:
		static PtrS<Interpolation_Biquadratic> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Interpolation_Biquadratic());
			return PINSTANCE;
		}

	private:
		static PtrS<Interpolation_Biquadratic> PINSTANCE;

	public:
		virtual int DimBasis() const override { return 2; };

		virtual int NumPoints() const override { return 4; };
		
		virtual void Compute_Weights(const VectorXd& vp, VectorXd& vW) const override
		{
			throw PhySim::exception("Not implemented");
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			throw PhySim::exception("Not implemented");
		}

	};

	class Interpolation_Linear4 : public Interpolation
	{
	public:
		static PtrS<Interpolation_Linear4> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Interpolation_Linear4());
			return PINSTANCE;
		}

	private:
		static PtrS<Interpolation_Linear4>	PINSTANCE;

	public:
		virtual int DimBasis() const override { return 3; };

		virtual int NumPoints() const override { return 4; };

		virtual bool IsValidParametric(const VectorXd& vp) const override
		{
			if (vp.size() != 3)
				return false;

			if (vp(0) < 0 || vp(0) > 1 ||
				vp(1) < 0 || vp(1) > 1 ||
				vp(2) < 0 || vp(2) > 1 ||
				vp(0) + vp(1) + vp(2) > 1)
				return false;

			return true;
		}

		virtual void Compute_Weights(const VectorXd& vp, VectorXd& vW) const override
		{
			vW.resize(4);
			vW(1) = vp(0);
			vW(2) = vp(1);
			vW(3) = vp(2);
			vW(0) = 1.0 - vp(0) - vp(1) - vp(2);
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			mB.resize(4, 3);
			mB.row(0) = Vector3d(-1, -1, -1);
			mB.row(1) = Vector3d(1, 0, 0);
			mB.row(2) = Vector3d(0, 1, 0);
			mB.row(3) = Vector3d(0, 0, 1);
		}

	};

	class Interpolation_Quadratic4 : public Interpolation_Linear4
	{

	public:
		static PtrS<Interpolation_Quadratic4> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Interpolation_Quadratic4());
			return PINSTANCE;
		}

	private:
		static PtrS<Interpolation_Quadratic4> PINSTANCE;

	public:
		virtual int DimBasis() const override { return 3; };

		virtual int NumPoints() const override { return 10; };

		virtual void Compute_Weights(const VectorXd& vp, VectorXd& vW) const override
		{
			vW.resize(10);
			Real t0 = 1.0 - vp(0) - vp(1) - vp(2);
			Real t1 = vp(0);
			Real t2 = vp(1);
			Real t3 = vp(2);
			vW(0) = t0 * (2 * t0 - 1);
			vW(1) = t1 * (2 * t1 - 1);
			vW(2) = t2 * (2 * t2 - 1);
			vW(3) = t3 * (2 * t3 - 1);
			vW(4) = 4 * t0*t1;
			vW(5) = 4 * t1*t2;
			vW(6) = 4 * t2*t0;
			vW(7) = 4 * t0*t3;
			vW(8) = 4 * t1*t3;
			vW(9) = 4 * t2*t3;
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			Real t0 = 1.0 - vp(0) - vp(1) - vp(2);
			Real t1 = vp(0);
			Real t2 = vp(1);
			Real t3 = vp(2);

			MatrixXd DNDt(10, 4);
			DNDt.row(0) = Vector4d(4 * t0 - 1, 0, 0, 0);
			DNDt.row(1) = Vector4d(0, 4 * t1 - 1, 0, 0);
			DNDt.row(2) = Vector4d(0, 0, 4 * t2 - 1, 0);
			DNDt.row(3) = Vector4d(0, 0, 0, 4 * t3 - 1);
			DNDt.row(4) = Vector4d(4 * t1, 4 * t0, 0, 0);
			DNDt.row(5) = Vector4d(0, 4 * t2, 4 * t1, 0);
			DNDt.row(6) = Vector4d(4 * t2, 0, 4 * t0, 0);
			DNDt.row(7) = Vector4d(4 * t3, 0, 0, 4 * t0);
			DNDt.row(8) = Vector4d(0, 4 * t3, 0, 4 * t1);
			DNDt.row(9) = Vector4d(0, 0, 4 * t3, 4 * t2);

			MatrixXd DtDp(4, 3);
			DtDp.row(0) = Vector3d(-1, -1, -1);
			DtDp.row(1) = Vector3d(1, 0, 0);
			DtDp.row(2) = Vector3d(0, 1, 0);
			DtDp.row(3) = Vector3d(0, 0, 1);

			mB = DNDt * DtDp;
		}

	};

	class Interpolation_Trilinear : public Interpolation
	{

	public:
		static PtrS<Interpolation_Trilinear> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Interpolation_Trilinear());
			return PINSTANCE;
		}

	private:
		static PtrS<Interpolation_Trilinear>	PINSTANCE;

	public:
		virtual int DimBasis() const override { return 3; };

		virtual int NumPoints() const override { return 8; };

		virtual bool IsValidParametric(const VectorXd& vp) const override
		{
			if (vp.size() != 3)
				return false;

			if (vp(0) < -1 || vp(0) > 1 ||
				vp(1) < -1 || vp(1) > 1 ||
				vp(1) < -1 || vp(1) > 1)
				return false;

			return true;
		}

		virtual void Compute_Weights(const VectorXd& vp, VectorXd& vW) const override
		{
			vW.resize(8);
			Real factor = 1.0 / 8.0;
			vW(0) = (1 - vp[0])*(1 - vp[1])*(1 - vp[2]);
			vW(1) = (1 + vp[0])*(1 - vp[1])*(1 - vp[2]);
			vW(2) = (1 + vp[0])*(1 + vp[1])*(1 - vp[2]);
			vW(3) = (1 - vp[0])*(1 + vp[1])*(1 - vp[2]);
			vW(4) = (1 - vp[0])*(1 - vp[1])*(1 + vp[2]);
			vW(5) = (1 + vp[0])*(1 - vp[1])*(1 + vp[2]);
			vW(6) = (1 + vp[0])*(1 + vp[1])*(1 + vp[2]);
			vW(7) = (1 - vp[0])*(1 + vp[1])*(1 + vp[2]);
			vW = factor * vW;
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			mB.resize(8, 3);
			Real factor = 1.0 / 8.0;
			// DNDp1
			mB(0, 0) = -(1 - vp[1])*(1 - vp[2]);
			mB(1, 0) = +(1 - vp[1])*(1 - vp[2]);
			mB(2, 0) = +(1 + vp[1])*(1 - vp[2]);
			mB(3, 0) = -(1 + vp[1])*(1 - vp[2]);
			mB(4, 0) = -(1 - vp[1])*(1 + vp[2]);
			mB(5, 0) = +(1 - vp[1])*(1 + vp[2]);
			mB(6, 0) = +(1 + vp[1])*(1 + vp[2]);
			mB(7, 0) = -(1 + vp[1])*(1 + vp[2]);
			// DNDp1
			mB(0, 1) = -(1 - vp[0])*(1 - vp[2]);
			mB(1, 1) = -(1 + vp[0])*(1 - vp[2]);
			mB(2, 1) = +(1 + vp[0])*(1 - vp[2]);
			mB(3, 1) = +(1 - vp[0])*(1 - vp[2]);
			mB(4, 1) = -(1 - vp[0])*(1 + vp[2]);
			mB(5, 1) = -(1 + vp[0])*(1 + vp[2]);
			mB(6, 1) = +(1 + vp[0])*(1 + vp[2]);
			mB(7, 1) = +(1 - vp[0])*(1 + vp[2]);
			// DNDp2
			mB(0, 2) = -(1 - vp[0])*(1 - vp[1]);
			mB(1, 2) = -(1 + vp[0])*(1 - vp[1]);
			mB(2, 2) = -(1 + vp[0])*(1 + vp[1]);
			mB(3, 2) = -(1 - vp[0])*(1 + vp[1]);
			mB(4, 2) = +(1 - vp[0])*(1 - vp[1]);
			mB(5, 2) = +(1 + vp[0])*(1 - vp[1]);
			mB(6, 2) = +(1 + vp[0])*(1 + vp[1]);
			mB(7, 2) = +(1 - vp[0])*(1 + vp[1]);
			mB = factor * mB;
		}

	};

	class Interpolation_Triquadratic : public Interpolation_Trilinear
	{
	public:
		static PtrS<Interpolation_Triquadratic> Instance()
		{
			if (PINSTANCE == NULL)
				PINSTANCE.reset(new Interpolation_Triquadratic());
			return PINSTANCE;
		}

	private:
		static PtrS<Interpolation_Triquadratic>	PINSTANCE;

	public:
		virtual int DimBasis() const override { return 3; };

		virtual int NumPoints() const override { return 8; };

		virtual void Compute_Weights(const VectorXd& vp, VectorXd& vW) const override
		{
			throw PhySim::exception("Not implemented");
		}

		virtual void Compute_Partial(const VectorXd& vp, MatrixXd& mB) const override
		{
			throw PhySim::exception("Not implemented");
		}

	};

	class Interpolation_Chen : public Interpolation_Trilinear
	{
	protected:
		vector<vector<Matrix3d>> m_vmNij; // This matrix vectors store the precomputed tensor weights used for the interpolation
		int m_subdiv;
		int m_fineNodesPerDime;
		int m_fineElemsPerDime;
		int m_fineNodesPerElem;
		int m_fineElemsPerElem;
		MatrixXi m_vHCoar;
		MatrixXi m_mHFine;
		MatrixXd m_mVFine;

	public:
		Interpolation_Chen(const vector<vector<Matrix3d>>& vmnij, int subdiv = 1);

		virtual ~Interpolation_Chen() { };

		virtual void Interpolate(const MatrixXd& mV, const MatrixXd& mP, MatrixXd& mI) const override;

	public:

		int ChooseSubelement(const VectorXd& vp) const;

	};

}
